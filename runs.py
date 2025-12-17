import os
import sys

import json
import gc
import platform
import time
import multiprocessing
from functools import partial
from typing import Optional
from ml.strategies import Execution
from ml.event_sources import Timer
from ml.tracer import Tracer, analyze_trace_log
from ml.enums import OnFullBehavior, LogLevel
from ml import data
from diagrams.serializer import DiagramSerializer
from diagrams.engine import MainWindow
from diagrams.optimization import run_simulated_annealing
from datetime import datetime
from monitor.plot_server import main as run_plot_server
from description import Top
import conf
import types
import sim_conf

class Configuration:
    """A simple object to hold and pass simulation configuration."""
    def __init__(self, *configurations):
        """Loads and merges variables from one or more configuration modules."""
        for configuration in configurations:
            # Load all variables from the conf module into this object
            for key, value in vars(configuration).items():
                if not key.startswith('__'):
                    # Only copy attributes that are not modules or functions, as they are unpicklable.
                    # This prevents multiprocessing errors when passing the configuration.
                    is_module = isinstance(value, types.ModuleType)
                    if not is_module:
                        setattr(self, key, value)

# --- Log constants ---
MAIN_COMPONENT_ID = "MAIN"
LOG_EVENT_START = "START"
LOG_EVENT_FAILURE = "FAILURE"
LOG_EVENT_SUCCESS = "SUCCESS"
LOG_EVENT_INTERRUPT = "INTERRUPT"
LOG_DETAIL_KEY_MESSAGE = "message"
MSG_SIM_START = "Starting simulation threads on {}"
MSG_SIM_FAILURE = "Simulation finished with errors."
MSG_SIM_SUCCESS = "Simulation finished."
MSG_INTERRUPT = "Ctrl+C received, stopping simulation."
MSG_PLOT_SERVER_STARTING = "Starting plot server process..."
MSG_SHUTDOWN_COMPLETE = "All components terminated. Flushing final logs."

# --- Diagram utility messages ---
MSG_DIAG_INSTANTIATING = "Instantiating the simulation model..."
MSG_DIAG_PART_FOUND = "Found part to serialize: '{}'"
MSG_DIAG_SERIALIZING = "Serializing part structure to JSON..."
MSG_DIAG_EXPORT_SUCCESS = "\nSuccessfully exported the structure of '{}' to '{}'"
MSG_DIAG_EXPORT_ERROR = "\nError: {}"
MSG_DIAG_FILE_NOT_FOUND = "Error: JSON file '{}' not found. Please run with 'export' argument first."
MSG_DIAG_LOADING = "Loading diagram from '{}'..."
MSG_DIAG_IMPORT_SUCCESS = "Diagram imported successfully."
MSG_DIAG_IMPORT_ERROR = "Error importing diagram: {}"

def set_high_priority():
    """
    Sets the process priority to high to improve timer accuracy for real-time simulation.
    This is crucial for reducing OS-level scheduling jitter.
    """
    if platform.system() == "Windows":
        # On Windows, 0x80 is "High Priority".
        # Note: This requires the 'psutil' library.
        # You may need to run: pip install psutil
        import psutil
        p = psutil.Process(os.getpid())
        p.nice(psutil.HIGH_PRIORITY_CLASS)
    else:
        # On Linux/macOS, a lower nice value means higher priority.
        os.nice(-10)

def simulate(trace_filename=None, error_filename=None):

    # Create a configuration object that holds all parameters.
    proj_conf = Configuration(conf, sim_conf)

    plot_server_process = None
    top = None
    timer = None
    try:
        if proj_conf.PLOT:
            print(MSG_PLOT_SERVER_STARTING)
            plot_server_process = multiprocessing.Process(
                target=run_plot_server,
                args=(proj_conf.POSITION_GRAPH_BOUNDARIES, proj_conf.SPEED_GRAPH_BOUNDARIES)
            )
            plot_server_process.daemon = True  # Ensure the process exits with the main script
            plot_server_process.start()
            # Wait a moment for the server to initialize and start listening
            time.sleep(1.5)

        log_queue = None
        # Conditionally start the tracer based on configuration
        error_queue = None
        if proj_conf.TRACER_ENABLED:
            if proj_conf.PARALLEL_EXECUTION_MODE == 'process':
                log_queue = multiprocessing.Queue()
                error_queue = multiprocessing.Queue()
            Tracer.start(
                level=LogLevel.TRACE,
                flush_interval_seconds=5.0,
                output_file=trace_filename,
                log_to_console=True,
                error_file=error_filename,
                log_queue=log_queue,
                error_queue=error_queue
            )

        top_execution = Execution(name='parallel_multirotor_execution',
            parallelization_condition=lambda part: part.get_identifier() == 'multirotor',
            mode=proj_conf.PARALLEL_EXECUTION_MODE,
            log_queue=log_queue,
            error_queue=error_queue
        )

        data.configure(proj_conf)

        top = Top('top', conf=proj_conf, execution_strategy=top_execution)

        # Initialize the simulation to set up pybullet and get the time step
        top.init()

        # Create the external timer event source
        # The `on_full` policy is determined by the REAL_TIME_SIMULATION flag.
        # In real-time mode (`FAIL`), the simulation stops if it cannot keep up
        # with the timer, which is crucial for exposing performance bottlenecks.
        # In non-real-time mode (`DROP`), it drops events to keep running,
        # which can be useful for non-critical runs or debugging.
        on_full_behavior = OnFullBehavior.FAIL if proj_conf.REAL_TIME_SIMULATION else OnFullBehavior.DROP
        timer = Timer(identifier='physics_timer', interval_seconds=proj_conf.TIME_STEP, on_full=on_full_behavior)

        # Connect the timer to the simulation's main event queue
        top.connect_event_source(timer, 'time_event_in')

        Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_START, {LOG_DETAIL_KEY_MESSAGE: MSG_SIM_START.format(datetime.now())})
        
        # Disable the garbage collector to prevent unpredictable pauses during
        # the simulation, which can cause jitter and missed deadlines.
        gc.disable()

        # For real-time simulations, increase process priority to minimize
        # OS scheduler jitter, which can cause timer inaccuracies.
        if proj_conf.HIGH_PRIORITY:
            set_high_priority()
        
        # Start the simulation thread, which will spawn worker processes.
        top.start(stop_condition=lambda _: timer.stop_event_is_set())

        # If using persistent processes, block until all workers have confirmed they are initialized.
        # This is not needed for 'thread' mode.
        #if proj_conf.PARALLEL_EXECUTION_MODE == 'process':
        top.wait_for_ready()

        # Now that all components are ready, start the timer to begin event flow.
        timer.start()

        # Block here and wait for the main simulation thread to finish.
        top.wait()

        if top.get_exception() or timer.get_exception():
            Tracer.log(LogLevel.ERROR, MAIN_COMPONENT_ID, LOG_EVENT_FAILURE, {LOG_DETAIL_KEY_MESSAGE: MSG_SIM_FAILURE})
        else:
            Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_SUCCESS, {LOG_DETAIL_KEY_MESSAGE: MSG_SIM_SUCCESS})

    except KeyboardInterrupt:
        Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_INTERRUPT, {LOG_DETAIL_KEY_MESSAGE: MSG_INTERRUPT})
    finally:
        # This block ensures cleanup happens on normal exit or Ctrl+C.
        # Re-enable the garbage collector.
        gc.enable()

        # The order is important for a graceful shutdown and complete logging.
        # 1. Stop event sources first to prevent new work.
        if timer:
            timer.stop()
            timer.wait()

        # 2. Wait for the main simulation part to finish its loop, then terminate it.
        #    top.term() will handle shutting down worker processes.
        if top:
            top.wait()
            top.term()

        # 3. Terminate auxiliary processes.
        if plot_server_process and plot_server_process.is_alive():
            plot_server_process.terminate()

        # 4. Stop the tracer last. This is a blocking call that flushes all final logs.
        if proj_conf.TRACER_ENABLED:
            Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_SUCCESS, {LOG_DETAIL_KEY_MESSAGE: MSG_SHUTDOWN_COMPLETE})
            Tracer.stop()

def export_diagram(structure_filename=None, part_id=None):
    """
    Instantiates the Multirotor model, serializes its structure to JSON,
    and saves it to a file.

    Args:
        structure_filename (str): The path to save the JSON file.
        part_id (str): The dot-separated ID of the part to export (e.g., 'multirotor.controller').
    """
    print(MSG_DIAG_INSTANTIATING)
    proj_conf = Configuration(conf, sim_conf)
    top = Top('top', conf=proj_conf)
    
    # Traverse the hierarchy to find the requested part
    part = top
    if part_id:
        for child_id in part_id.split('.'):
            part = part.get_part(child_id)

    print(MSG_DIAG_PART_FOUND.format(part.get_full_identifier()))

    serializer = DiagramSerializer()
    try:
        print(MSG_DIAG_SERIALIZING)
        json_output = serializer.export_part_to_json(part)
        with open(structure_filename, 'w') as f:
            f.write(json_output)
        print(MSG_DIAG_EXPORT_SUCCESS.format(part.get_full_identifier(), structure_filename))
    except TypeError as e:
        print(MSG_DIAG_EXPORT_ERROR.format(e))

def import_diagram(structure_filename=None):
    """
    Loads a diagram from the JSON file and displays it in the editor.
    """
    if not os.path.exists(structure_filename):
        print(MSG_DIAG_FILE_NOT_FOUND.format(structure_filename))
        return

    print(MSG_DIAG_LOADING.format(structure_filename))
    from PyQt5.QtWidgets import QApplication
    with open(structure_filename, 'r') as f:
        json_data = f.read()

    # A QApplication instance is always required for a Qt app.
    qapp = QApplication(sys.argv)

    # --- Configure Optimizer ---
    # Use Simulated Annealing as the optimization strategy.
    sa_params = {
        'iterations': 2000,
        'initial_temp': 20.0,
        'cooling_rate': 0.995,
        'move_step_grid_units': 15,
        'intersection_weight': 100.0,
        'wirelength_weight': 0.1
    }
    configured_optimizer = partial(run_simulated_annealing, params=sa_params)

    # Create the main window with the optimizer and the serializer
    main_window = MainWindow(enable_logging=True, optimizer_func=configured_optimizer)
    serializer = DiagramSerializer()

    # Import the data and build the diagram
    try:
        serializer.import_part_from_json(json_data, main_window)
        print(MSG_DIAG_IMPORT_SUCCESS)
    except (ValueError, json.JSONDecodeError) as e:
        print(MSG_DIAG_IMPORT_ERROR.format(e))
        return

    # Start the application event loop to show the window.
    sys.exit(main_window.start())

def analyze_trace(trace_file: str, output_format: str = 'text', output_file: Optional[str] = None):
    """
    Parses and analyzes the trace log file generated by the simulation.

    Args:
        trace_file: Path to the simulation trace log file.
        output_format: The desired output format ('text', 'json', 'json:perfetto').
        output_file: Optional path to write the output to.
    """
    if not os.path.exists(trace_file):
        print(f"Error: Trace log file not found at '{trace_file}'")
        return

    analyze_trace_log(trace_file, output_format=output_format, output_file=output_file)