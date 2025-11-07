import os

import json
import gc
import platform
import threading
import time
import multiprocessing
from functools import partial
from typing import Optional
from PyQt5.QtWidgets import QApplication
from ml.strategies import sequential_execution, execute
from ml.event_sources import Timer
from ml.tracer import Tracer, analyze_trace_log
from ml.enums import OnFullBehavior, LogLevel
from diagrams.serializer import DiagramSerializer
from diagrams.engine import MainWindow
from diagrams.optimization import run_simulated_annealing
from datetime import datetime
from monitor.plot_server import main as run_plot_server
from description import Top
import conf

class Configuration:
    """A simple object to hold and pass simulation configuration."""
    def __init__(self):
        # Load all variables from the conf module into this object
        for key, value in vars(conf).items():
            if not key.startswith('__'):
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
def parallel_controller_execution(parent_part, scheduled_parts, strategy_event):
    """
    A custom execution strategy that runs the 'atas' mixer and all PID-based
    'Control_Element' parts in separate threads, while executing any other
    parts sequentially. This parallelizes the main computational load of the
    controller.
    """
    parallelizable_parts = []
    sequential_parts = []
    threads = []

    # Separate the parts that can be run in parallel from the others
    for part in scheduled_parts:
        part_id = part.get_identifier()
        if part_id == 'atas' or 'control_element' in part_id:
            parallelizable_parts.append(part)
        else:
            sequential_parts.append(part)

    creator_thread_name = threading.current_thread().name

    # Start each parallelizable part in its own thread
    for part in parallelizable_parts:
        thread = threading.Thread(target=execute, args=(part, creator_thread_name, strategy_event, parent_part.tick), name=f"{part.get_full_identifier()}_thread")
        thread.start()
        threads.append(thread)

    # Execute the rest of the parts sequentially in the main thread while parallel threads run
    if sequential_parts:
        sequential_execution(parent_part, sequential_parts, strategy_event)

    # Wait for all parallel threads to finish before the strategy returns
    for thread in threads:
        thread.join()


def parallel_toplevel_execution(parent_part, scheduled_parts, strategy_event):
    """
    A custom execution strategy that runs the main 'simulator' and 'multirotor'
    parts in parallel threads, as they are the main computational loads and
    are independent within a single step. Any other parts are run sequentially.
    """
    parallelizable_parts = []
    sequential_parts = []
    threads = []

    # Separate the parts that can be run in parallel
    for part in scheduled_parts:
        part_id = part.get_identifier()
        if part_id in ['simulator', 'multirotor']:
            parallelizable_parts.append(part)
        else:
            sequential_parts.append(part)

    creator_thread_name = threading.current_thread().name

    for part in parallelizable_parts:
        thread = threading.Thread(target=execute, args=(part, creator_thread_name, strategy_event, parent_part.tick), name=f"{part.get_full_identifier()}_thread")
        thread.start()
        threads.append(thread)

    # Execute sequential parts (if any) in the main thread while parallel threads run
    if sequential_parts:
        sequential_execution(parent_part, sequential_parts, strategy_event)

    for thread in threads:
        thread.join()

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
    sim_conf = Configuration()

    plot_server_process = None
    top = None
    try:
        if sim_conf.PLOT:
            print(MSG_PLOT_SERVER_STARTING)
            plot_server_process = multiprocessing.Process(
                target=run_plot_server,
                args=(sim_conf.POSITION_GRAPH_BOUNDARIES, sim_conf.SPEED_GRAPH_BOUNDARIES)
            )
            plot_server_process.daemon = True  # Ensure the process exits with the main script
            plot_server_process.start()
            # Wait a moment for the server to initialize and start listening
            time.sleep(1.5)

        # Conditionally start the tracer based on configuration
        if sim_conf.TRACER_ENABLED:
            Tracer.start(
                level=LogLevel.TRACE,
                flush_interval_seconds=5.0,
                output_file=trace_filename,
                log_to_console=True,
                error_file=error_filename
            )

        top = Top('top', conf=sim_conf, execution_strategy=parallel_toplevel_execution, controller_execution_strategy=sequential_execution)

        # Initialize the simulation to set up pybullet and get the time step
        top.init()

        # Create the external timer event source
        # The `on_full` policy is determined by the REAL_TIME_SIMULATION flag.
        # In real-time mode (`FAIL`), the simulation stops if it cannot keep up
        # with the timer, which is crucial for exposing performance bottlenecks.
        # In non-real-time mode (`DROP`), it drops events to keep running,
        # which can be useful for non-critical runs or debugging.
        on_full_behavior = OnFullBehavior.FAIL if sim_conf.REAL_TIME_SIMULATION else OnFullBehavior.DROP
        timer = Timer(identifier='physics_timer', interval_seconds=sim_conf.TIME_STEP, on_full=on_full_behavior)

        # Connect the timer to the simulation's main event queue
        top.connect_event_source(timer, 'time_event_in')

        Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_START, {LOG_DETAIL_KEY_MESSAGE: MSG_SIM_START.format(datetime.now())})
        
        # Disable the garbage collector to prevent unpredictable pauses during
        # the simulation, which can cause jitter and missed deadlines.
        gc.disable()

        # For real-time simulations, increase process priority to minimize
        # OS scheduler jitter, which can cause timer inaccuracies.
        if sim_conf.HIGH_PRIORITY:
            set_high_priority()
        
        top.start(stop_condition=lambda _: timer.stop_event_is_set())
        timer.start()

        top.wait() # Block until the simulation ends

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

        # The order is important for a graceful shutdown and correct user feedback.
        
        # 1. Stop the timer and simulation threads first.
        if 'timer' in locals() and timer:
            timer.stop()
            timer.wait()
        if top: # top.wait() is implicitly handled by the loop exit
            # Terminate hooks (e.g., disconnect pybullet)
            top.term()

        # 2. Terminate the plot server process if it's still running.
        if plot_server_process and plot_server_process.is_alive():
            plot_server_process.terminate()

        # 3. Stop the tracer. This is a blocking call that flushes final logs.
        if sim_conf.TRACER_ENABLED:
            Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_SUCCESS, {LOG_DETAIL_KEY_MESSAGE: MSG_SHUTDOWN_COMPLETE})
            Tracer.stop()

def export_diagram(structure_filename=None):
    """
    Instantiates the Multirotor model, serializes its structure to JSON,
    and saves it to a file.
    """
    print(MSG_DIAG_INSTANTIATING)
    top = Top('top')
    multirotor_part = top.get_part('multirotor')
    print(MSG_DIAG_PART_FOUND.format(multirotor_part.get_identifier()))

    serializer = DiagramSerializer()

    try:
        print(MSG_DIAG_SERIALIZING)
        json_output = serializer.export_part_to_json(multirotor_part)
        with open(structure_filename, 'w') as f:
            f.write(json_output)
        print(MSG_DIAG_EXPORT_SUCCESS.format(multirotor_part.get_identifier(), structure_filename))
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