import os
import sys

# Add the 'src' directory to the Python path to allow for absolute imports
# of modules within the project (e.g., 'plotting', 'description').
# This must be done before any project-specific imports.
sys.path.append(os.path.dirname(__file__))

import json
import time
import multiprocessing
from functools import partial
from PyQt5.QtWidgets import QApplication
from ml.strategies import sequential_execution
from ml.event_sources import Timer
from ml.tracer import Tracer
from ml.enums import OnFullBehavior, LogLevel
from diagrams.serializer import DiagramSerializer
from diagrams.engine import MainWindow
from diagrams.optimization import run_simulated_annealing
from datetime import datetime
from monitor.plot_server import main as run_plot_server
from description import Top
import conf

TRACE_FILENAME = "outs/multirotor_trace.log"
ERROR_FILENAME = "outs/multirotor_errors.log"
STRUCTURE_FILENAME = "outs/multirotor_structure.json"

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

def simulate(INITIAL_POSITION=None, INITIAL_ROTATION=None, SET_POSITION=None, SET_SPEED=None, POSITION_GRAPH_BOUNDARIES=None, SPEED_GRAPH_BOUNDARIES=None, PLOT=None, GUI=None):

    conf.INITIAL_POSITION = INITIAL_POSITION
    conf.INITIAL_ROTATION = INITIAL_ROTATION
    conf.SET_POSITION = SET_POSITION
    conf.SET_SPEED = SET_SPEED
    conf.POSITION_GRAPH_BOUNDARIES = POSITION_GRAPH_BOUNDARIES
    conf.SPEED_GRAPH_BOUNDARIES = SPEED_GRAPH_BOUNDARIES
    conf.PLOT = PLOT
    conf.GUI = GUI

    plot_server_process = None
    try:
        if conf.PLOT:
            print(MSG_PLOT_SERVER_STARTING)
            plot_server_process = multiprocessing.Process(
                target=run_plot_server,
                args=(conf.POSITION_GRAPH_BOUNDARIES, conf.SPEED_GRAPH_BOUNDARIES)
            )
            plot_server_process.start()
            # Wait a moment for the server to initialize and start listening
            time.sleep(1.5)

        # Start the tracer
        Tracer.start(
            level=LogLevel.TRACE,
            flush_interval_seconds=5.0,
            output_file=TRACE_FILENAME,
            log_to_console=True,
            error_file=ERROR_FILENAME
        )

        top = Top('top', execution_strategy=sequential_execution)

        # Initialize the simulation to set up pybullet and get the time step
        top.init()

        # Create the external timer event source
        # The `on_full` policy is determined by the REAL_TIME_SIMULATION flag.
        # In real-time mode (`FAIL`), the simulation stops if it cannot keep up
        # with the timer, which is crucial for exposing performance bottlenecks.
        # In non-real-time mode (`DROP`), it drops events to keep running,
        # which can be useful for non-critical runs or debugging.
        on_full_behavior = OnFullBehavior.FAIL if conf.REAL_TIME_SIMULATION else OnFullBehavior.DROP
        timer = Timer(identifier='physics_timer', interval_seconds=conf.TIME_STEP, on_full=on_full_behavior)

        # Connect the timer to the simulation's main event queue
        top.connect_event_source(timer, 'time_event_in')

        Tracer.log(LogLevel.INFO, MAIN_COMPONENT_ID, LOG_EVENT_START, {LOG_DETAIL_KEY_MESSAGE: MSG_SIM_START.format(datetime.now())})
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
        if 'timer' in locals() and timer:
            timer.stop()
            timer.wait()
        if 'top' in locals() and top: # top.wait() is implicitly handled by the loop exit
            top.term()
        if plot_server_process and plot_server_process.is_alive():
            print("Terminating plot server process...")
            plot_server_process.terminate()
        Tracer.stop()

def export_diagram():
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
        with open(STRUCTURE_FILENAME, 'w') as f:
            f.write(json_output)
        print(MSG_DIAG_EXPORT_SUCCESS.format(multirotor_part.get_identifier(), STRUCTURE_FILENAME))
    except TypeError as e:
        print(MSG_DIAG_EXPORT_ERROR.format(e))

def import_diagram():
    """
    Loads a diagram from the JSON file and displays it in the editor.
    """
    if not os.path.exists(STRUCTURE_FILENAME):
        print(MSG_DIAG_FILE_NOT_FOUND.format(STRUCTURE_FILENAME))
        return

    print(MSG_DIAG_LOADING.format(STRUCTURE_FILENAME))
    with open(STRUCTURE_FILENAME, 'r') as f:
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