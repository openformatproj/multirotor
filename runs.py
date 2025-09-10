import os
import sys
import json
from functools import partial
from PyQt5.QtWidgets import QApplication
from ml.event_sources import Timer
from ml.tracer import Tracer
from ml.enums import OnFullBehavior, LogLevel
from diagrams.serializer import DiagramSerializer
from diagrams.engine import MainWindow
from diagrams.optimization import run_simulated_annealing
sys.path.append(os.path.dirname(__file__))
from datetime import datetime
from description import Top
import conf

TRACE_FILENAME = "outs/multirotor_trace.log"
ERROR_FILENAME = "outs/multirotor_errors.log"
STRUCTURE_FILENAME = "outs/multirotor_structure.json"

def simulate(INITIAL_POSITION=None, INITIAL_ROTATION=None, SET_POSITION=None, SET_SPEED=None, POSITION_GRAPH_BOUNDARIES=None, SPEED_GRAPH_BOUNDARIES=None, PLOT=None, GUI=None):

    conf.INITIAL_POSITION = INITIAL_POSITION
    conf.INITIAL_ROTATION = INITIAL_ROTATION
    conf.SET_POSITION = SET_POSITION
    conf.SET_SPEED = SET_SPEED
    conf.POSITION_GRAPH_BOUNDARIES = POSITION_GRAPH_BOUNDARIES
    conf.SPEED_GRAPH_BOUNDARIES = SPEED_GRAPH_BOUNDARIES
    conf.PLOT = PLOT
    conf.GUI = GUI

    # Start the tracer
    Tracer.start(
        level=LogLevel.TRACE,
        flush_interval_seconds=5.0,
        output_file=TRACE_FILENAME,
        log_to_console=True,
        error_file=ERROR_FILENAME
    )

    try:
        top = Top('top')

        # Initialize the simulation to set up pybullet and get the time step
        top.init()

        # Create the external timer event source
        # The timer interval is now driven by the central configuration.
        # The `on_full` policy is set to `OnFullBehavior.FAIL`. This ensures
        # that every physics step is processed, preventing instability. The
        # system will stop with a queue full error if it cannot keep up, which
        # is the correct behavior for a real-time simulation that misses its
        # deadline.
        timer = Timer(identifier='physics_timer', interval_seconds=conf.TIME_STEP, on_full=OnFullBehavior.OVERWRITE)

        # Connect the timer to the simulation's main event queue
        top.connect_event_source(timer, 'time_event_in')

        # Start the simulation and timer threads.
        Tracer.log(LogLevel.INFO, "MAIN", "START", {"message": f"Starting simulation threads on {datetime.now()}"})
        top.start(stop_condition=lambda _: timer.stop_event_is_set())
        timer.start()

        try:
            # Wait for the main simulation thread to finish, for any reason
            # (e.g., completion, error, or user interrupt).
            top.join()
        except KeyboardInterrupt:
            Tracer.log(LogLevel.INFO, "MAIN", "INTERRUPT", {"message": "Ctrl+C received, stopping simulation."})
        finally:
            # Ensure all threads are stopped gracefully, regardless of how the loop ended.
            timer.stop()
            timer.join() # Wait for the timer thread to exit.

        if top.get_exception() or timer.get_exception():
            Tracer.log(LogLevel.ERROR, "MAIN", "FAILURE", {"message": "Simulation finished with errors."})
        else:
            Tracer.log(LogLevel.INFO, "MAIN", "SUCCESS", {"message": "Simulation finished."})

    finally:
        # Terminate hooks (e.g., disconnect pybullet) and stop the tracer
        if 'top' in locals() and top:
            top.term()
        Tracer.stop()

def export_diagram():
    """
    Instantiates the Multirotor model, serializes its structure to JSON,
    and saves it to a file.
    """
    print("Instantiating the simulation model...")
    top = Top('top')
    multirotor_part = top.get_part('multirotor')
    print(f"Found part to serialize: '{multirotor_part.get_identifier()}'")

    serializer = DiagramSerializer()

    try:
        print("Serializing part structure to JSON...")
        json_output = serializer.export_part_to_json(multirotor_part)
        with open(STRUCTURE_FILENAME, 'w') as f:
            f.write(json_output)
        print(f"\nSuccessfully exported the structure of '{multirotor_part.get_identifier()}' to '{STRUCTURE_FILENAME}'")
    except TypeError as e:
        print(f"\nError: {e}")

def import_diagram():
    """
    Loads a diagram from the JSON file and displays it in the editor.
    """
    if not os.path.exists(STRUCTURE_FILENAME):
        print(f"Error: JSON file '{STRUCTURE_FILENAME}' not found. Please run with 'export' argument first.")
        return

    print(f"Loading diagram from '{STRUCTURE_FILENAME}'...")
    with open(STRUCTURE_FILENAME, 'r') as f:
        json_data = f.read()

    # A QApplication instance is always required for a Qt app.
    app = QApplication(sys.argv)

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
        print("Diagram imported successfully.")
    except (ValueError, json.JSONDecodeError) as e:
        print(f"Error importing diagram: {e}")
        return

    # Start the application event loop to show the window.
    sys.exit(main_window.start())