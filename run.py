import os
import sys
import argparse
import multiprocessing

# Set the Qt platform plugin for Wayland if not already configured.
# This is a workaround for a common issue where PyQt5 fails to initialize
# on Wayland-based desktop environments. This must be done before any
# Qt-related library (like PyQt5 or matplotlib with a Qt backend) is imported.
if 'QT_QPA_PLATFORM' not in os.environ and 'wayland' in os.environ.get('XDG_SESSION_TYPE', '').lower():
    os.environ['QT_QPA_PLATFORM'] = 'wayland'

# Set up the Python path to ensure that both 'src' and its parent directory
# are accessible. This allows for consistent absolute imports across the project,
# especially for finding the sibling 'ml' library.
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
sys.path.insert(0, os.path.abspath(os.path.join(script_dir, '../..'))) # For `from ml...` and `from diagrams...`

if __name__ == "__main__":
    import multiprocessing

    # Set the multiprocessing start method to 'spawn'. This must be done
    # inside the `if __name__ == '__main__'` block and before any
    # multiprocessing-related objects are created. 'spawn' creates a clean
    # new process, which is crucial for compatibility with GUI libraries and
    # complex C-extensions like PyBullet.
    try:
        multiprocessing.set_start_method('spawn', force=True)
    except RuntimeError:
        # The start method can only be set once.
        pass

    parser = argparse.ArgumentParser(
        description="Run the multirotor simulation or related tools.",
        formatter_class=argparse.RawTextHelpFormatter
    )

    # Set the default command if none is provided
    parser.set_defaults(command='simulate')

    subparsers = parser.add_subparsers(
        dest='command',
        help='Available commands'
    )

    # --- Simulate Command ---
    parser_simulate = subparsers.add_parser(
        'simulate',
        help='Run the simulation and generate trace files (default command).'
    )
    parser_simulate.add_argument(
        '--trace-file', default='logs/trace.log',
        help="Path for the simulation trace log file (default: logs/trace.log)."
    )
    parser_simulate.add_argument(
        '--error-file', default='logs/errors.log',
        help="Path for the simulation error log file (default: logs/errors.log)."
    )

    # --- Analyze Command ---
    parser_analyze = subparsers.add_parser(
        'analyze',
        help='Analyze a simulation trace log.'
    )
    parser_analyze.add_argument(
        '--trace-file', default='logs/trace.log',
        help="Path of the simulation trace log file to analyze (default: logs/trace.log)."
    )
    parser_analyze.add_argument("--format", choices=['text', 'json', 'json:perfetto'], default='text', help="Output format (default: text).")
    parser_analyze.add_argument("-o", "--output", help="Path to the output file. If not provided, prints to stdout.")

    # --- Export Command ---
    parser_export = subparsers.add_parser(
        'export',
        help='Export the model structure to a JSON file.'
    )
    parser_export.add_argument(
        '--structure-file', default='logs/structure.json',
        help="Path to save the model structure JSON file (default: logs/structure.json)."
    )
    parser_export.add_argument(
        'part_id', nargs='?', default='multirotor',
        help="The ID of the part to export (e.g., 'multirotor', 'multirotor.controller'). Default: 'multirotor'."
    )

    # --- Import Command ---
    parser_import = subparsers.add_parser(
        'import',
        help='Import and display the model structure from a JSON file.'
    )
    parser_import.add_argument(
        '--structure-file', default='logs/structure.json',
        help="Path of the model structure JSON file to import (default: logs/structure.json)."
    )

    # --- Merge Command ---
    parser_merge = subparsers.add_parser(
        'merge',
        help='Merge two Perfetto JSON trace files into one.'
    )
    parser_merge.add_argument(
        'trace_file_1',
        help="Path to the first trace file."
    )
    parser_merge.add_argument(
        'trace_file_2',
        help="Path to the second trace file."
    )
    parser_merge.add_argument(
        'output_file',
        help="Path to the output merged trace file."
    )

    # Parse arguments. If no command is given, default to 'simulate'.
    args, unknown = parser.parse_known_args()
    if args.command is None:
        # No command was specified, so we run the default 'simulate' command.
        # We need to re-parse the arguments for the 'simulate' subparser
        # to correctly load its defaults (e.g., file paths).
        args = parser_simulate.parse_args(unknown)
        args.command = 'simulate'

    if args.command == 'simulate':
        # The simulate function is called directly. Lazy loading within runs.py
        # prevents child processes from re-executing heavy initializations.
        from runs import simulate
        simulate(trace_filename=args.trace_file, error_filename=args.error_file)
    elif args.command == 'analyze':
        from runs import analyze_trace
        analyze_trace(trace_file=args.trace_file, output_format=args.format, output_file=args.output)
    elif args.command == 'export':
        from runs import export_diagram
        export_diagram(structure_filename=args.structure_file, part_id=args.part_id)
    elif args.command == 'import':
        from runs import import_diagram
        import_diagram(structure_filename=args.structure_file)
    elif args.command == 'merge':
        from runs import merge_traces
        merge_traces(args.trace_file_1, args.trace_file_2, args.output_file)