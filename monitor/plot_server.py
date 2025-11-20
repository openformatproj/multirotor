import socket
import json
import os
import sys
from PyQt5.QtWidgets import QApplication
import signal
import matplotlib.pyplot as plt
from . import conf as monitor_conf

# --- Constants ---
# Environment
QT_QPA_PLATFORM_ENV = 'QT_QPA_PLATFORM'
WAYLAND_SESSION_TYPE = 'wayland'
XDG_SESSION_TYPE_ENV = 'XDG_SESSION_TYPE'

# Plotting
FIG_TITLE = 'Multirotor State Monitor (Server)'
# Data keys
KEY_TIME = 't'
KEY_X = 'x'
KEY_Y = 'y'
KEY_Z = 'z'
KEY_X_SPEED = 'x_speed'
KEY_Y_SPEED = 'y_speed'
KEY_Z_SPEED = 'z_speed'
# Line keys
LINE_KEY_XY_POS = 'xy_pos'
LINE_KEY_Z_POS = 'z_pos'
LINE_KEY_XY_SPD = 'xy_spd'
LINE_KEY_Z_SPD = 'z_spd'
# Plot titles
TITLE_XY_POS = 'XY Position Trajectory'
TITLE_Z_POS = 'Z Position (Altitude)'
TITLE_XY_SPD = 'XY Speed Vector'
TITLE_Z_SPD = 'Z Speed (Vertical)'
# Plot labels
LABEL_X_POS = 'X Position (m)'
LABEL_Y_POS = 'Y Position (m)'
LABEL_Z_POS = 'Z Position (m)'
LABEL_X_SPD = 'X Speed (m/s)'
LABEL_Y_SPD = 'Y Speed (m/s)'
LABEL_Z_SPD = 'Z Speed (m/s)'
LABEL_TIME = 'Time (s)'
# Plot styles
ASPECT_EQUAL = 'equal'
ADJUSTABLE_BOX = 'box'
LINE_STYLE_BLUE = '-b'
LINE_STYLE_RED = '-r'

# Server Messages
MSG_SERVER_STARTING = "Plotting Server: Starting..."
MSG_SERVER_LISTENING = "Plotting Server: Listening on {host}:{port}"
MSG_CLIENT_CONNECTED = "Plotting Server: Client connected from {addr}"
MSG_CLIENT_DISCONNECTED = "Plotting Server: Client disconnected."
MSG_INVALID_JSON = "Plotting Server: Invalid JSON received: {message}"
MSG_GENERIC_ERROR = "An error occurred: {error}"
MSG_SERVER_SHUTDOWN = "Plotting Server: Shutting down."

# Ensure correct Qt platform plugin for Wayland
if QT_QPA_PLATFORM_ENV not in os.environ and WAYLAND_SESSION_TYPE in os.environ.get(XDG_SESSION_TYPE_ENV, '').lower():
    os.environ[QT_QPA_PLATFORM_ENV] = WAYLAND_SESSION_TYPE

class RealTimePlotter:
    def __init__(self, position_bounds, speed_bounds):
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle(FIG_TITLE)

        self.data = {
            KEY_TIME: [], KEY_X: [], KEY_Y: [], KEY_Z: [],
            KEY_X_SPEED: [], KEY_Y_SPEED: [], KEY_Z_SPEED: []
        }
        self.lines = {}

        # Setup XY Position Plot
        ax_xy_pos = self.axs[0, 0]
        ax_xy_pos.set_title(TITLE_XY_POS)
        ax_xy_pos.set_xlabel(LABEL_X_POS)
        ax_xy_pos.set_ylabel(LABEL_Y_POS)
        ax_xy_pos.set_xlim(position_bounds[0])
        ax_xy_pos.set_ylim(position_bounds[1])
        ax_xy_pos.set_aspect(ASPECT_EQUAL, adjustable=ADJUSTABLE_BOX)
        ax_xy_pos.grid(True)
        self.lines[LINE_KEY_XY_POS], = ax_xy_pos.plot([], [], LINE_STYLE_BLUE)

        # Setup Z Position Plot
        ax_z_pos = self.axs[0, 1]
        ax_z_pos.set_title(TITLE_Z_POS)
        ax_z_pos.set_xlabel(LABEL_TIME)
        ax_z_pos.set_ylabel(LABEL_Z_POS)
        ax_z_pos.set_ylim(position_bounds[2])
        ax_z_pos.grid(True)
        self.lines[LINE_KEY_Z_POS], = ax_z_pos.plot([], [], LINE_STYLE_RED)

        # Setup XY Speed Plot
        ax_xy_spd = self.axs[1, 0]
        ax_xy_spd.set_title(TITLE_XY_SPD)
        ax_xy_spd.set_xlabel(LABEL_X_SPD)
        ax_xy_spd.set_ylabel(LABEL_Y_SPD)
        ax_xy_spd.set_xlim(speed_bounds[0])
        ax_xy_spd.set_ylim(speed_bounds[1])
        ax_xy_spd.set_aspect(ASPECT_EQUAL, adjustable=ADJUSTABLE_BOX)
        ax_xy_spd.grid(True)
        self.lines[LINE_KEY_XY_SPD], = ax_xy_spd.plot([], [], LINE_STYLE_BLUE)

        # Setup Z Speed Plot
        ax_z_spd = self.axs[1, 1]
        ax_z_spd.set_title(TITLE_Z_SPD)
        ax_z_spd.set_xlabel(LABEL_TIME)
        ax_z_spd.set_ylabel(LABEL_Z_SPD)
        ax_z_spd.set_ylim(speed_bounds[2])
        ax_z_spd.grid(True)
        self.lines[LINE_KEY_Z_SPD], = ax_z_spd.plot([], [], LINE_STYLE_RED)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    def update_data(self, new_data):
        for key, value in new_data.items():
            if key in self.data:
                self.data[key].append(value)

    def update_plot(self):
        # Update plot data
        self.lines[LINE_KEY_XY_POS].set_data(self.data[KEY_X], self.data[KEY_Y])
        self.lines[LINE_KEY_Z_POS].set_data(self.data[KEY_TIME], self.data[KEY_Z])
        self.lines[LINE_KEY_XY_SPD].set_data(self.data[KEY_X_SPEED], self.data[KEY_Y_SPEED])
        self.lines[LINE_KEY_Z_SPD].set_data(self.data[KEY_TIME], self.data[KEY_Z_SPEED])

        # Autoscale axes that need it
        self.lines[LINE_KEY_Z_POS].axes.relim()
        self.lines[LINE_KEY_Z_POS].axes.autoscale_view(scalex=True, scaley=False)
        self.lines[LINE_KEY_Z_SPD].axes.relim()
        self.lines[LINE_KEY_Z_SPD].axes.autoscale_view(scalex=True, scaley=False)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(position_bounds=None, speed_bounds=None):
    # Ignore SIGINT in the child process. The parent process will handle shutdown.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    print(MSG_SERVER_STARTING)
    # Use provided bounds or fall back to defaults
    if position_bounds is None:
        position_bounds = [(-2, 2), (-2, 2), (0, 3)]
    if speed_bounds is None:
        speed_bounds = [(-2, 2), (-2, 2), (-2, 2)]

    app = QApplication(sys.argv)
    plotter = RealTimePlotter(position_bounds, speed_bounds)
    plt.show(block=False)
    print(MSG_SERVER_LISTENING.format(host=monitor_conf.DEFAULT_HOST, port=monitor_conf.DEFAULT_PORT))

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((monitor_conf.DEFAULT_HOST, monitor_conf.DEFAULT_PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(MSG_CLIENT_CONNECTED.format(addr=addr))
            buffer = ""
            while plt.fignum_exists(plotter.fig.number):
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buffer += data.decode('utf-8')
                    while '\n' in buffer:
                        message, buffer = buffer.split('\n', 1)
                        if message:
                            point = json.loads(message)
                            plotter.update_data(point)
                    
                    plotter.update_plot()
                    app.processEvents() # Keep GUI responsive

                except (ConnectionResetError, BrokenPipeError):
                    print(MSG_CLIENT_DISCONNECTED)
                    break
                except json.JSONDecodeError:
                    print(MSG_INVALID_JSON.format(message=message))
                    continue
                except Exception as e:
                    print(MSG_GENERIC_ERROR.format(error=e))
                    break

    print(MSG_SERVER_SHUTDOWN)
    plt.close('all')

if __name__ == '__main__':
    main()