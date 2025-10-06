import socket
import json
import os
import sys
import time
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Ensure correct Qt platform plugin for Wayland
if 'QT_QPA_PLATFORM' not in os.environ and 'wayland' in os.environ.get('XDG_SESSION_TYPE', '').lower():
    os.environ['QT_QPA_PLATFORM'] = 'wayland'

HOST = '127.0.0.1'
PORT = 65432

class RealTimePlotter:
    def __init__(self, position_bounds, speed_bounds):
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle('Multirotor State Monitor (Server)')

        self.data = {
            't': [], 'x': [], 'y': [], 'z': [],
            'x_speed': [], 'y_speed': [], 'z_speed': []
        }
        self.lines = {}

        # Setup XY Position Plot
        ax_xy_pos = self.axs[0, 0]
        ax_xy_pos.set_title('XY Position Trajectory')
        ax_xy_pos.set_xlabel('X Position (m)')
        ax_xy_pos.set_ylabel('Y Position (m)')
        ax_xy_pos.set_xlim(position_bounds[0])
        ax_xy_pos.set_ylim(position_bounds[1])
        ax_xy_pos.set_aspect('equal', adjustable='box')
        ax_xy_pos.grid(True)
        self.lines['xy_pos'], = ax_xy_pos.plot([], [], '-b')

        # Setup Z Position Plot
        ax_z_pos = self.axs[0, 1]
        ax_z_pos.set_title('Z Position (Altitude)')
        ax_z_pos.set_xlabel('Time (s)')
        ax_z_pos.set_ylabel('Z Position (m)')
        ax_z_pos.set_ylim(position_bounds[2])
        ax_z_pos.grid(True)
        self.lines['z_pos'], = ax_z_pos.plot([], [], '-r')

        # Setup XY Speed Plot
        ax_xy_spd = self.axs[1, 0]
        ax_xy_spd.set_title('XY Speed Vector')
        ax_xy_spd.set_xlabel('X Speed (m/s)')
        ax_xy_spd.set_ylabel('Y Speed (m/s)')
        ax_xy_spd.set_xlim(speed_bounds[0])
        ax_xy_spd.set_ylim(speed_bounds[1])
        ax_xy_spd.set_aspect('equal', adjustable='box')
        ax_xy_spd.grid(True)
        self.lines['xy_spd'], = ax_xy_spd.plot([], [], '-b')

        # Setup Z Speed Plot
        ax_z_spd = self.axs[1, 1]
        ax_z_spd.set_title('Z Speed (Vertical)')
        ax_z_spd.set_xlabel('Time (s)')
        ax_z_spd.set_ylabel('Z Speed (m/s)')
        ax_z_spd.set_ylim(speed_bounds[2])
        ax_z_spd.grid(True)
        self.lines['z_spd'], = ax_z_spd.plot([], [], '-r')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    def update_data(self, new_data):
        for key, value in new_data.items():
            if key in self.data:
                self.data[key].append(value)

    def update_plot(self):
        # Update plot data
        self.lines['xy_pos'].set_data(self.data['x'], self.data['y'])
        self.lines['z_pos'].set_data(self.data['t'], self.data['z'])
        self.lines['xy_spd'].set_data(self.data['x_speed'], self.data['y_speed'])
        self.lines['z_spd'].set_data(self.data['t'], self.data['z_speed'])

        # Autoscale axes that need it
        self.lines['z_pos'].axes.relim()
        self.lines['z_pos'].axes.autoscale_view(scalex=True, scaley=False)
        self.lines['z_spd'].axes.relim()
        self.lines['z_spd'].axes.autoscale_view(scalex=True, scaley=False)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(position_bounds=None, speed_bounds=None):
    print("Plotting Server: Starting...")
    # Use provided bounds or fall back to defaults
    if position_bounds is None:
        position_bounds = [(-2, 2), (-2, 2), (0, 3)]
    if speed_bounds is None:
        speed_bounds = [(-2, 2), (-2, 2), (-2, 2)]

    app = QApplication(sys.argv)
    plotter = RealTimePlotter(position_bounds, speed_bounds)
    plt.show(block=False)
    print(f"Plotting Server: Listening on {HOST}:{PORT}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Plotting Server: Client connected from {addr}")
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
                    print("Plotting Server: Client disconnected.")
                    break
                except json.JSONDecodeError:
                    print(f"Plotting Server: Invalid JSON received: {message}")
                    continue
                except Exception as e:
                    print(f"An error occurred: {e}")
                    break

    print("Plotting Server: Shutting down.")
    plt.close('all')

if __name__ == '__main__':
    main()