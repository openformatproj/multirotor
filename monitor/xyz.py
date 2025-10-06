import socket
import json
from ml.engine import Part, Port
from ml.tracer import Tracer
from ml.enums import LogLevel

class XYZ_Monitor(Part):
    """
    A behavioral part that acts as a client to a remote plotting server.

    It receives time, position, and speed data from the simulation,
    serializes it into JSON, and sends it over a TCP socket to a server
    responsible for visualization.
    """
    def __init__(self, identifier: str, plot_decimation: int, host: str = '127.0.0.1', port: int = 65432):
        """
        Initializes the XYZ_Monitor client.

        Args:
            identifier (str): The unique name for this part.
            plot_decimation (int): Send data only every Nth call to reduce network traffic.
            host (str): The hostname or IP address of the plotting server.
            port (int): The port number of the plotting server.
        """
        ports = [
            Port('time', Port.IN),
            Port('x', Port.IN), Port('y', Port.IN), Port('z', Port.IN),
            Port('x_speed', Port.IN), Port('y_speed', Port.IN), Port('z_speed', Port.IN)
        ]
        super().__init__(identifier, ports=ports, scheduling_condition=lambda p: p.get_port('time').is_updated())

        self.host = host
        self.port = port
        self.plot_decimation = plot_decimation
        self.tick_counter = 0
        self.socket = None

        self.add_hook('init', self._connect_to_server)
        self.add_hook('term', self._disconnect_from_server)

    def _connect_to_server(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            Tracer.log(LogLevel.INFO, self.get_identifier(), "CONNECT", {"host": self.host, "port": self.port})
        except ConnectionRefusedError:
            Tracer.log(LogLevel.ERROR, self.get_identifier(), "CONNECT_FAIL", {"message": "Connection refused. Is the plot_server running?"})
            self.socket = None # Ensure socket is None on failure

    def _disconnect_from_server(self):
        if self.socket:
            self.socket.close()
            Tracer.log(LogLevel.INFO, self.get_identifier(), "DISCONNECT", {})

    def behavior(self):
        if not self.socket:
            return # Do nothing if not connected

        self.tick_counter += 1
        if self.tick_counter % self.plot_decimation != 0:
            return

        data_point = {
            't': float(self.get_port('time').get()),
            'x': float(self.get_port('x').get()),
            'y': float(self.get_port('y').get()),
            'z': float(self.get_port('z').get()),
            'x_speed': float(self.get_port('x_speed').get()),
            'y_speed': float(self.get_port('y_speed').get()),
            'z_speed': float(self.get_port('z_speed').get()),
        }

        try:
            message = json.dumps(data_point) + '\n'
            self.socket.sendall(message.encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError):
            Tracer.log(LogLevel.WARNING, self.get_identifier(), "SEND_FAIL", {"message": "Connection to server lost."})
            self.socket = None # Stop trying to send