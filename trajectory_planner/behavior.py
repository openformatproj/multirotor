from decimal import Decimal
from ml.engine import Part, Port
from ml.parts import Part_Timed
from constants import X, Y, Z

class Trajectory_Planner(Part_Timed):

    def set_t_step(self, t_step):
        None

    def behavior(self):
        if all([self.get_port(input).is_updated() for input in self.get_ports(Port.IN)]):
            t = self.get_port('time').get()
            x = self.get_port(X.name()).get()
            y = self.get_port(Y.name()).get()
            z = self.get_port(Z.name()).get()
            yaw = self.get_port('yaw').get()
            self.get_port(f'{X.name()}_speed').set(Decimal(0.7)*(Decimal(self.set_position(t)[X.index()]) - x) + Decimal(self.set_speed(t)[X.index()]))
            self.get_port(f'{Y.name()}_speed').set(Decimal(0.7)*(Decimal(self.set_position(t)[Y.index()]) - y) + Decimal(self.set_speed(t)[Y.index()]))
            self.get_port(f'{Z.name()}_speed').set(Decimal(3.0)*(Decimal(self.set_position(t)[Z.index()]) - z) + Decimal(self.set_speed(t)[Z.index()]))
            self.get_port('yaw_speed').set(Decimal(0.0))

    def __init__(self, identifier, set_position, set_speed):
        ports = list()
        for i in [X, Y, Z]:
            ports.append(Port(i.name(), Port.IN))
            ports.append(Port(f'{i.name()}_speed', Port.OUT))
        ports.append(Port('yaw', Port.IN))
        ports.append(Port('yaw_speed', Port.OUT))
        super().__init__(identifier, Part.NO_ALLOCATION, ports = ports)
        self.set_position = set_position
        self.set_speed = set_speed