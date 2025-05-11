from ml.engine import Port
from ml.parts import Part_Timed

class Propeller(Part_Timed):

    RIGHT_HANDED = 0 # When rotates clockwise, the thrust is downwards
    LEFT_HANDED = 1 # When rotates clockwise, the thrust is upwards

    @staticmethod
    def thrust(angular_speed, direction):
        if direction == Propeller.RIGHT_HANDED:
            return angular_speed
        else:
            return -angular_speed

    def behavior(self):
        if all([self.get_port(input).is_updated() for input in self.get_ports(Port.IN)]):
            t = self.get_port('time').get()
            thrust = Propeller.thrust(self.get_port('angular_speed').get(), self.direction)
            self.get_port('thrust').set(thrust.tolist())

    def __init__(self, identifier, direction):
        ports = [
            Port('angular_speed', Port.IN),
            Port('thrust', Port.OUT),
        ]
        super().__init__(identifier, Part_Timed.NO_ALLOCATION, ports = ports)
        self.direction = direction