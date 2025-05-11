from ml.engine import Port
from ml.parts import Part_Timed

class Motor(Part_Timed):

    @staticmethod
    def torque(angular_speed):
        return angular_speed

    def behavior(self):
        if all([self.get_port(input).is_updated() for input in self.get_ports(Port.IN)]):
            t = self.get_port('time').get()
            angular_speed = self.get_port('angular_speed_in').get()
            reaction_torque = -Motor.torque(angular_speed)
            self.get_port('angular_speed_out').set(angular_speed)
            self.get_port('reaction_torque').set(reaction_torque.tolist())

    def __init__(self, identifier):
        ports = [
            Port('angular_speed_in', Port.IN),
            Port('angular_speed_out', Port.OUT),
            Port('reaction_torque', Port.OUT),
        ]
        super().__init__(identifier, Part_Timed.NO_ALLOCATION, ports = ports)