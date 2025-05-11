from decimal import Decimal
from ml.engine import Part, Port
from ml.parts import Part_Timed, Operator, Control_Element, Broadcaster
from constants import X, Y, Z

class Trajectory_Planner(Part_Timed):

    def set_t_step(self, t_step):
        for control_element in self.control_elements:
            control_element.set_t_step(Decimal(t_step))

    def __init__(self, identifier, set_position, set_speed):
        ports = list()
        ports.append(Port('yaw', Port.IN))
        ports.append(Port('yaw_speed', Port.OUT))
        parts = {
            Broadcaster('broadcaster_time', 10)
        }
        self.control_elements = []
        for i in [X, Y, Z]:
            ports.append(Port(i.name(), Port.IN))
            ports.append(Port(f'{i.name()}_speed', Port.OUT))
            if i in [X, Y]:
                kp = 0.7
            else:
                kp = 3.0
            control_element = Control_Element(f'{i.name()}_control_element', Decimal(kp), Decimal(0.0), Decimal(0.0), Decimal(0.0), Decimal(0.0))
            parts.add(control_element)
            self.control_elements.append(control_element)
            parts.add(Operator(f'{i.name()}_set_position', 1, lambda in_0: Decimal(set_position(in_0)[i.index()])))
            parts.add(Operator(f'{i.name()}_set_speed', 1, lambda in_0: Decimal(set_speed(in_0)[i.index()])))
            parts.add(Operator(f'{i.name()}_sum', 2, lambda in_0, in_1: in_0 + in_1))
        parts.add(Operator('yaw_set_speed', 2, lambda in_0, in_1: Decimal(0.0)))
        super().__init__(identifier, Part.SEQUENTIAL_ALLOCATION, ports = ports, parts = parts)
        self.connect(self.get_port('time'), self.get_part('broadcaster_time').get_port('in'))
        broadcaster_time_index = 0
        for i in [X, Y, Z]:
            self.connect(self.get_part('broadcaster_time').get_port(f'out_{broadcaster_time_index}'), self.get_part(f'{i.name()}_control_element').get_port('time'))
            broadcaster_time_index += 1
            self.connect(self.get_part('broadcaster_time').get_port(f'out_{broadcaster_time_index}'), self.get_part(f'{i.name()}_set_position').get_port('in_0'))
            broadcaster_time_index += 1
            self.connect(self.get_part(f'{i.name()}_set_position').get_port('out'), self.get_part(f'{i.name()}_control_element').get_port('setpoint'))
            self.connect(self.get_port(i.name()), self.get_part(f'{i.name()}_control_element').get_port('measure'))
            self.connect(self.get_part(f'{i.name()}_control_element').get_port('actuation'), self.get_part(f'{i.name()}_sum').get_port('in_0'))
            self.connect(self.get_part('broadcaster_time').get_port(f'out_{broadcaster_time_index}'), self.get_part(f'{i.name()}_set_speed').get_port('in_0'))
            broadcaster_time_index += 1
            self.connect(self.get_part(f'{i.name()}_set_speed').get_port('out'), self.get_part(f'{i.name()}_sum').get_port('in_1'))
            self.connect(self.get_part(f'{i.name()}_sum').get_port('out'), self.get_port(f'{i.name()}_speed'))
        self.connect(self.get_part('broadcaster_time').get_port(f'out_{broadcaster_time_index}'), self.get_part('yaw_set_speed').get_port('in_0'))
        self.connect(self.get_port('yaw'), self.get_part('yaw_set_speed').get_port('in_1'))
        self.connect(self.get_part('yaw_set_speed').get_port('out'), self.get_port('yaw_speed'))