- [Introduction](#introduction)
  - [Advantages](#advantages)
- [Quick Start](#quick-start)
- [Overview](#overview)
- [Tags](#tags)
- [Improvement Areas](#improvement-areas)
- [Contributing](#contributing)
  - [Contributor Acknowledgment](#contributor-acknowledgment)
- [License](#license)

# Introduction

This project demonstrates how to design an autonomous multirotor model and simulate it using a component-based environment built with Python (hence, purely text-based, at least for now).

## Advantages

The advantages of the component-based approach are well known:

1. **Modularity**: Simplifies system design by breaking it into smaller, reusable components.
    a. **Flexibility**: Allows easy modification or replacement of individual components without affecting the entire system.
    b. **Functional Allocation**: Enables the allocation of configuration, functionalities, and behaviors to specific components.
    c. **Interfacing**: Highlights the separation between functionalities and interfaces.
2. **Early Validation**: Enables real-time simulation, testing, and validation of the system's behavior before physical implementation.
3. **Cost Efficiency**: Reduces development costs by identifying and addressing issues early in the design phase.
4. **Scalability**: Facilitates the addition of new features or components as the system evolves.
5. **Interdisciplinary Collaboration**: Promotes collaboration by providing a clear structure for integrating contributions from different domains.
6. **Code Generation**: Automates the creation of boilerplate and target-specific code from high-level behavioral descriptions, reducing manual effort and ensuring consistency across components.
7. **Simulation of Target-Specific Code**: Allows the import and simulation of code taken from real-world targets.
8. **Project Management**: Streamlines configuration, documentation generation, and overall project organization, making it easier to manage complex systems.

# Quick Start

A Docker image providing the source code included in this repository and all needed dependencies is available on Docker Hub. It can be launched, if, for instance, you use Docker Desktop, with the following commands:

```text
systemctl --user start docker-desktop
docker pull openformatproj/multirotor:latest
docker run -it openformatproj/multirotor:latest
```

Once launched, you can execute the `run.py` script from the container:

```text
/workspaces/multirotor# python run.py
```

This should produce the following result:

![Figure 1: Multirotor simulation - 3D rendering](https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/1.gif)
<p align="center">Figure 1: Multirotor simulation - 3D rendering</p>

![Figure 2: Multirotor simulation - Position plot](https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/2.gif)
<p align="center">Figure 2: Multirotor simulation - Position plot</p>

A simulation involving a flying multirotor is executed. By default, `run.py` configures the multirotor to follow a circular trajectory around the origin:

```python
SET_POSITION = lambda t : [cos(w*t), sin(w*t), 1 + a*sin(q*t)]
SET_SPEED = lambda t : [w * (-sin(w*t)), w * cos(w*t), a * q * cos(q*t)]
```

This behavior can be easily changed from the script itself.

# Overview

The idea in this case is to define the behavior of bottom-level components (motors, propellers, sensors, the trajectory planner, and the controller), build a multirotor model by attaching their ports together, connect such a model to a simulator, and run it.

<p id="figure-3"/>

![Figure 3: Multirotor model structure](https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/3.svg)
<p align="center">Figure 3: Multirotor model structure</p>

<a href="#figure-3">Figure 3</a> shows how the model has been built. Along with the components cited above, it is possible to see the simulator engine and a structure used to step the simulation, which is basically composed of a timer connected to all elements that must step.

Here's a snippet of how the environment is built at the code level (`description.py`):

```python
from ml.engine import Part
from ml.engine import Port
from ml.parts import Broadcaster, Part_Timed, Timer
import conf
from propeller.description import Propeller
from motor.description import Motor
from sensors.description import Sensors
from trajectory_planner.behavior import Trajectory_Planner
from controller.description import Controller

PROPELLERS_INDEXES = range(1, conf.PROPELLERS + 1)

class Multirotor(Part_Timed):
    def __init__(self, identifier):
        ports = [
            Port('position', Port.IN),
            Port('orientation', Port.IN),
            Port('linear_speed', Port.IN),
            Port('angular_speed', Port.IN)
        ]
        parts = {
            Sensors('sensors', conf.PLOT),
            Trajectory_Planner('trajectory_planner', conf.SET_POSITION, conf.SET_SPEED),
            Controller('controller', PROPELLERS_INDEXES)
        }
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'thrust_{i}', Port.OUT))
            ports.append(Port(f'torque_{i}', Port.OUT))
            parts.add(Motor(f'motor_{i}'))
            direction = Propeller.LEFT_HANDED if i in [2, 4] else Propeller.RIGHT_HANDED
            parts.add(Propeller(f'propeller_{i}', direction))
        super().__init__(identifier, Part.SEQUENTIAL_ALLOCATION, ports=ports, parts=parts)
        self.connect(self.get_port('position'), self.get_part('sensors').get_port('position'))
        self.connect(self.get_port('orientation'), self.get_part('sensors').get_port('orientation'))
        self.connect(self.get_port('linear_speed'), self.get_part('sensors').get_port('linear_speed'))
        self.connect(self.get_port('angular_speed'), self.get_part('sensors').get_port('angular_speed'))
        self.connect(self.get_part('sensors').get_port('roll'), self.get_part('controller').get_port('roll'))
        self.connect(self.get_part('sensors').get_port('pitch'), self.get_part('controller').get_port('pitch'))
        self.connect(self.get_part('sensors').get_port('yaw'), self.get_part('trajectory_planner').get_port('yaw'))
        self.connect(self.get_part('sensors').get_port('roll_speed'), self.get_part('controller').get_port('roll_speed'))
        self.connect(self.get_part('sensors').get_port('pitch_speed'), self.get_part('controller').get_port('pitch_speed'))
        self.connect(self.get_part('sensors').get_port('yaw_speed'), self.get_part('controller').get_port('yaw_speed'))
        for i in ['x', 'y', 'z']:
            self.connect(self.get_part('sensors').get_port(i), self.get_part('trajectory_planner').get_port(i))
            self.connect(self.get_part('trajectory_planner').get_port(f'{i}_speed'), self.get_part('controller').get_port(f'{i}_speed_setpoint'))
            self.connect(self.get_part('sensors').get_port(f'{i}_speed'), self.get_part('controller').get_port(f'{i}_speed'))
        self.connect(self.get_part('trajectory_planner').get_port('yaw_speed'), self.get_part('controller').get_port('yaw_speed_setpoint'))
        for i in PROPELLERS_INDEXES:
            self.connect(self.get_part('controller').get_port(f'angular_speed_{i}'), self.get_part(f'motor_{i}').get_port('angular_speed_in'))
            self.connect(self.get_part(f'motor_{i}').get_port('angular_speed_out'), self.get_part(f'propeller_{i}').get_port('angular_speed'))
            self.connect(self.get_part(f'propeller_{i}').get_port(f'thrust'), self.get_port(f'thrust_{i}'))
            self.connect(self.get_part(f'motor_{i}').get_port(f'reaction_torque'), self.get_port(f'torque_{i}'))
        if conf.PLOT:
            self.add_hook('init', self.init_plots)
            self.add_hook('term', self.term_plots)

class Rigid_Body_Simulator(Part_Timed):
    def behavior(self):
        if self.get_port('time').is_updated():
            self.t = self.get_port('time').get()
            position, orientation = self.engine.getBasePositionAndOrientation(self.multirotor_avatar)
            self.get_port('multirotor_position').set(position)
            self.get_port('multirotor_orientation').set(orientation)
            linear_speed, angular_speed = self.engine.getBaseVelocity(self.multirotor_avatar)
            self.get_port('multirotor_linear_speed').set(linear_speed)
            self.get_port('multirotor_angular_speed').set(angular_speed)
            if all((self.get_port(f'multirotor_thrust_{i}').is_updated() and self.get_port(f'multirotor_torque_{i}').is_updated()) for i in PROPELLERS_INDEXES):
                for i in PROPELLERS_INDEXES:
                    thrust = self.get_port(f'multirotor_thrust_{i}').get()
                    torque = self.get_port(f'multirotor_torque_{i}').get()
                    self.engine.applyExternalForce(self.multirotor_avatar, i-1, thrust, [0, 0, 0], self.engine.LINK_FRAME)
                    self.engine.applyExternalTorque(self.multirotor_avatar, i-1, torque, self.engine.LINK_FRAME)
                self.engine.stepSimulation()
    def __init__(self, identifier):
        ports = [
            Port('multirotor_position', Port.OUT),
            Port('multirotor_orientation', Port.OUT),
            Port('multirotor_linear_speed', Port.OUT),
            Port('multirotor_angular_speed', Port.OUT)
        ]
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'multirotor_thrust_{i}', Port.IN))
            ports.append(Port(f'multirotor_torque_{i}', Port.IN))
        super().__init__(identifier, Part.NO_ALLOCATION, ports = ports)

class Top(Part):
    def __init__(self, identifier):
        parts = [
            Rigid_Body_Simulator('simulator'),
            Multirotor('multirotor'),
            Timer('timer'),
            Broadcaster('broadcaster_time', outputs = 2)
        ]
        super().__init__(identifier, Part.SEQUENTIAL_ALLOCATION, parts = parts)
        self.connect(self.get_part('simulator').get_port('multirotor_position'), self.get_part('multirotor').get_port('position'))
        self.connect(self.get_part('simulator').get_port('multirotor_orientation'), self.get_part('multirotor').get_port('orientation'))
        self.connect(self.get_part('simulator').get_port('multirotor_linear_speed'), self.get_part('multirotor').get_port('linear_speed'))
        self.connect(self.get_part('simulator').get_port('multirotor_angular_speed'), self.get_part('multirotor').get_port('angular_speed'))
        self.connect(self.get_part('timer').get_port('time'), self.get_part('broadcaster_time').get_port('in'))
        self.connect(self.get_part('broadcaster_time').get_port('out_0'), self.get_part('simulator').get_port('time'))
        self.connect(self.get_part('broadcaster_time').get_port('out_1'), self.get_part('multirotor').get_port('time'))
        for i in PROPELLERS_INDEXES:
            self.connect(self.get_part('multirotor').get_port(f'thrust_{i}'), self.get_part('simulator').get_port(f'multirotor_thrust_{i}'))
            self.connect(self.get_part('multirotor').get_port(f'torque_{i}'), self.get_part('simulator').get_port(f'multirotor_torque_{i}'))
        self.add_hook('init', self.init_simulation_engine)
```

As it's possible to understand, `class Multirotor` defines its ports, the parts it's composed of (motors, propellers, sensors, the trajectory_planner and the controller) and their connections. All these parts are further defined in their specific subfolders, such as `./motor`, `./propeller`, and so on. This kind of description is called *structural* and allows building hierarchies. `class Rigid_Body_Simulator` provides instead an example of *behavioral* description:

* Whenever time port is updated, position, orientation and their derivatives are extracted from the simulation environment (specifically, from the multirotor avatar) and sent on output ports
* When, moreover, thrusts and torques are updated by the multirotor model, they are applied to the avatar and the simulation engine is stepped

`class Top` finally aggregates the simulator, the multirotor model and a timer whose role is updating the time with a specific periodicity. This allows to run a "real time" simulation, provided that all computations are able to terminate within that period. To perform the simulation one can simply instantiate that class and run the timer:

```python
top = Top('top')

top.init()
top.get_part('timer').run(t_end = None)
top.term()
```

This is exactly what the script `run.py` available in the Docker image does.

# Tags

`python`, `simulation`, `multirotor`, `drone`, `robotics`, `component-based`, `autonomous-systems`, `control-systems`, `MBSE`, `model-based-systems-engineering`, `systems-engineering`

# Improvement Areas

While the project demonstrates a functional simulation of a multirotor, there are several areas for potential improvement:

1. **Better Modeling of Sensors**: Enhance the accuracy and realism of sensor models (radio-altimeters, GNSS, INS, LIDARs, proximity sensors and so on).
2. **Better Modeling of Actuators (Motors and Propellers)**: Enhance the precision and reliability of actuator models.
3. **Creation of a More Detailed Simulation Environment**: Develop a richer and more complex simulation environment to test the multirotor in diverse scenarios.
4. **Development of Trajectory Planning Based on Local Features**: Implement trajectory planning that relies on local features rather than global world coordinates, enabling more adaptive and context-aware navigation.
5. **Experiment Advanced Features**: Explore capabilities like reinforcement learning for adaptive control policies, machine vision for enhanced environmental awareness, and advanced sensor fusion techniques for robust state estimation.

# Contributing

Contributions are welcome! If you'd like to contribute to this project, please follow these steps:

1. Create a new branch for your feature or bug fix within this repository.
2. Commit your changes and push them to your branch.
3. Submit a pull request with a detailed description of your changes.

For major changes, please open an issue first to discuss what you would like to change.

## Contributor Acknowledgment

To ensure proper credit is given, here are some guidelines:

- **Acknowledgment**: Major contributors may be acknowledged in this section or other relevant parts of the documentation.
- **Commit History**: All contributions will be visible in the commit history of the repository.
- **Pull Request Description**: Please include your name or preferred alias in the pull request description if you'd like to be credited.
- **Issues and Discussions**: Active participants in issues and discussions may also be acknowledged for their input and support.

# License

![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0). You are free to use, modify, and distribute this software, provided that you include proper attribution to the original author(s). Redistribution must retain the original copyright notice and this license.