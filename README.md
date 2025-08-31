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

<div style="display: flex; justify-content: center; align-items: flex-start; flex-wrap: wrap; gap: 20px;">
  <div> <!-- Figure 1 container -->
    <img src="https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/1.gif" alt="Figure 1: Multirotor simulation - Trajectory view" height="300" style="display: block; margin-left: auto; margin-right: auto;" />
    <p style="text-align: center;">Figure 1: Multirotor simulation - Trajectory view</p>
  </div>
  <div> <!-- Figure 2 container -->
    <img src="https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/2.gif" alt="Figure 2: Multirotor simulation - Altitude view" height="300" style="display: block; margin-left: auto; margin-right: auto;" />
    <p style="text-align: center;">Figure 2: Multirotor simulation - Altitude view</p>
  </div>
</div>

A simulation involving a flying multirotor is executed. By default, `run.py` configures the multirotor to follow a circular trajectory around the origin:

```python
SET_POSITION = lambda t : [cos(w*t), sin(w*t), 1 + a*sin(q*t)]
SET_SPEED = lambda t : [w * (-sin(w*t)), w * cos(w*t), a * q * cos(q*t)]
```

This behavior can be easily changed from the script itself.

# Overview

The idea in this case is to define the behavior of bottom-level components (motors, propellers, sensors, the trajectory planner, and the controller), build a multirotor model by attaching their ports together, connect such a model to a simulator, and run it.

<div style="text-align: center;">
<div style="display: flex; justify-content: center;">
  <div id="figure-3">
    <img src="https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/3.png" alt="Figure 3: Multirotor model structure" style="display: block; margin-left: auto; margin-right: auto;" />
    <p style="text-align: center;">Figure 3: Multirotor model structure</p>
  </div>
</div>

<a href="#figure-3">Figure 3</a> shows the high-level structure of the `Multirotor` model (the diagram has been generated with [openformatproj/diagrams](https://github.com/openformatproj/diagrams)). The `Top` component encapsulates the `Multirotor` model and the `Rigid_Body_Simulator`. The entire simulation is driven by an external `Timer` (an `EventSource`), which provides time-tick events to the `Top` part. This event-driven approach ensures that the simulation only executes when a time step is required.

Here's a snippet of how the environment is built at the code level (`description.py`):

```python
from ml.engine import Part, Port, EventQueue, sequential_execution
from ml.parts import EventToDataSynchronizer
from ml.event_sources import Timer
import conf
from propeller.description import Propeller
from motor.description import Motor
from sensors.description import Sensors
from trajectory_planner.behavior import Trajectory_Planner
from controller.description import Controller
from constants import X, Y, Z

PROPELLERS_INDEXES = range(1, conf.PROPELLERS + 1)

class Multirotor(Part):
    def __init__(self, identifier: str):
        ports = [
            Port('time', Port.IN),
            Port('position', Port.IN),
            Port('orientation', Port.IN),
            Port('linear_speed', Port.IN),
            Port('angular_speed', Port.IN)
        ]
        parts = {
            'sensors': Sensors('sensors', conf.PLOT),
            'trajectory_planner': Trajectory_Planner('trajectory_planner', conf.SET_POSITION, conf.SET_SPEED),
            'controller': Controller('controller', PROPELLERS_INDEXES)
        }
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'thrust_{i}', Port.OUT))
            ports.append(Port(f'torque_{i}', Port.OUT))
            parts[f'motor_{i}'] = Motor(f'motor_{i}')
            direction = Propeller.LEFT_HANDED if i in [2, 4] else Propeller.RIGHT_HANDED
            parts[f'propeller_{i}'] = Propeller(f'propeller_{i}', direction)
            
        super().__init__(
            identifier=identifier,
            execution_strategy=sequential_execution,
            ports=ports,
            parts=parts
        )
        
        # Connect time, position, orientation, etc. to inner parts
        # ... (connection logic as in description.py) ...

class Rigid_Body_Simulator(Part):
    def behavior(self):
        time_port = self.get_port('time')
        if time_port.is_updated():
            # Consume the time input to allow the simulation to advance
            time_port.get()

            position, orientation = self.engine.getBasePositionAndOrientation(self.multirotor_avatar)
            self.get_port('multirotor_position').set(position)
            # ... (set other sensor outputs) ...

            # Check if all motor inputs are ready before applying forces
            thrust_ports = [self.get_port(f'multirotor_thrust_{i}') for i in PROPELLERS_INDEXES]
            torque_ports = [self.get_port(f'multirotor_torque_{i}') for i in PROPELLERS_INDEXES]
            if all(p.is_updated() for p in thrust_ports) and all(p.is_updated() for p in torque_ports):
                for i in PROPELLERS_INDEXES:
                    thrust = thrust_ports[i-1].get()
                    torque = torque_ports[i-1].get()
                    self.engine.applyExternalForce(self.multirotor_avatar, i-1, thrust, [0, 0, 0], self.engine.LINK_FRAME)
                    self.engine.applyExternalTorque(self.multirotor_avatar, i-1, torque, self.engine.LINK_FRAME)
                self.engine.stepSimulation()

    def __init__(self, identifier: str):
        ports = [
            Port('time', Port.IN),
            Port('multirotor_position', Port.OUT),
            # ... (other ports) ...
        ]
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'multirotor_thrust_{i}', Port.IN))
            ports.append(Port(f'multirotor_torque_{i}', Port.IN))
        super().__init__(identifier=identifier, ports=ports)

class Top(Part):
    def __init__(self, identifier: str):
        event_queues = [EventQueue('time_event_in', EventQueue.IN, size=1)]
        parts = {
            'time_dist': EventToDataSynchronizer('time_dist', 'time_event_in', 'time_out'),
            'simulator': Rigid_Body_Simulator('simulator'),
            'multirotor': Multirotor('multirotor')
        }
        super().__init__(identifier=identifier, execution_strategy=sequential_execution, parts=parts, event_queues=event_queues)
        
        # Connect the top-level event queue to the time distributor
        self.connect_event_queue(self.get_event_queue('time_event_in'), self.get_part('time_dist').get_event_queue('time_event_in'))
        
        # Connect the time distributor's data output to the simulator and multirotor
        time_out_port = self.get_part('time_dist').get_port('time_out')
        self.connect(time_out_port, self.get_part('simulator').get_port('time'))
        self.connect(time_out_port, self.get_part('multirotor').get_port('time'))
        
        # Connect the simulator and multirotor together
        simulator = self.get_part('simulator')
        multirotor = self.get_part('multirotor')
        self.connect(simulator.get_port('multirotor_position'), multirotor.get_port('position'))
        self.connect(simulator.get_port('multirotor_orientation'), multirotor.get_port('orientation'))
        # ... and other connections from simulator to multirotor ...

        self.add_hook('init', self._init_pybullet)
        self.add_hook('term', self._term_pybullet)
```

As it's possible to understand, `class Multirotor` defines its ports, the parts it's composed of (motors, propellers, sensors, the trajectory_planner and the controller) and their connections. All these parts are further defined in their specific subfolders, such as `./motor`, `./propeller`, and so on. This kind of description is called *structural* and allows building hierarchies. `class Rigid_Body_Simulator` provides instead an example of *behavioral* description:

* Whenever time port is updated, position, orientation and their derivatives are extracted from the simulation environment (specifically, from the multirotor avatar) and sent on output ports
* When, moreover, thrusts and torques are updated by the multirotor model, they are applied to the avatar and the simulation engine is stepped

`class Top` finally aggregates the simulator, the multirotor model and a timer whose role is updating the time with a specific periodicity. This allows to run a "real time" simulation, provided that all computations are able to terminate within that period. To perform the simulation one can simply instantiate that class and run the timer:
`class Top` finally aggregates the simulator and the multirotor model. It is driven by an external `Timer` (an `EventSource`) that provides time events. This allows running a "real time" simulation, provided that all computations are able to terminate within the timer's period. To perform the simulation, one can instantiate these components and run them in separate threads:

```python
top = Top('top')
# Initialize the simulation to set up pybullet and get the physics time step
top.init()
t_step = top.t_step

# Create the external timer event source using the time step from the physics engine
timer = Timer(identifier='physics_timer', interval_seconds=t_step)

# Connect the timer to the simulation's main event queue
top.connect_event_source(timer, 'time_event_in')

# Start the simulation and timer threads
top.start(stop_condition=lambda p: timer.stop_event.is_set())
timer.start()

# Wait for threads to complete...
top.join()
timer.join()

# Terminate hooks (e.g., disconnect pybullet)
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