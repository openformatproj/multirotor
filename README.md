- [Introduction](#introduction)
  - [Advantages](#advantages)
- [Quick Start](#quick-start)
- [Overview](#overview)
- [Tags](#tags)
- [Improvement Areas](#improvement-areas)
- [Contributing](#contributing)
  - [Ways to Contribute](#ways-to-contribute)
  - [Contribution Workflow](#contribution-workflow)
  - [Contributor Acknowledgment](#contributor-acknowledgment)
- [License](#license)

# Introduction

This project demonstrates how to design an autonomous multirotor model and simulate it using a component-based environment built with Python (hence, purely text-based).

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

This script builds the multirotor architecture, initializes it and starts the simulation timer. The execution should produce the following result:

![Figure 1: Multirotor simulation - 3D rendering](https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/1.gif)
<p align="center">Figure 1: Multirotor simulation - 3D rendering (PyBullet)</p>

![Figure 2: Multirotor simulation - Position plot](https://raw.githubusercontent.com/openformatproj/multirotor/refs/heads/master/img/2.gif)
<p align="center">Figure 2: Multirotor simulation - Position plot (matplotlib)</p>

By default, `run.py` configures the multirotor to follow a circular trajectory around the origin:

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
4. **Advanced Autonomy and AI Integration**: Explore advanced capabilities for autonomous navigation and decision-making. This includes:
    *   **Adaptive Trajectory Planning**: Implement trajectory planning based on local features and real-time sensor data (e.g., using LIDAR or cameras) for dynamic obstacle avoidance and navigation in complex environments.
        *   *Technologies*: Consider leveraging libraries like `OpenCV` for visual feature extraction, `PyLIDAR` (or similar) for LIDAR data processing, and path planning algorithms (e.g., A*, RRT*, D*) potentially integrated with robotics frameworks like `ROS` (Robot Operating System) with its Python bindings (`rospy`, `rclpy`).
    *   **Reinforcement Learning**: Develop and apply reinforcement learning techniques for learning optimal control policies, adaptive flight maneuvers, or complex mission execution.
        *   *Technologies*: Explore frameworks such as `TensorFlow (TF-Agents)`, `PyTorch (TorchRL)`, `Stable Baselines3`, or `Ray (RLlib)`.
    *   **Machine Vision & Perception**: Integrate advanced machine vision for tasks like object detection/classification (e.g., using YOLO, SSD, Faster R-CNN models), semantic segmentation, 3D reconstruction, visual odometry, and SLAM (Simultaneous Localization and Mapping).
        *   *Technologies*: Utilize `OpenCV`, `PyTorch`/`TensorFlow` for deep learning models, `Open3D` for 3D data processing, and libraries supporting various SLAM algorithms.
    *   **Advanced Sensor Fusion**: Implement sophisticated sensor fusion algorithms (e.g., Extended Kalman Filters (EKF), Unscented Kalman Filters (UKF), Particle Filters, Factor Graphs) to combine data from heterogeneous sensors (IMU, GNSS, cameras, LIDAR, radar) for robust and accurate state estimation, especially in challenging (e.g., GNSS-denied) environments.
        *   *Technologies*: Libraries like `FilterPy`, `NumPy`/`SciPy` for numerical backends, and potentially custom implementations or integrations with `ROS` message filters.
    *   **AI-Driven Mission Planning and Execution**: Develop higher-level cognitive functions for autonomous mission planning, dynamic replanning, task allocation, and complex decision-making under uncertainty.
        *   *Technologies*: Investigate AI planning techniques like PDDL (Planning Domain Definition Language) with Python parsers/solvers, Hierarchical Task Networks (HTNs) using libraries like `Pyhop`, or Behavior Trees (e.g., `py_trees`).
    *   **Swarm Intelligence and Multi-Agent Systems**: Extend the framework to simulate and coordinate multiple multirotors for collaborative tasks, exploring decentralized control strategies, formation flying, and distributed task allocation.
        *   *Technologies*: This could involve custom multi-agent simulation logic, or leveraging communication middleware like `ZeroMQ`, `MQTT`, or `ROS 2` for inter-agent communication.

# Contributing

We warmly welcome contributions to enhance this multirotor simulation project! Whether you're fixing a bug, adding a new feature, improving documentation, or suggesting new ideas, your input is valuable.

## Ways to Contribute

*   **Bug Reports**: If you find a bug, please open an issue detailing the problem, steps to reproduce, and your environment.
*   **Feature Requests**: Have an idea for a new feature or an improvement to an existing one? Open an issue to discuss it.
*   **Code Contributions**: Implement new features, fix bugs, or refactor code. See the [Improvement Areas](#improvement-areas) for inspiration.
*   **Documentation**: Improve code comments or add other explanatory documents.
*   **Testing**: Add or improve unit tests, integration tests, or simulation scenarios.

## Contribution Workflow

If you'd like to contribute code or documentation, please follow these general steps:

1.  **Discuss (for major changes)**: For significant changes (e.g., new core features, large refactors), please open an issue first to discuss your ideas. This helps ensure your efforts align with the project's direction and avoids duplicate work.
2.  **Fork the Repository**: Create your own fork of the project on GitHub.
3.  **Create a Branch**: Create a new branch in your fork for your feature or bug fix.
    ```bash
    git checkout -b your-descriptive-branch-name
    ```
4.  **Make Your Changes**: Implement your changes, adhering to any existing code style if possible.
5.  **Test Your Changes**: Ensure your changes work as expected and do not introduce new issues.
6.  **Commit Your Changes**: Write clear, concise commit messages.
7.  **Push to Your Fork**: Push your changes to your branch on your fork.
    ```bash
    git push origin your-descriptive-branch-name
    ```
8.  **Submit a Pull Request (PR)**: Open a pull request from your branch to the `master` (or main development) branch of the original repository. Provide a clear title and a detailed description of your changes, referencing any relevant issues.

## Contributor Acknowledgment

To ensure proper credit is given, here are some guidelines:

- **Acknowledgment**: Major contributors may be acknowledged in this section or other relevant parts of the documentation.
- **Commit History**: All contributions will be visible in the commit history of the repository.
- **Pull Request Description**: Please include your name or preferred alias in the pull request description if you'd like to be credited.
- **Issues and Discussions**: Active participation in issue discussions, providing feedback, and helping others are also valuable forms of contribution and may be acknowledged.

# License

![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0). You are free to use, modify, and distribute this software, provided that you include proper attribution to the original author(s). Redistribution must retain the original copyright notice and this license.
