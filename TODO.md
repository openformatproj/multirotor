# Project TODO and Design Notes

This document outlines planned refactorings, design considerations, and future work for the multirotor simulation project.

## High-Priority Refactoring

### 1. Introduce Dedicated Data Classes for Physical Quantities

The current implementation uses primitive types (e.g., `list`, `float`) for data transfers between parts. This creates implicit contracts about the structure and meaning of data (e.g., is this list a position or a velocity?), making the system brittle and harder to understand. A more robust, object-oriented approach is to create dedicated data classes.

**Plan:**
1.  **Create `datatypes.py`**: A new file to house all physical quantity classes.
2.  **Define Data Classes**: Implement classes for all key physical quantities, such as:
    -   `Position`
    -   `Velocity`
    -   `Angle`
    -   `AngularRate`
    -   `Thrust`
    -   `Torque`
    These classes should encapsulate the data and provide helper methods (e.g., `as_pybullet_vector()`).
3.  **Refactor Parts**: Update all parts (`Sensors`, `Trajectory_Planner`, `Controller`, etc.) to use these new data types on their ports instead of primitive types.

---

## Design and Architectural Considerations

This section lists open questions and design areas that require further thought and discussion.

### 1. Decouple `Rigid_Body_Simulator` from PyBullet
- **Observation**: The `Rigid_Body_Simulator` part in `description.py` contains a lot of PyBullet-specific logic.
- **Goal**: Refactor it to be more generic. This could involve creating an abstract `PhysicsSimulator` base class and a concrete `PyBulletSimulator` implementation. This would make the framework more extensible to other physics engines in the future.

### 2. Review and Refactor Constant Definitions
- **Observation**: The `constants.py` file uses classes (`__X`, `__Y`, etc.) to define constants like `X`, `Y`, and `Z`.
- **Goal**: Evaluate if this is the best approach. Alternatives like `Enum`, `namedtuple`, or simple dictionaries could offer better type safety, readability, and ease of use. The chosen approach should be applied consistently.