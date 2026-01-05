# Project TODO and Design Notes

This document outlines planned refactorings, design considerations, and future work for the multirotor simulation project.

## High-Priority Refactoring

NA

## Design and Architectural Considerations

### 1. Review and Refactor Constant Definitions
- **Observation**: The `constants.py` file uses classes (`__X`, `__Y`, etc.) to define constants like `X`, `Y`, and `Z`.
- **Goal**: Evaluate if this is the best approach. Alternatives like `Enum`, `namedtuple`, or simple dictionaries could offer better type safety, readability, and ease of use. The chosen approach should be applied consistently.