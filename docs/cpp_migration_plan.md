# C++ Migration Plan

This project should converge on C++ for runtime ROS 2 nodes, with Python kept for experiments, analysis, map generation, and quick diagnostics.

## Target Split

- `src/`: canonical ROS 2 workspace packages used by launch files.
- `cpp_src/`: migration source and reference implementations until each package is promoted into `src/`.
- Root-level `*.py`: research prototypes, algorithm notebooks-as-scripts, diagnostics, or one-off tools.
- `backup_*`: historical snapshots only; do not treat as active source.

## Migration Order

1. Keep `puppy_interfaces` as the shared contract.
2. Promote navigation kernel code first: planning, costmap, AMCL, recovery, and runtime state.
3. Promote adapter nodes next: motion, mode, status, safety, and hardware abstraction.
4. Promote behavior/control nodes: mission manager, safety manager, gait services, patrol orchestration.
5. Leave launch/config packages mixed only where Python launch files are standard ROS 2 practice.

## Package Direction

- Prefer `ament_cmake` for runtime packages.
- Use `rclcpp` lifecycle nodes where startup order matters.
- Keep all node names, topics, actions, services, and message fields compatible with existing Python nodes during migration.
- For each migrated node, keep one smoke test that launches or instantiates the C++ node and one contract test against `puppy_interfaces`.
- Do not migrate research scripts unless they are required at runtime.

## Promotion Checklist

For each Python runtime node:

1. Identify launch references and topic/service/action contracts.
2. Port behavior to a C++ class with explicit parameters.
3. Add the executable to `CMakeLists.txt`.
4. Update launch files to use the C++ executable.
5. Run `colcon build --symlink-install`.
6. Run unit/smoke tests in WSL/ROS.
7. Mark the Python node as legacy or remove it after parity is confirmed.

## Near-Term Candidates

- `src/puppy_core/puppy_core/mission_manager.py`
- `src/puppy_core/puppy_core/safety_manager.py`
- `src/puppy_core/puppy_core/goal_dispatcher.py`
- `src/puppypi_adapter/puppypi_adapter/motion_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/mode_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/status_adapter_node.py`

These already have conceptual C++ counterparts under `cpp_src/`, so the main work is package promotion, launch switching, and parity tests.
