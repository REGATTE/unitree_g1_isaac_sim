# Changelog

## Unreleased

### Added

- Added Phase 0 configuration for explicit ROS 2 and native Unitree lowstate/lowcmd surfaces.
- Added single-authority lowcmd preflight validation so ROS 2 lowcmd and native lowcmd cannot both be enabled.
- Added shared lowcmd dataclasses in `src/dds/lowcmd_types.py`.
- Added Phase 1 native lowstate packet encoding, UDP publication, and native manager integration.
- Added native C++ bridge build scaffold under `native_sdk_bridge`.
- Added native C++ bridge UDP receive and Unitree SDK `LowState_` DDS publication path.

### Changed

- ROS 2 lowcmd is disabled by default; native lowcmd is the default command authority setting.
- ROS 2 lowstate and native lowstate are independently configurable and default to enabled.
- Native bridge launch now prefers the Unitree SDK2 bundled DDS libraries to avoid ROS 2 CycloneDDS linkage conflicts.

### Verified

- Focused Python tests pass for config, ROS 2 DDS manager, lowcmd handling, robot control, native lowstate protocol, and native manager behavior.
- Native C++ bridge builds against `~/unitree_sdk2`.
