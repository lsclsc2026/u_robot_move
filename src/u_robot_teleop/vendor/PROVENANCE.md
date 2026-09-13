# Native receiver provenance

Migration date: 2026-09-13.

- Device-delivered executable: `a2_network_bridge`.
- SHA256: `985f9d035418756e0f342919e5324a62a892dd5c1a1acb1408f86bafea7bd276`.
- Preserved from the operator’s existing A2 joystick deployment.
- Original C++ source was not available in this workspace. This package installs the existing binary; it does not reconstruct its protocol or claim a source rebuild.
- Native DDS libraries are installed privately from the pinned SDK2 checkout.
- Set `TELEOP_UPSTREAM_SOURCE` to a complete original CMake project providing the `a2_network_bridge` target to build from source. That mode requires validation when source becomes available.
- Distribution permission for this device-specific executable must be confirmed before making the repository public. This first repository is a private review snapshot.
