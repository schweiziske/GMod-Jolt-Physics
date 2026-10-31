# Extension Bridges

[Home](../../README.md)

Each file here implements a [public interface](../../public/vjolt/README.md) on top of the Jolt world that the current Source physics environment already owns.

| File | Interface | Responsibility |
| --- | --- | --- |
| `vjolt_gravity_bridge.cpp` | `VJoltGravityBridge001` | World and per-object gravity; directional, attraction and repulsion fields |
| `vjolt_constraints_bridge.cpp` | `VJoltConstraints001` | Gear and rack-and-pinion assemblies and their motors |
| `vjolt_path_bridge.cpp`, `vjolt_path_spline.h` | `VJoltPathBridge001` | Rail constraints and motors on a spline parameterized by arc length |
| `vjolt_softbody_bridge.cpp` | `VJoltSoftBody001`, `VJoltSoftBodyTools001` | Soft-body lifetime, state, forces and plasticity |
| `vjolt_softbody_mesh.cpp` | — | Mesh and settings validation; builds the Jolt shared settings |
| `vjolt_softbody_query.cpp` | — | Soft-body traces |
| `vjolt_stats_bridge.cpp` | `VJoltStats001` | Opt-in backend statistics; no Lua module uses them |

Each `*_internal.h` declares backend hooks for simulation steps, object release and transfer, and environment destruction. Add new implementation files to `volt_sources` in [CMake](../CMakeLists.txt).

- Gear, rack and rail constraints are removed when either of their bodies is destroyed or moved to another environment, and when their environment is destroyed.
- `Destroy` called during simulation or from a physics callback is deferred to a safe point in the step.
- Soft bodies have no Source `IPhysicsObject`; the addon's [`sent_jolt_softbody`](../../addons/jolt/lua/entities/sent_jolt_softbody/shared.lua) entity provides traces and player collision. The physics gun cannot grab them directly. Environment teardown releases its soft bodies.

Stock vehicles, players, NPC and door shadow controllers and the physics gun belong to the [base backend](../README.md), not to the bridges.
