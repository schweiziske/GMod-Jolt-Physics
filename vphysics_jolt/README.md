# VPhysics Jolt Backend

[Home](../README.md)

This directory implements Source's `IPhysics*` interfaces on top of Jolt and owns the physics state, solver calls, queries and object lifetimes. The extra features are implemented in [`bridge/`](bridge/README.md), their contracts live in [`public/vjolt/`](../public/vjolt/README.md), and their Lua bindings are the `gmsv_jolt*` modules.

## Code map

| Responsibility | Files |
| --- | --- |
| DLL initialization, global Jolt resources | [vjolt_interface.cpp](vjolt_interface.cpp) |
| CPU variant loading, thread affinity | [vphysics_wrapper.cpp](../vphysics_wrapper/vphysics_wrapper.cpp) selects the backend DLL; [cpu_policy.h](../public/vjolt/cpu_policy.h) supplies thread and CPU assignment policies |
| World, simulation order, Source callback stages | [vjolt_environment.cpp](vjolt_environment.cpp): creating objects, controllers and constraints; `Simulate`; queries; object transfer; save/restore; springs |
| Synchronous callbacks during solving, deferred events | [vjolt_owner_callback_jobs.h](vjolt_owner_callback_jobs.h) routes synchronous game callbacks to the simulation caller's thread; [vjolt_environment_events.h](vjolt_environment_events.h) manages deferred events and listener generations |
| Object deletion, world teardown | [vjolt_environment_lifecycle.cpp](vjolt_environment_lifecycle.cpp): delete queues, reentrancy guards, collision-data release, remaining objects |
| Single Source physics object | [vjolt_object.cpp](vjolt_object.cpp): wraps a Jolt body; converts mass, inertia, velocity and impulses; controllers and destruction listeners |
| Collision events, contact compatibility | [vjolt_listener_contact.cpp](vjolt_listener_contact.cpp), [vjolt_friction.cpp](vjolt_friction.cpp): filtering, callbacks, contact and friction queries; caches in [vjolt_should_collide_cache.h](vjolt_should_collide_cache.h) and [vjolt_friction_contact_cache.h](vjolt_friction_contact_cache.h) |
| Player and shadow controllers | [vjolt_controller_player.cpp](vjolt_controller_player.cpp), [vjolt_controller_shadow.cpp](vjolt_controller_shadow.cpp), [vjolt_shadow_control.h](vjolt_shadow_control.h) |
| Motion, drag, buoyancy, vehicle controllers | `vjolt_controller_motion.cpp`, `vjolt_controller_drag.cpp`, `vjolt_controller_fluid.cpp`, `vjolt_controller_vehicle.cpp` |
| Constraints and constraint groups | [vjolt_constraints.cpp](vjolt_constraints.cpp); [vjolt_havana_constraints.cpp](vjolt_havana_constraints.cpp) solves fixed, length and ragdoll constraints with Source/Havana's own equations |
| Collision geometry and traces | [vjolt_collide.cpp](vjolt_collide.cpp) and [vjolt_querymodel.cpp](vjolt_querymodel.cpp) manage geometry and query models; trace entry points in [vjolt_collide_trace.cpp](vjolt_collide_trace.cpp) call the Source compatibility implementation in [vjolt_source_trace.inl](vjolt_source_trace.inl) and [vjolt_source_trace_solver.inl](vjolt_source_trace_solver.inl) |
| Materials and physics KeyValues | [vjolt_surfaceprops.cpp](vjolt_surfaceprops.cpp), [vjolt_parse.cpp](vjolt_parse.cpp), [vjolt_keyvalues_parser.h](vjolt_keyvalues_parser.h), [vjolt_keyvalues_schema.cpp](vjolt_keyvalues_schema.cpp) |
| Gravity fields, gears and racks, rails, soft bodies, statistics | [bridge/](bridge/README.md) |
| Build and SDK ABI compatibility | [CMakeLists.txt](CMakeLists.txt), [GModSDK.cmake](../cmake/GModSDK.cmake), [gmod_convar.cmake](../cmake/gmod_convar.cmake), [sdk_prefix.h](compat/sdk_prefix.h), [branch_overrides.h](compat/branch_overrides.h) |

## Deletion and callback rules

- `DestroyObject` marks and queues objects, deferring release during simulation, cleanup or when the delete queue is enabled. `CleanupDeleteList` does not release objects during simulation or nested cleanup.
- Destruction callbacks can append to queues or delete other listeners. Remove the current entry before calling it, then re-read the container; never retain references or iterators across callbacks.
- Release constraints and objects before shared `CPhysCollide` data.
- Environment teardown uses its own BodyID snapshot; queries in game callbacks can overwrite `m_CachedBodies`. Remove extension objects and detach the activation listener before cleaning up Source objects.
- Synchronous callbacks during solving run on the simulation caller's thread while Jolt still holds body locks. Use the environment's callback-aware query entry points, `NoLock` paths and physics mutation deferral mechanisms.

## Making changes

- When you add or move a backend `.cpp`, update `volt_sources` in the [CMake source list](CMakeLists.txt).
- Preserve Source/IVP compatibility behavior, such as `ClampVelocityChanges` scaling pending angular velocity by the pending *linear* speed.
- Build instructions are in the [build guide](../docs/build.md#physics-backend). Test solver and contact changes in game, with scenes that exercise them.
