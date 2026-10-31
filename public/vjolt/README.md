# VJolt Public Physics Interfaces

[Home](../../README.md)

`public/vjolt/` contains this project's own physics interfaces. It is not the Source/GMod SDK: the consumer headers depend on neither Source nor Jolt, and they reach the Jolt world of the running backend instead of linking another copy of Jolt.

| Header | Interface | Used by |
| --- | --- | --- |
| `gravity.h` | `VJoltGravityBridge001` | `jolt.gravity`: gravity fields and black holes |
| `constraints.h` | `VJoltConstraints001` | `jolt.constraints`: gears and racks |
| `path.h` | `VJoltPathBridge001` | `jolt.constraints`: rails |
| `softbody.h` | `VJoltSoftBody001` | `jolt.softbody` |
| `softbody_tools.h` | `VJoltSoftBodyTools001` | `jolt.softbody`'s optional `Trace`, `SetPlasticity` and `AddImpulseAt` |
| `stats.h` | `VJoltStats001` | C++ only: opt-in step timings and body, constraint and contact counters |
| `loader.h` | — | Finds the interfaces above in the loaded backend |

`cpu_policy.h` (CPU affinity for the Source simulation thread and the Jolt workers) and `source_abi.h` (Source SDK layout checks; needs the Source SDK) are used by the backend build, not by consumers.

## Use the headers

```cmake
include("${VJOLT_ROOT}/cmake/VJoltSDK.cmake")  # from a source checkout
# or, from an installed SDK: find_package(VJoltSDK CONFIG REQUIRED)
target_link_libraries(my_module PRIVATE VJolt::SDK)
```

`VJolt::SDK` is header-only and requires C++17. Build against the same headers as the backend you run with. To install the SDK, configure `gmsv_jolt` and install its `SDK` component: it writes `include/vjolt/`, `lib/cmake/VJoltSDK/` and the developer docs in `share/doc/VJoltSDK/`. For a contracts-only package, configure with `-DVJOLT_BUILD_LUA_MODULES=OFF`; `garrysmod_common` is then not needed.

## Discovery and lifetime

`loader.h` is Windows-only. `DiscoverVersionedInterface<T>(name, version)` checks the already-loaded backends in order (`vphysics_jolt_avx2`, `_sse42`, `_sse2`, unsuffixed `vphysics_jolt`, then `vphysics.dll`) and returns the first whose `CreateInterface` exposes `name` with a matching `GetVersion()`: a borrowed `api` pointer plus the module handle and name, or an error that `ErrorMessage()` describes. For an optional extension in the same DLL, call `QueryInterface<T>(result.module, name)` and check its version yourself, as `gmsv_joltsoftbody` does. The loader never loads a DLL or keeps the backend alive, so destroy your handles and finish all calls before the backend unloads.

Unless a header says otherwise:

- Call on the thread that owns the environment, outside physics steps and contact callbacks.
- Keep every handle you create, and destroy it when its entity is removed, the map is cleaned up or your code shuts down. Entities and `IPhysicsObject`s are borrowed.
- Soft-body vertex and triangle indices start at 0 in C++ and at 1 in Lua.

For usage, see the [physics API](../../docs/physics-api.md). To add an interface, follow the [feature export example](../../examples/physics/export/README.md).
