# Export a Jolt Feature

[Home](../../../README.md)

This example exports Jolt's `GetNumBodies()` and `GetNumActiveBodies()`: for a given environment it reports the total number of bodies and the number of active rigid and soft bodies. It follows the same chain as the shipped `gmsv_jolt*` features:

`Lua → gmsv_joltfeature → IVJoltBodyCounts (backend) → the environment's JPH::PhysicsSystem`

| Layer | Example source | Destination and responsibility |
| --- | --- | --- |
| Public contract | [body_counts.h](public/vjolt/body_counts.h) | `public/vjolt/body_counts.h`: name, version, layout and threading rules |
| Backend implementation | [vjolt_body_counts.cpp](backend/vjolt_body_counts.cpp) | `vphysics_jolt/bridge/vjolt_body_counts.cpp`: calls Jolt and registers the interface |
| Lua module | [main.cpp](gmsv_joltfeature/source/main.cpp) | Discovers the interface, checks arguments, publishes Lua functions |
| Module build | [CMakeLists.txt](gmsv_joltfeature/CMakeLists.txt) | Builds `gmsv_joltfeature_win64.dll` (`_win32.dll` for 32-bit) |
| Lua caller | [body_counts.lua](lua/body_counts.lua) | Reads the counts on the server |

## 1. Define the public interface

The factory name `VJoltBodyCounts001` together with `GetVersion() == 1` forms the version contract. `Read(environment, out)` returns whether it succeeded and fills a struct of fixed-width integers; the backend checks the struct's `size` field against its own layout. Call it on the environment's owner thread, outside simulation steps and contact callbacks.

Give each new feature its own interface. Don't pass Jolt objects, STL containers or memory that the other DLL has to free across the boundary. The backend owns the interface singleton and modules only borrow it, which is why its destructor is protected. Changing a virtual method or a data layout requires a new version. New capabilities can instead go into a separate interface, as `path.h` and `softbody_tools.h` do, so existing modules keep working.

## 2. Implement and register it in the backend

Copy `public/vjolt/body_counts.h` and `backend/vjolt_body_counts.cpp` to `public/vjolt/body_counts.h` and `vphysics_jolt/bridge/vjolt_body_counts.cpp` in the repository.

`Read` gets the existing environment from `JoltPhysicsInterface::GetInstance().GetActiveEnvironmentByIndex(index)`, fails while that environment is simulating or inside a contact callback, and queries Jolt through `GetPhysicsSystem()`. `total` counts every body in the physics system, static ones included; active rigid and soft bodies are counted separately. None of these are Source entity counts.

The end of the file registers the interface with the backend's existing `CreateInterface` factory:

```cpp
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
    BodyCountsBridge, IVJoltBodyCounts, VJOLT_BODY_COUNTS_VERSION, g_BodyCounts);
```

Add `bridge/vjolt_body_counts.cpp` to `set(volt_sources ...)` in [`vphysics_jolt/CMakeLists.txt`](../../../vphysics_jolt/CMakeLists.txt); it then shares the backend's SDK, Jolt, include paths and forced includes.

Then rebuild the backend as described in the [build guide](../../../docs/build.md#physics-backend). The released backend doesn't contain this example interface, so the module can't discover it there.

## 3. Write the Lua binding

In `GMOD_MODULE_OPEN`, the module calls:

```cpp
auto found = VJoltSDK::DiscoverVersionedInterface<IVJoltBodyCounts>(
    VJOLT_BODY_COUNTS_VERSION, kVJoltBodyCountsVersion);
```

The loader looks for the interface in the backend that is already loaded and checks its version; `found.ErrorMessage()` explains a failure. The module then publishes the global table `joltfeature` with `PushCFunction` and `SetField`.

`GetBodyCounts([environment])` checks the environment index (an integer from 0 to 4096, default 0), calls the backend and copies the result into a Lua table. Failures return `nil, reason`; a non-numeric argument raises a Lua error. `Available()` returns `true`, or `false, reason`. `GMOD_MODULE_CLOSE` only drops the borrowed pointer, because the backend owns the singleton.

This example only reads data and owns nothing. A feature that creates fields, constraints or soft bodies also needs:

- handles that are never reused (with a generation or sequence number);
- an explicit `Destroy` plus a userdata `__gc`, and a `GMOD_MODULE_CLOSE` that destroys the handles the module still owns;
- cleanup when a body or environment is destroyed;
- deferred removal while the world is stepping.

[`gmsv_joltconstraints`](../../../gmsv_joltconstraints/source/main.cpp) and its [backend](../../../vphysics_jolt/bridge/vjolt_constraints_bridge.cpp) do all of this.

Keep Jolt access in the backend and Lua argument handling in the module. Document coordinates, units, input ranges, threading and lifetime in the public header. Never call Lua or Source from Jolt worker threads.

## 4. Build the module

The module needs only an external `garrysmod_common` checkout for the GMod Lua headers (see the [build guide](../../../docs/build.md#lua-modules-and-addon)) and this repository's `VJolt::SDK`. It needs neither the Source/GMod SDK nor the Jolt submodule. From the repository root:

```powershell
$GmodCommon = "C:/deps/garrysmod_common"
cmake -S examples/physics/export/gmsv_joltfeature -B .cache/joltfeature-win64 `
  -G "Visual Studio 17 2022" -A x64 "-DGMOD_COMMON=$GmodCommon"
cmake --build .cache/joltfeature-win64 --config Release
cmake --install .cache/joltfeature-win64 --config Release --prefix .cache/joltfeature-package/garrysmod
```

The module is staged as `.cache/joltfeature-package/garrysmod/lua/bin/gmsv_joltfeature_win64.dll`. For an experimental 32-bit build, use `-A Win32` and a separate build directory. If you move `gmsv_joltfeature/` out of this repository, pass `-DVJOLT_ROOT=<checkout>`, and keep the sibling `public/` folder unless the header is already in the checkout's `public/vjolt/`.

The module, the backend and the game must share the same architecture and ABI; see the [build guide](../../../docs/build.md).

## 5. Call it from Lua and optionally register it in the facade

With the module and the matching backend installed, run on the server:

```lua
require("joltfeature")
local available, reason = joltfeature.Available()
assert(available, reason)
local counts, detail = joltfeature.GetBodyCounts(0)
assert(counts, detail)
print(counts.total, counts.activeRigid, counts.activeSoft)
```

That is all a standalone module needs. To expose it as `jolt.feature` as well, add this entry to the `features` table in [`jolt.lua`](../../../addons/jolt/lua/includes/modules/jolt.lua), and add `"feature"` to its `order` list:

```lua
feature = {module = "joltfeature", global = "joltfeature", probe = "Available"},
```

`jolt.IsAvailable("feature")` then loads the module on demand, and `jolt.feature.GetBodyCounts()` calls it. The example does not register itself.

To build it with the bundled modules, follow the conventions in [`gmsv_jolt/CMakeLists.txt`](../../../gmsv_jolt/CMakeLists.txt): a `gmsv_jolt<name>/` directory at the repository root, a target named `gmsv_jolt<name>_win64` or `gmsv_jolt<name>_win32`, and `<name>` added to `vjolt_supported_lua_features`. A feature that becomes part of the project also needs install rules and English and Chinese docs.

A borrowed interface must not outlive the backend: finish all calls and unload the module before the backend unloads.
