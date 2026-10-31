# Jolt Lua Modules

[Home](../README.md)

`require("jolt")` is the server-side Lua entry point to the three native modules:

| Namespace | Native module | Provides | API reference |
| --- | --- | --- | --- |
| `jolt.gravity` | `gmsv_joltgravity` | World and per-object gravity, gravity fields (black holes are built on these) | [Gravity](../gmsv_joltgravity/README.md) |
| `jolt.constraints` | `gmsv_joltconstraints` | Gears, racks and rails | [Gears and racks](../gmsv_joltconstraints/README.md), [rails](../gmsv_joltconstraints/PATH.md) |
| `jolt.softbody` | `gmsv_joltsoftbody` | Cloth, pressure meshes, vertex pinning | [Soft bodies](../gmsv_joltsoftbody/README.md) |

Requiring `jolt` doesn't load any native module. Each feature loads the first time you ask for it:

| Call | Result |
| --- | --- |
| `jolt.ListFeatures()` | `{"gravity", "constraints", "softbody"}`; loads nothing |
| `jolt.IsAvailable(name)` | Loads the module and checks the backend; returns `true`, or `false, reason` |
| `jolt.Load(name)` | Returns the feature table, or `nil, reason` |
| `jolt.gravity` (or another namespace) | Loads the feature on first access; raises a Lua error if it's unavailable |

Native features run only on the server; on the client, `jolt.Load` returns `nil, reason`. Use `IsAvailable` when your addon should keep working without Jolt:

```lua
require("jolt")
local available, reason = jolt.IsAvailable("gravity")
if not available then
    print("Jolt gravity is unavailable: " .. reason)
    return
end
```

The modules operate on the physics world of the running Jolt backend; they never create a Jolt world of their own.

- Build the modules: [build guide](../docs/build.md#lua-modules-and-addon)
- API overview and C++ access: [physics API](../docs/physics-api.md)
- Write your own module: [feature export example](../examples/physics/export/README.md)
