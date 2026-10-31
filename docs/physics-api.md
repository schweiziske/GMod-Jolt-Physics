# Physics API

[Home](../README.md)

Start here to use the extra physics features from Lua or C++, or to export a new Jolt capability yourself.

## Export a new Jolt feature

To build something like `gmsv_joltgravity` or `gmsv_joltconstraints`, follow the [feature export example](../examples/physics/export/README.md). It passes Jolt body counts through every layer:

1. Define a versioned interface and its cross-DLL structures in `public/vjolt/`.
2. Implement it in `vphysics_jolt/bridge/` on the current environment's `JPH::PhysicsSystem`, and register it with the backend's `CreateInterface` factory through `EXPOSE_SINGLE_INTERFACE_GLOBALVAR`.
3. Add the source file to the backend's CMake source list.
4. In your own `gmsv_jolt*` module, discover and validate the interface, then bind the Lua functions, argument checks and cleanup.
5. Optionally register a namespace in the `jolt` Lua facade. Build and ship the backend and the module together.

Jolt objects never leave the backend; modules only borrow its interfaces.

## Call existing features

Native features run only on the server. Lua code needs the backend, the feature's `gmsv_jolt*` module and `addons/jolt` installed there. C++ code needs only the backend: link `VJolt::SDK` and discover the interfaces at run time.

`require("jolt")` loads no native code by itself. `jolt.IsAvailable(name)` loads one feature and checks its backend interface, returning `true`, or `false, reason`; `jolt.Load(name)` returns the feature table, or `nil, reason`; reading `jolt.<name>` also loads the feature but raises a Lua error if it is unavailable. Clients can only list features with `jolt.ListFeatures()`.

Positions are in Source units, masses in kg and accelerations in Source units/s². Each header in `public/vjolt/` states its own threading, unit and lifetime rules. In the examples below, `prop` is an entity your addon controls.

### Lua: attraction field

```lua
require("jolt")
local available, reason = jolt.IsAvailable("gravity")
assert(available, reason)

local field = assert(jolt.gravity.CreateField({
    center = Vector(0, 0, 128), radius = 256,
    shape = 0, mode = 1, -- sphere, attract
    strength = 600, priority = 10,
}))
-- Keep a reference: garbage collection destroys an unreferenced field.
assert(field:Update({center = Vector(256, 0, 128)}))
-- Call field:Destroy() when you are done.
```

Black holes are built on this field; the addon adds the timing, visuals, permissions and networking. Fields act only on dynamic rigid bodies: they don't affect soft bodies, player movement or NPC AI. [`attractor.lua`](../examples/physics/lua/attractor.lua) wraps ownership and cleanup.

### Lua: put a prop on a rail

```lua
require("jolt")
assert(jolt.constraints.PathsAvailable())
local path, reason = jolt.constraints.CreatePath(prop:GetPhysicsObject(), {
    Vector(0, 0, 100), Vector(256, 0, 100), Vector(512, 128, 200),
}, {
    mode = "velocity", speed = 128, maxForce = 100000,
    rotation = "follow", snap = true,
})
assert(path, reason)
prop.MyJoltPath = path
prop:CallOnRemove("MyJoltPath", function()
    path:Destroy()
end)
-- Creating the rail never unfreezes the prop. path:Destroy() removes the rail and keeps the prop.
```

## Available features

| Feature | Lua namespace | Lua reference | C++ header |
| --- | --- | --- | --- |
| World and per-object gravity; directional, attraction and repulsion fields | `jolt.gravity` | [Gravity](../gmsv_joltgravity/README.md) | `vjolt/gravity.h` |
| Gears, racks and their motors | `jolt.constraints` | [Gears and racks](../gmsv_joltconstraints/README.md) | `vjolt/constraints.h` |
| Rails along curved paths, with motors | `jolt.constraints` | [Rails](../gmsv_joltconstraints/PATH.md) | `vjolt/path.h` |
| Cloth and closed pressure meshes | `jolt.softbody` | [Soft bodies](../gmsv_joltsoftbody/README.md) | `vjolt/softbody.h`, `vjolt/softbody_tools.h` |

## Soft bodies

`jolt.softbody.Create(vertices, triangles[, settings])` returns soft-body userdata, or `nil, reason`. Vertex positions are relative to `settings.pos` and `settings.angles`; triangle indices start at 1.

```lua
require("jolt")
assert(jolt.IsAvailable("softbody"))
local body, reason = jolt.softbody.Create({
    {pos = Vector(-32, 0, 0), mass = 1, pinned = true},
    {pos = Vector(32, 0, 0), mass = 1, pinned = true},
    Vector(-32, 0, -64), Vector(32, 0, -64),
}, {{1, 3, 2}, {2, 3, 4}}, {
    pos = Vector(0, 0, 128), iterations = 5, pressure = 0,
})
assert(body, reason)
prop.MySoftBody = body
prop:CallOnRemove("MySoftBody", function() body:Destroy() end)
local snapshot = {positions = {}, pinned = {}}
assert(body:ReadSnapshot(snapshot, false)) -- full read, even while asleep
-- Later reads: body:ReadSnapshot(snapshot, true) reuses the arrays and skips the copy while asleep.
-- positions is flat, in world coordinates: x1, y1, z1, x2, y2, z2, ...
```

A bare `Vector` vertex has mass 1 and is not pinned. The mesh must be connected and consistently wound. `pressure > 0` needs a closed mesh with outward winding; cloth keeps 0.

| Setting | Meaning |
| --- | --- |
| `pos`, `angles`, `velocity` | Initial world position, `Angle` orientation and initial world velocity |
| `environment` | Physics environment index; default 0 |
| `iterations` | Solver iterations, 1–32 |
| `edgeCompliance`, `bendCompliance` | Edge and bend compliance; smaller is stiffer |
| `pressure` | Gas energy nRT in kg·Source-unit²/s², not pascals |
| `gravityFactor`, `damping` | Gravity multiplier, linear damping |
| `friction`, `restitution` | Friction, restitution |
| `vertexRadius`, `maxVelocity` | Vertex collision radius, vertex speed limit |
| `allowSleeping`, `doubleSided`, `dihedralBend` | Switches: allow sleeping, double-sided triangles, dihedral-angle bending |

Omitted settings take the defaults from `softbody.h`, and the backend validates every value. `jolt.softbody.GetLimits()` returns the native caps: 8192 vertices and 16384 triangles per body, 256 bodies and 131072 vertices in total. The bundled Sandbox tool enforces lower limits through ConVars.

| Methods | Calls |
| --- | --- |
| State and cleanup | `IsValid()`, `Destroy()`, `GetState()`, `GetHandle()` |
| Reads into reusable tables | `ReadState(out)`, `ReadSnapshot(out, skipSleeping)`, `ReadVertex(index, out)` |
| Reads that allocate new tables | `GetVertices([first, count])`, `GetTriangles([first, count])` |
| Single vertices | `SetVertexPinned(index, pinned)`, `SetVertexVelocity(index, velocity)`, `AddVertexImpulse(index, impulse)` |
| Whole body | `SetVelocity(velocity)`, `AddImpulse(impulse)`, `SetPressure(value)`, `SetGravityFactor(value)`, `Wake()` |
| Only when the backend provides `VJoltSoftBodyTools001` | `Trace(start, delta[, extents][, out])`, `SetPlasticity(yield, creep, maxStrain)`, `AddImpulseAt(position, impulse, radius)` |

`Create` makes no rendering, networking or Source player collision. [`cloth.lua`](../examples/physics/lua/cloth.lua) shows reusable reads, and the bundled soft-body addon shows a complete presentation. Defaults, ranges, units and error behavior are in the [soft-body reference](../gmsv_joltsoftbody/README.md).

## C++: access the running backend

```cmake
include("${VJOLT_ROOT}/cmake/VJoltSDK.cmake")  # VJOLT_ROOT: a GMod-Jolt-Physics checkout
# With an installed SDK instead: find_package(VJoltSDK CONFIG REQUIRED)
target_link_libraries(my_toy PRIVATE VJolt::SDK)
```

```cpp
#include <vjolt/gravity.h>
#include <vjolt/loader.h>

auto bridge = VJoltSDK::DiscoverVersionedInterface<IVJoltGravityBridge>(
    VJOLT_GRAVITY_BRIDGE_VERSION, kVJoltGravityBridgeVersion);
if (!bridge) {
    // bridge.ErrorMessage() says why.
    return;
}
VJoltGravityField settings;
settings.mode = VJoltGravityAttract;
settings.center = {0, 0, 128};
settings.halfExtent = {256, 256, 256};
settings.strength = 600;
const auto field = bridge.api->CreateField(0, settings);
// 0 means failure. Keep the handle and call bridge.api->DestroyField(field) when done.
```

`VJolt::SDK` is header-only and requires C++17. `loader.h` is Windows-only: it looks for a backend that is already loaded, checks the interface name and version, and returns a borrowed pointer. It never loads a DLL, creates a world or links Jolt. Destroy your handles and finish all calls before the backend unloads. [`native/`](../examples/physics/native/) is a complete consumer, and the [public interface guide](../public/vjolt/README.md) lists every interface.

## Ownership and cleanup

- Call from the server thread that owns the physics environment (the Lua thread), outside the physics step. `Destroy` may also be called from physics callbacks; the removal is deferred to a safe point. Other exceptions are noted in each header.
- Keep a reference to every field, mechanism, path and soft body you create, and destroy it when its entity is removed, the map is cleaned up or your feature shuts down. Entities and PhysObjs are borrowed; check that they are valid before use.
- C++ handles are 64-bit values that carry a generation or sequence number, so a stale handle fails instead of reaching a newer object; 0 means creation failed. Lua code holds module userdata instead. String IDs, such as a gravity field's `id` or a soft body's `GetHandle()`, must not be converted to numbers.
- Soft bodies are standalone native objects without an `IPhysicsObject`. Their vertex and triangle indices start at 0 in C++ and at 1 in Lua. Reuse snapshot tables and buffers, and limit how often you read.
