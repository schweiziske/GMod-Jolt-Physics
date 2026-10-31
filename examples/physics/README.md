# Physics Examples

[Home](../../README.md)

| Example | What it shows |
| --- | --- |
| [`export/`](export/README.md) | Exporting a new Jolt feature end to end: public interface, backend, `gmsv_joltfeature` module and Lua caller |
| [`lua/attractor.lua`](lua/attractor.lua) | Owning an attraction field from Lua |
| [`lua/cloth.lua`](lua/cloth.lua) | Creating a cloth and reading it into reusable tables |
| [`native/`](native/) | Driving an attraction field from C++ through `VJolt::SDK` |

To add a new capability, start with `export/`. The other examples call existing features; their parameters are in the [physics API](../../docs/physics-api.md).

## Lua: attraction field

Copy the files in `lua/` into your addon, for example to `lua/my_toy/`, and use them on the server:

```lua
local Attractor = include("my_toy/attractor.lua")
local toy, reason = Attractor.Create({
    center = prop:GetPos(), radius = 256, strength = 600, priority = 10,
})
assert(toy, reason)
prop.MyAttractor = toy
prop:CallOnRemove("MyAttractor", function() toy:Destroy() end)
-- Move the center: toy:SetCenter(newPosition)
-- Stop early: toy:Destroy()
```

`prop` is the entity that owns the field, so the field is destroyed together with it; garbage collection alone would release it only eventually. `SetCenter` reuses one update table. Errors from `Create` and `Destroy` are passed through unchanged, and if `Destroy` fails while the field is still valid, the wrapper keeps it so you can retry.

## Lua: cloth

```lua
local Cloth = include("my_toy/cloth.lua")
local cloth, reason = Cloth.Create({
    pos = Vector(0, 0, 128), width = 64, height = 64, vertexMass = 1,
    iterations = 5, damping = 0.1,
})
assert(cloth, reason)
prop.MyCloth = cloth
prop:CallOnRemove("MyCloth", function() cloth:Destroy() end)
local snapshot = assert(cloth:Read())
print(snapshot.positions[1], snapshot.positions[2], snapshot.positions[3])
-- Later Read() calls reuse the snapshot and its arrays, and skip the vertex copy while the cloth sleeps.
-- cloth.body:AddImpulse(Vector(0, 100, 0))
```

This is a native cloth with four vertices and two triangles, pinned at its top two vertices; for more detail, generate a denser connected mesh yourself. The example covers creating, reading and cleaning up on the server only; `addons/jolt/lua/jolt_softbody/` shows client rendering and networking. Don't call `Read` every frame without a budget.

## C++: attraction field

`native/` builds `vjolt_attractor`, a static library whose `Attractor` class creates, moves and destroys an attraction field through `VJolt::SDK`, with no Lua binding involved. Link it into your module and keep an `Attractor` alive as long as your gameplay object; its destructor destroys the field.

```cpp
Attractor toy;
VJoltGravityField field;
field.center = {0, 0, 128};
field.halfExtent = {256, 256, 256};
field.strength = 600;
const char *error;
if (!toy.Create(0, field, error)) { /* report error */ }
// toy.SetCenter({256, 0, 128});
// toy.Destroy();
```

To build it on its own (Windows, MSVC):

```powershell
cmake -S examples/physics/native -B .cache/physics-consumer -A x64
cmake --build .cache/physics-consumer --config Release
```

The library uses the static CRT (`/MT`). Added to your project with `add_subdirectory`, it reuses an existing `VJolt::SDK` target. Otherwise it includes `cmake/VJoltSDK.cmake` when it sits inside this repository, and falls back to `find_package(VJoltSDK CONFIG REQUIRED)` elsewhere, for example in the copy installed with the SDK; then set `CMAKE_PREFIX_PATH` to the SDK install prefix. Use `Attractor` on the environment's owner thread, outside the physics step, and destroy it before the backend unloads.
