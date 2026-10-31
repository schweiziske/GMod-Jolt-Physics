# Jolt Gravity

[Home](../README.md)

Change gravity for the whole world, a single prop or an area. The Lua API can also create attraction and repulsion fields, which black holes are built on. For setup, see the [installation guide](../gmsv_jolt/INSTALL.md).

## Gravity Controller

Find it under **Q → Weapons → Jolt → Jolt Gravity Controller** (`weapon_jolt_gravity`).

| Input | Action |
| --- | --- |
| Hold E + move the mouse | Aim the direction arrow (it starts pointing down); release E to keep it |
| Left click | Apply gravity along the arrow. World mode sets world gravity, prop mode overrides the prop you aim at, area mode places a field where you aim. |
| Right click | World mode: restore the map's gravity. Prop mode: clear the override and reset the factor to 1. Area mode: remove your latest field. |
| R | Cycle world → prop → area mode (admins only; ignored while holding E) |
| H | Switch area fields between sphere and box |
| Z | Undo; in prop mode the prop stays |

E doesn't use objects while you hold the controller. Non-admins can only use area mode, with a strength up to 1500 and a radius or half-size up to 512. Prop mode changes every physics bone of the entity.

Fields survive their owner's death; undo, disconnecting and map cleanup remove them, and map cleanup also resets world and prop gravity. The server ConVar `vjolt_gravity_fields_per_player` (default 16, range 1–128) caps fields per player.

Your settings, under **Q → Options → Jolt → Gravity Fields**:

| ConVar | Default | Range | Meaning |
| --- | --- | --- | --- |
| `vjolt_gravity_strength` | 600 | 0–10000 | Acceleration, Source units/s² |
| `vjolt_gravity_radius` | 256 | 16–32767 | Sphere radius or box half-size, Source units |
| `vjolt_gravity_show_fields` | 1 | 0–1 | Show field outlines on your screen |
| `vjolt_gravity_show_direction` | 1 | 0–1 | Show direction arrows on your screen; always shown while aiming |

## Rules

- An object uses its own override if it has one, else the field containing its center of mass, else world gravity; the result is multiplied by its factor.
- Overlapping fields don't add up: the highest `priority` wins, then the lowest `slot`. Controller fields use priority 0 and black holes 50.
- Fields act on dynamic physics objects and vehicles, not on player movement or NPC AI. Players and NPCs can't have their own gravity.
- `EnableGravity(false)` keeps the factor and override for when gravity is turned back on.
- Source saves and the duplicator don't keep prop gravity.

## Lua API

Server only. Positions are Source world coordinates; accelerations are Source units/s².

```lua
require("jolt")
local available, reason = jolt.IsAvailable("gravity")
assert(available, reason)
local gravity = jolt.gravity

local original = gravity.GetWorld() -- physics environment 0
gravity.SetWorld(Vector(0, 0, -300))

local phys = ent:GetPhysicsObject()
gravity.SetFactor(phys, 2)
gravity.SetGravity(phys, Vector(600, 0, 0)) -- override
phys:EnableGravity(false)
phys:EnableGravity(true)      -- factor 2 and the override are kept
gravity.SetGravity(phys, nil) -- back to field or world gravity, still doubled

local previous = JoltGravity.SetEntity(ent, Vector(0, 600, 0), 1) -- every physics bone
JoltGravity.RestoreObjects(previous)

local field = gravity.CreateField({
    shape = 0, mode = 0, -- sphere, directional
    center = Vector(0, 0, 128), radius = 256,
    gravity = Vector(0, 600, 0), priority = 10,
})
field:Update({mode = 1, strength = 600}) -- now it attracts
local config = field:GetConfig()
field:Destroy()
gravity.SetWorld(original)
```

| Function | Description |
| --- | --- |
| `Available()` | Whether a matching backend is loaded |
| `GetWorld([env])` / `SetWorld(vector[, env])` | World gravity; `env` is the physics environment index, default 0 |
| `GetObject(phys)` | `{factor, enabled, gravity}`; `gravity` is `nil` without an override |
| `Configure(phys, {factor, enabled, gravity})` | Replace the whole object configuration; omitted keys mean factor 1, enabled, no override |
| `SetFactor(phys, factor)` | Factor, −1024 to 1024 |
| `SetGravity(phys, vector or nil)` | Set or clear the override |
| `GetEffectiveGravity(phys)` | Current acceleration including the factor; zero while gravity is off |
| `Sample(position[, env])` | World or field gravity at a point, without any object's override or factor |
| `CreateField(config[, env])` | Create a field and return its handle |
| `GetFields([env])` | Array of field configurations, as from `GetConfig()` |
| `GetRevision()` | Counter that changes with any world, field or object change |
| `Reset([env])` | Remove all fields, overrides and factors; keep enabled states and world gravity |

| Field key | Default | Meaning |
| --- | --- | --- |
| `shape` | 0 | 0 sphere, 1 box |
| `mode` | 0 | 0 directional, 1 attract, 2 repel |
| `center` | `Vector(0, 0, 0)` | Field center |
| `radius` | 256 | Sphere radius |
| `halfExtent` | `Vector(256, 256, 256)` | Box half-size; boxes are aligned with the world axes |
| `gravity` | `Vector(0, 0, -600)` | Acceleration in a directional field |
| `strength` | 600 | Attract or repel acceleration, 0–10,000,000; the same throughout the field, zero at the exact center |
| `priority` | 0 | Integer, ±1,000,000 |

A field handle has `IsValid()`, `GetConfig()`, `Update(config)`, which merges the given keys, and `Destroy()`, which returns `false` if the field is already gone. `GetConfig()` adds `slot` and a string `id`; keep `id` a string, since Lua numbers can't hold every 64-bit value. Garbage collection destroys the field, so keep a reference. The backend holds up to 1024 fields.

Wrong argument types, non-finite numbers in a configuration table, an invalid `shape`, `mode` or `priority`, and an invalid environment index raise a Lua error; other failures return `nil, reason`. During a physics step, for example in a collision callback, every call fails except `field:Destroy()`, which completes before the next step.

`JoltGravity`, unlike the native `jolt.gravity`, is the addon's helper table from `addons/jolt/lua/autorun/jolt_gravity.lua`. `JoltGravity.SetEntity(ent, gravity, factor)` sets the override (`nil` clears it) and, if given, the factor on all physics bones at once, rolling back on failure. It rejects players and NPCs and returns the previous configurations for `JoltGravity.RestoreObjects(previous)`, or `nil, reason`.

C++ interface: `VJoltGravityBridge001` in [`public/vjolt/gravity.h`](../public/vjolt/gravity.h).
