# Rails and PathConstraint

[Home](../README.md)

Move a prop along a smooth 3D curve: back and forth, around a loop, at constant speed, to a target position, or sliding freely. For setup, see the [installation guide](../gmsv_jolt/INSTALL.md).

## Path Track tool

Open **Q → Tools → Jolt → Path Track - PathConstraint**.

1. Freeze the prop and orient it the way it should face.
2. Left-click to place nodes: at least 2, or 3 for a closed track. Arrows show the forward direction; nodes sit 8 units off the surface by default.
3. Right-click the prop to attach it. By default its center of mass snaps to the nearest point on the track, and it stays frozen.
4. Unfreeze it. Open tracks travel end to end automatically and closed tracks loop; **Track motion** also offers constant speed, a target position and free sliding with the motor off.

| Input | Action |
| --- | --- |
| Left click | Add a node at the end |
| E + left click | Select a nearby node; repeat to move it to the crosshair |
| Shift + left click | Insert a node after the selected one |
| R | Delete the selected node, or the last node if none is selected |
| H | Start a new track, clearing the draft and selection |
| Right-click a prop | Attach it, or apply the draft if it is the selected track's prop |
| E + right-click a prop | Load its track for editing (your own tracks only) |
| Shift + right-click a prop | Use that prop as a moving, rotating reference frame |
| Shift + right-click the map | Fix the track to the world again |
| Z, or the Jolt Path Track cleanup | Remove the track constraint; the prop stays |

The panel lists the nodes for selecting, reordering and typing in XYZ coordinates. **Apply track and all settings** rebuilds the constraint; **Update motion and visibility only** keeps the nodes, reference frame and rotation mode. Switching tools keeps the draft and cancels a pending node move.

**Hold position / resume motion** holds the prop with a position motor, so the motor force must be above 0; collisions can still push the prop away if the motor is too weak. **Reverse motion / mirror target position** negates the speed and, in target mode, turns the target `p` into `1 - p`. Free sliding switches off the drive but keeps the prop on the track.

### Orientation and reference frames

**Follow track direction** keeps the orientation the prop had relative to the track when attached, so orient the model first. **Object rotation** can also allow free rotation, allow rotation only around the tangent, normal or binormal, or keep the orientation fixed relative to the reference.

A reference prop may be frozen or moving and takes the constraint's reaction forces. Both props need CanTool/CPPI permission; players, NPCs, vehicles, triggers and shadow-controlled objects can't be used. Collisions aren't changed, so use Sandbox's No Collide tool on overlapping parts.

With center-of-mass snapping off, the prop keeps its offset and swings around the track in corners; large offsets can make it less stable.

### Presets and duplication

Presets are saved on your client in `data/jolt_paths/*.json`: the settings, plus the nodes relative to the first node and to your horizontal view direction. Loading one puts the first node at the crosshair, rotated to your view plus **Extra rotation when loading preset (degrees)**; nothing changes physically until you attach a prop. Names allow letters, digits, underscores and hyphens, up to 48 characters.

The standard duplicator copies the prop, track and reference prop together. Pasted props keep their saved pose instead of snapping again.

## Lua API

Server only.

```lua
require("jolt")
assert(jolt.constraints.PathsAvailable(), "requires a path-capable backend and gmsv_joltconstraints")

local path, reason = jolt.constraints.CreatePath(prop:GetPhysicsObject(), {
    Vector(0, 0, 100), Vector(256, 0, 100), Vector(512, 128, 200),
}, {
    closed = false, snap = true, rotation = "follow", smoothing = 0.75,
    mode = "velocity", speed = 128, maxForce = 100000,
})
assert(path, reason)
prop.MyPath = path -- Keep a reference: garbage collection destroys the constraint.

local state = assert(path:GetState())
print(state.progress, state.length, state.velocity)
assert(path:SetMotor({mode = "position", target = 0.5, maxForce = 100000,
    frequency = 2, damping = 1}))
local samples = assert(path:Sample(129)) -- World position, tangent and normal.
-- path:Destroy()
```

`CreatePath(followerPhys, worldNodes[, settings][, referencePhys])` returns a path or `nil, reason`. Without a reference the track is fixed to the world; with one, the nodes (world coordinates at creation) move with that object. End-to-end travel is a tool feature: the tool runs a velocity motor and reverses it at each end.

| Setting | Default | Range |
| --- | --- | --- |
| `closed` / `snap` | `false` / `true` | Booleans |
| `rotation` | `"follow"` | `"free"`, `"tangent"`, `"normal"`, `"binormal"`, `"follow"`, `"fixed"` |
| `smoothing` | 0.75 | 0.05–1 |
| `mode` | `"off"` | `"off"`, `"velocity"`, `"position"` |
| `speed` | 128 | ±32767 Source units/s |
| `target` | 0 | 0–1, a fraction of the track's length |
| `maxForce` | 100000 | 0–1e9 kg·Source-unit/s²; 0 disables the drive |
| `friction` | 0 | 0–1e9, same unit; applies while the motor is off |
| `frequency` / `damping` | 2 Hz / 1 | 0.1–20 / 0–5; position mode only |

Wrong argument types, non-boolean `closed`/`snap`, unknown `rotation`/`mode` names and non-finite numbers raise a Lua error; out-of-range values return `nil, reason`.

`SetMotor(table)` takes `mode`, `speed`, `target`, `maxForce`, `friction`, `frequency` and `damping`, using the defaults above for omitted keys, so leaving out `mode` turns the motor off. To change one value, edit `GetState().motor` and pass it back. Position mode isn't limited by `speed`; shape its approach with `maxForce`, `frequency` and `damping`.

`GetState()` returns `kind` (`"path"`), `enabled`, `closed`, `nodeCount`, `progress` (0–1), `distance`, `length`, `velocity` and `motor`. `Sample(count)` returns 2–513 points (default 129) spaced evenly along the track, each with world `position`, `tangent` and `normal`. Lengths are in Source units. `IsValid()` and `Destroy()` return booleans; destroying twice is safe. `jolt.constraints.GetPathVersion()` returns the backend interface version.

Call the API outside the physics step; `Destroy()` also works in physics callbacks. Removing either body, replacing its collision shape or making it unsupported removes the constraint.

## Limits

- 2–32 nodes, at least 3 for a closed track. Adjacent nodes, including a loop's last and first, must be at least 1 unit apart, and the track can't nearly double back on itself.
- The backend holds 256 tracks. The tool allows 8 per player (`sbox_maxjolt_paths`, range 0–64), which also count toward the `constraints` limit, and one track per physics object. A player's tracks are removed when they disconnect.
- Tool nodes and editing reach are limited to ±32767 units. The API accepts coordinates up to ±1,048,576 and a total length up to 262,144.
- Clients draw up to 24 nearby tracks marked **Show this track to players**; `jolt_path_show 0` hides them, but not your own draft.

C++ interface: `VJoltPathBridge001` in [`public/vjolt/path.h`](../public/vjolt/path.h).
