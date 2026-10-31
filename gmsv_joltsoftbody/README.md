# Jolt Soft Body

[Home](../README.md)

The Soft Body Workshop places cloth and inflatable soft balls in Sandbox, and the `jolt.softbody` Lua API builds soft bodies from your own triangle meshes. For setup, see the [installation guide](../gmsv_jolt/INSTALL.md); for how the workshop networks and draws them, see the [soft-body client notes](../addons/jolt/docs/SOFTBODY_CLIENT.md).

## Soft Body Workshop

Open **Q → Tools → Jolt → Soft Body Workshop** and pick a mode in the panel.

**Create soft body**: choose cloth or a soft ball and set its size, mesh quality, total mass (default 4 kg, shared by all vertices), softness, inflation (balls), pins (cloth: none, top corners or the entire top edge), color, rotation and player collision. The solver settings below them are capped by the server settings. The panel shows the vertex and triangle count and the bandwidth per nearby player. Left-click a surface within 600 units to place the body; Z and Sandbox cleanup remove it.

Softness scales edge and bend compliance logarithmically. Inflation sets a ball's gas energy (nRT) from its mass and size; it is neither a pressure in pascals nor a target volume.

**Edit vertices**: aim at a soft body within 600 units. Blue marks the selected vertex, gold the pinned ones.

| Input | Action |
| --- | --- |
| Hold left click | Drag the selected vertex; it is pinned while dragged and gets its original pin state back on release |
| Right click | Pin or unpin the selected vertex; pins hold it fixed in the world |
| R, or switching tools | Finish dragging |

Only the owner or an admin can edit a soft body, subject to CanTool and CPPI, and only one player can drag it at a time. The server redoes the aim trace and checks walls and distance; clients never send vertex numbers or positions. Death, disconnecting, removing the body or reloading Lua ends a drag. The physics gun can't grab soft bodies.

With **Player collision** on, the body blocks players: its entity answers Source traces against the live mesh, natively on the server and against the drawn mesh on each client. With it off (the default), players walk through, and the server checks overlaps at a limited rate and pushes the body with an impulse from the player's mass and velocity. That approximates contact; it doesn't solve it continuously. Damage forces such as bullets and explosions also push the body near the hit point.

| Mesh quality | Cloth vertices / triangles | Ball vertices / triangles | KiB/s per nearby player, cloth / ball |
| --- | --- | --- | --- |
| Low | 64 / 98 | 42 / 80 | 6.2 / 4.2 |
| Standard | 256 / 450 | 162 / 320 | 23.4 / 15.0 |
| High | 576 / 1058 | 642 / 1280 | 52.1 / 58.0 |

Bandwidth is the snapshot payload at 15 Hz while the body moves, without network overhead. Higher quality costs more server simulation, bandwidth and client drawing alike.

A body within 1200 units of a player updates at up to 15 Hz, farther away at 5 Hz, and not at all beyond 3000 units or outside the player's PVS. Resting bodies send nothing. Clients interpolate with about 100 ms of delay and don't predict soft-body physics. The duplicator doesn't copy soft bodies.

### Server settings

Admins can also change these under **Q → Options → Jolt → Soft body**. Size and solver limits apply to new soft bodies; network and push settings apply at once.

| ConVar | Default | Meaning |
| --- | --- | --- |
| `sbox_maxjolt_softbodies` | 3 | Soft bodies per player |
| `jolt_softbody_maxrecords` | 32 | Soft bodies on the server; the addon never exceeds 32 |
| `jolt_softbody_maxvertices` | 4096 | Vertices on the server in total |
| `jolt_softbody_maxvertices_per_body` / `jolt_softbody_maxtriangles_per_body` | 1024 / 2048 | Vertices / triangles per body |
| `jolt_softbody_iterations` | 5 | Maximum solver iterations |
| `jolt_softbody_damping` | 0.35 | Maximum damping |
| `jolt_softbody_friction` / `jolt_softbody_restitution` | 0.6 / 0.05 | Maximum friction / restitution |
| `jolt_softbody_vertex_radius` | 0.5 | Maximum vertex collision radius |
| `jolt_softbody_max_velocity` | 2000 | Maximum vertex speed, Source units/s |
| `jolt_softbody_gravity_factor` | 1 | Largest gravity factor allowed, by absolute value (at least 1) |
| `jolt_softbody_player_push_rate` | 15 | Overlap checks per second for each player and body |
| `jolt_softbody_player_push_budget` | 32 | Player/body pairs checked per tick |
| `jolt_softbody_player_push_impulse` | 1 | Player push impulse multiplier |
| `jolt_softbody_net_kib` | 96 | Network budget per player, KiB/s |
| `jolt_softbody_near_distance` / `jolt_softbody_view_distance` | 1200 / 3000 | Range of near-rate updates / of any updates |
| `jolt_softbody_near_rate` / `jolt_softbody_far_rate` | 15 / 5 | Near / far snapshot rate, Hz |
| `jolt_softbody_interest_rate` | 4 | How often each player's set of nearby bodies is refreshed, Hz |

### Client settings

These are under **Q → Options → Jolt → Soft body**; the first three also appear in the tool panel's **Display and performance** section.

| ConVar | Default | Meaning |
| --- | --- | --- |
| `jolt_softbody_viewdistance` | 3000 | Draw distance, 256–16384 |
| `jolt_softbody_drawrate` | 0 | Interpolation updates per second, up to 144; 0 follows the frame rate, and values below 15 act as 15 |
| `jolt_softbody_wireframe` | 0 | Show the selected body's wireframe in Edit vertices mode |
| `jolt_softbody_collision_profile` | 0 | Time client collision queries for `jolt_softbody_client_stats` |

## Lua API

Server only. `jolt.softbody` provides `Create(vertices, triangles[, settings])`, `Available()`, `GetVersion()` and `GetLimits()`, which returns `{verticesPerBody, trianglesPerBody, bodies, totalVertices}`.

```lua
require("jolt")
local available, reason = jolt.IsAvailable("softbody")
assert(available, reason)

-- Cloth: two pinned corners.
local body, err = jolt.softbody.Create({
    {pos = Vector(-32, -32, 0), mass = 1, pinned = true},
    {pos = Vector( 32, -32, 0), mass = 1, pinned = true},
    {pos = Vector( 32,  32, 0), mass = 1},
    {pos = Vector(-32,  32, 0), mass = 1},
}, {
    {1, 2, 3}, {1, 3, 4},
}, {
    environment = 0,
    pos = Vector(0, 0, 128),
    angles = Angle(0, 0, 0),
    edgeCompliance = 0.00001,
    bendCompliance = 0.001,
    pressure = 0,
})
assert(body, err)

-- prop is an existing entity that owns the body: keep a reference and destroy the body with it.
prop.MySoftBody = body
prop:CallOnRemove("MySoftBody", function() body:Destroy() end)
local snapshot = {positions = {}, pinned = {}}
assert(body:ReadSnapshot(snapshot, false))
body:AddImpulse(Vector(0, 0, 100))
```

`Create` takes a connected triangle mesh of 3–8192 vertices and 1–16384 triangles. A vertex is a `Vector` (1 kg, not pinned) or `{pos = Vector, mass = number, pinned = boolean}` with a mass of 0.001–10000 kg; use `pinned`, not a huge mass, to fix it. Vertex positions are relative to `pos` and `angles`; initial velocity and everything read back are in world coordinates. Lua indices start at **1**, C++ indices at **0**. The input arrays are copied.

The mesh must be connected and use every vertex. Faces can't repeat, shared edges must be wound consistently, and an edge can belong to at most two faces. Degenerate triangles, very short edges and nearly collinear faces are rejected. Open meshes suit cloth; closed meshes must be wound outward. Every vertex must end up within ±1,048,576 Source units.

The smallest closed mesh, a pressurized tetrahedron:

```lua
local ball, reason = jolt.softbody.Create({
    Vector(0, 0, 0), Vector(32, 0, 0),
    Vector(0, 32, 0), Vector(0, 0, 32),
}, {
    {1, 3, 2}, {1, 2, 4}, {1, 4, 3}, {2, 3, 4},
}, {
    pos = Vector(0, 0, 128),
    pressure = 10000,
    edgeCompliance = 0.01,
    bendCompliance = 0.01,
})
assert(ball, reason)
```

`pressure` is Jolt's gas energy nRT, in kg·Source-unit²/s²; divided by the current volume it gives the pressure, so it is neither pascals nor a fixed volume. Open meshes reject a nonzero pressure, both at creation and in `SetPressure`.

| Setting | Default | Range and meaning |
| --- | --- | --- |
| `environment` | 0 | VPhysics environment index; the body stays in it |
| `pos`, `angles` | Zero | Initial position and Source Euler angles |
| `velocity` | Zero | Initial world velocity of unpinned vertices, Source units/s, up to `maxVelocity` |
| `iterations` | 5 | Solver iterations, 1–32 |
| `edgeCompliance` | 0.00001 | Compliance of edges and generated shear edges, 0–1, s²/kg |
| `bendCompliance` | 0.001 | Bend compliance, 0–1; s²/kg for distance bending |
| `dihedralBend` | `false` | Bend by the angle between adjacent triangles instead |
| `pressure` | 0 | nRT of a closed mesh, 0–1e9 |
| `gravityFactor` | 1 | Multiplier on world gravity, −100 to 100 |
| `damping` | 0.1 | Linear damping, 0–10 |
| `friction`, `restitution` | 0.5, 0 | 0–1 |
| `vertexRadius` | 0.5 | Vertex collision radius, 0–64 Source units |
| `maxVelocity` | 4000 | Vertex speed limit, 1–100000 Source units/s |
| `allowSleeping` | `true` | Let the body sleep |
| `doubleSided` | `true` | Treat faces as double-sided in collision queries |

Compliance is inverse stiffness: larger is softer, 0 is rigid. With `dihedralBend`, `bendCompliance` applies to angles, in s²/(kg·m²) with Jolt metres, so it isn't comparable with distance bending.

| Method | Behavior |
| --- | --- |
| `IsValid()` / `Destroy()` | Check the handle or destroy the body; destroying twice returns `false` |
| `GetState()` | Vertex, triangle and pinned counts, `active`, `closed`, `mass`, `pressure`, `gravityFactor`, `volume`, `pos`, `boundsMin`, `boundsMax`, `simulationTime` |
| `ReadState(out)` | Fills a reused table with the counts, `active`, `pressure`, `simulationTime` and bounds `minX` … `maxZ` |
| `GetVertices([first, count])` | New `{pos, velocity, mass, pinned}` array; allocates per vertex |
| `GetTriangles([first, count])` | New array of index triples |
| `ReadSnapshot(out[, skipSleeping])` | Fills `out.positions` (flat x, y, z) and `out.pinned` in place, plus the `ReadState` fields and `updated` |
| `ReadVertex(index, out)` | Fills `x`, `y`, `z`, `vx`, `vy`, `vz`, `mass`, `pinned` in place |
| `SetVertexPinned(index, pinned)` | Pin or release a vertex; both clear its velocity, and releasing restores its creation mass |
| `SetVertexVelocity(index, velocity)` | Set one vertex's world velocity; works on pinned vertices too |
| `SetVelocity(velocity)` | Set every unpinned vertex's world velocity |
| `AddImpulse(impulse)` | Spread an impulse over the unpinned vertices by mass, so all get the same velocity change |
| `AddVertexImpulse(index, impulse)` | Impulse on one unpinned vertex |
| `SetPressure(value)` / `SetGravityFactor(value)` | Change pressure or gravity factor and wake the body; cloth only takes pressure 0 |
| `Wake()` | Wake the body |
| `GetHandle()` | Native handle as a decimal string; don't convert it to a number |
| `Trace(start, delta[, extents][, out])` | Optional. Trace the surface (a box trace with `extents`); returns a Source-style result with `Hit`, `Fraction`, `HitPos`, `HitNormal`, `StartSolid`, `AllSolid`, `FractionLeftSolid` and 1-based `Triangle` and `Vertex` (0 if none) |
| `SetPlasticity(yield, creep, maxStrain)` | Optional. Permanent deformation, see below |
| `AddImpulseAt(position, impulse, radius)` | Optional. Spread an impulse over unpinned vertices, falling off linearly within `radius` (0.01–4096); the total is conserved |

Optional methods exist only if the backend provides `VJoltSoftBodyTools001`.

`Create` returns the body or `nil, reason`; other calls also return `nil, reason` on failure, even for invalid argument types, while `IsValid` and `Destroy` return booleans. Garbage collection, module unload and environment destruction release a body, but destroy it explicitly together with its owner. Call everything outside the physics step; in physics callbacks only `IsValid` and `Destroy` work, and removal waits for a safe point.

Impulses are in kg·Source-unit/s. If a call would push any affected vertex past `maxVelocity`, it fails without applying anything. Pinned vertices ignore forces, but `SetVertexVelocity` still moves them until you set their velocity back to zero.

`GetState().pos` is Jolt's body origin, which shifts during simulation, not a fixed model origin. Vertex positions come back in world space and input order. Snapshots are copies; editing one doesn't move the body. For repeated reads, reuse the output tables rather than allocating per vertex each frame:

```lua
local snapshot = {positions = {}, pinned = {}}
local vertex = {}
assert(body:ReadSnapshot(snapshot, false))
assert(body:ReadVertex(1, vertex))
-- Keep reusing snapshot, snapshot.positions, snapshot.pinned and vertex.
-- positions is a flat world-coordinate array: x1, y1, z1, x2, y2, z2, ...
```

Pass `skipSleeping = true` only after a full read of the same body: while it sleeps, the call then succeeds without touching the arrays and sets `snapshot.updated = false`.

### Shell bending and plasticity

`dihedralBend = true` suits panels and shells rather than cloth. `body:SetPlasticity(yield, creep, maxStrain)` makes deformation permanent: past the yield strain, the rest shape gradually follows. The workshop uses neither; it only creates cloth and soft balls.

| Argument | Range | Meaning |
| --- | --- | --- |
| `yield` | 0.001–1 | Edge strain where permanent deformation starts; with dihedral bending, a fraction of π radians |
| `creep` | 0–100 | Relaxation rate per second; 0 turns plasticity off |
| `maxStrain` | 0–0.95 | Largest permanent change of an edge's length relative to the original; with dihedral bending, angles stay within ±`maxStrain * pi` radians of the original |

```lua
assert(body.SetPlasticity, "requires a backend with VJoltSoftBodyTools001")
assert(body:SetPlasticity(0.04, 30, 0.25))
```

A yield of 0.04 is 4% edge strain, or 7.2° with dihedral bending. Results depend on mesh density, mass, collisions and solver settings. `jolt.softbody.SupportsDihedralBend` only means this module knows the option; an older backend can still reject it.

## Interaction and limits

- Soft bodies collide both ways with rigid bodies and the world, but aren't `IPhysicsObject`s, so the physics gun, constraints, Source collision callbacks and saves don't see them. A body from `Create` has no entity, so Source traces miss it as well; the workshop adds an entity for that.
- The player controller's contact queries ignore soft bodies, and vehicle wheels can't rest on them, though a vehicle's chassis still hits them.
- Soft bodies use world gravity times `gravityFactor`; gravity fields and per-object gravity don't affect them.
- Not supported: soft body–soft body collisions, self-collision, tearing, skinning and tetrahedral volume constraints. Soft bodies can't join rigid-body constraints such as gears; a pinned vertex moved with `SetVertexVelocity` serves as a moving attachment instead.
- Backend limits, also returned by `GetLimits()`: 8192 vertices and 16384 triangles per body, 256 bodies and 131,072 vertices in total. The workshop's budgets are much lower; with many bodies, lower the mesh quality or the budgets.

C++ interfaces: `VJoltSoftBody001` in [`public/vjolt/softbody.h`](../public/vjolt/softbody.h) and the optional `VJoltSoftBodyTools001` in [`public/vjolt/softbody_tools.h`](../public/vjolt/softbody_tools.h); the backend side is in `vphysics_jolt/bridge/`.
