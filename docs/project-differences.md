# Differences from Upstream

[Home](../README.md)

This page explains where GMod Jolt Physics comes from and what it changes on top of RaphaelIT7's VPhysics-Jolt.

## Origin

The GMod Jolt Physics code derives from [RaphaelIT7/VPhysics-Jolt](https://github.com/RaphaelIT7/VPhysics-Jolt), itself a fork of [Volt (VPhysics-Jolt)](https://github.com/misyltoad/VPhysics-Jolt).

This repository is not a GitHub fork and does not contain the upstream commit history: development touched a wide range of code with a messy history, so the history was restarted for the public release. The commit records of Volt and RaphaelIT7's contributors remain in those upstream repositories. The upstream license ships with the source in [LICENSES](../LICENSES/Volt-MIT.txt).

The custom Jolt is based on Jorrit Rouwe's [JoltPhysics](https://github.com/jrouwe/JoltPhysics). It retains the upstream commit history and contributor records, with this project's changes committed on top.

Volt and RaphaelIT7's version already provide the VPhysics interface adaptation, basic rigid bodies, collision queries, constraints, player and shadow controllers, the vehicle interface, triggers and multithreaded simulation. This project builds on that foundation, with three main areas of work:

- Keeping the existing physics interfaces true to Source/IVP parameter meanings and execution timing.
- Making compatibility constraints and friction work together with Jolt's contact solver.
- Offering extra physics features through public interfaces, Lua modules and Sandbox tools.

## What is compared

| Subject | Version |
| --- | --- |
| This project | [`b61edbe`](https://github.com/schweiziske/GMod-Jolt-Physics/tree/b61edbe2d8849c9312cf63f4b9d93ad055fc9159) |
| Custom Jolt submodule | [`bca5609`](https://github.com/schweiziske/JoltPhysics-GMod/tree/bca5609eaebf54e48340229ab22efafec0af7b1c) |
| Baseline (RaphaelIT7's `fixed` branch) | [`cfabd9b`](https://github.com/RaphaelIT7/VPhysics-Jolt/tree/cfabd9b3dcff90227ff2f342e8970fc89f2a638e) |

The sections below describe the differences between these versions only, not later upstream versions.

## Source/GMod physics compatibility

| Area | Baseline | This project | Effect in game |
| --- | --- | --- | --- |
| Simulation time | Runs controllers in the external call, then calls the Jolt update and collision substeps | Accumulates time and runs whole physics steps, each running controllers, committing velocities and solving; `SetSimulationTimestep` sets the actual step | Game input, controllers and physics response keep the same rhythm |
| Controller order | Original update flow | Schedules by priority in a stable order, and handles attaching, detaching and deleting during callbacks | Reduces order differences when several controllers act together |
| Mass and inertia | Inertia mainly computed from the Jolt shape | Prefers the center of mass and unit-mass inertia from the Source collision model, and handles inertia multipliers and rotational inertia limits | Rotation, off-center impulses and constraint response are based on the original data |
| Forces and velocity | Maps interface calls directly to Jolt | Holds pending linear and local angular velocity changes, separating reads, immediate sets, accumulation and commits | Keeps Source's stacking semantics for repeated forces and controller output |
| Material elasticity | Product of material elasticities used directly as restitution | Converts IVP's retained-energy ratio into a velocity restitution coefficient, and resolves the material of the contact surface | Throw and landing bounce parameters mean the same as in the original |
| Friction snapshots | Builds contact information from nearby geometry queries | Reads the published contact and solver state, storing normal impulse, pressure, contact points and related history | The game layer sees the contacts and load that actually occurred |
| Springs | Soft distance constraint | Treats natural length, stiffness, axial and relative damping and extend-only as an IVP actuator, sampling once before applying force | Keeps the force meaning of spring parameters and reduces the effect of iteration order |
| Player physics | Creates an extra Character and syncs it to the public physics object | Drives the public dynamic body directly, handling moving platforms, relative velocity, pushing and contact limits | The object the game queries is the one that actually takes forces |
| Shadow control | Mainly kinematic moves or direct transform sets | The dynamic path drives by error, arrival time, speed limit and damping, sharing one Shadow calculation | Improves load transfer between controlled objects and those around them |
| Collision callbacks | Sends Pre/Post events after the update | Coordinates the order of PreCollision, velocity changes from callbacks, the actual response and PostCollision; structural changes wait for a safe point | Changes made in a callback affect the matching collision response |
| Physics traces | Implemented on Jolt queries | Keeps the Source query geometry and computes hit fraction, normal and overlap state through a compatible sweep/simplex path | Supports step detection, leaving a contact surface, tool hits and similar game logic |

Code: [environment and stepping](../vphysics_jolt/vjolt_environment.cpp), [objects and pending velocity](../vphysics_jolt/vjolt_object.cpp), [contact materials](../vphysics_jolt/vjolt_listener_contact.h), [friction snapshots](../vphysics_jolt/vjolt_friction.cpp), [player controller](../vphysics_jolt/vjolt_controller_player.cpp), [Shadow control](../vphysics_jolt/vjolt_shadow_control.h), [traces](../vphysics_jolt/vjolt_collide_trace.cpp).

### Buoyancy, water drag and rotation in fluids

The baseline handles buoyancy and drag together through Jolt's `ApplyBuoyancyImpulse`. This project keeps Jolt's submerged volume query and replaces the other forces with Source-style calculations, which affects how objects enter water, float, tumble, get pushed by currents and slow down:

- Pressure, friction and torque on partly submerged surfaces come from surface triangles; spheres use a separate projected-area calculation.
- Buoyancy comes from submerged volume, fluid density, the object's buoyancy multiplier and gravity; the center of buoyancy's offset from the center of mass produces torque.
- The response depends on the surface's velocity relative to the flow, the surfaces facing the flow and angular velocity. An object's motion in the previous step has a limited effect on the surrounding fluid, which is diluted as the submerged area grows.
- Fluid velocity is converted using the Source interface's direction rules, not scaled as a position or distance; forces use the matching physics step.

Code: [fluid controller](../vphysics_jolt/vjolt_controller_fluid.cpp).

### Stock vehicle interface

This is Source's existing vehicle interface, unrelated to the extra vehicle gameplay modules. The baseline already implements wheeled vehicles and airboats; this project adds parameters and runtime behavior:

- Drivetrain settings are converted using per-axle torque split, wheels per axle and Jolt's differential constraints.
- `SetWheelFriction` sets an absolute multiplier against the friction curve from construction, so repeated calls don't compound.
- Tire contact materials are resolved from the contact object and sub-shape.
- Airboats separate the configured spring length from the runtime ray length, and gain pontoon parameters and debug queries.
- Vehicle restore saves the necessary configuration and state and rebuilds contacts, rather than reusing BodyIDs and warm-start impulses from the old physics world.

Code: [vehicle controller](../vphysics_jolt/vjolt_controller_vehicle.cpp), [vehicle constraint](../vphysics_jolt/vjolt_vehicle_constraint.h).

### Freezing, mass, inertia and sphere radius

Changing a physics object at runtime also has to follow Source semantics:

- Freezing and unfreezing follow IVP's pinned behavior for velocity and pending changes, and keep the normal inertia for when motion resumes.
- `SetMass` scales existing inertia by the mass ratio, and correctly updates mass, inverse mass and inverse inertia while frozen.
- `SetInertia` converts between Source object axes and Jolt's inertia representation, so principal-axis reordering doesn't change what the interface means.
- Changing a sphere's radius replaces the immutable Jolt shape while keeping mass, inertia and velocity, and handles radius values Jolt can't accept.
- `GetEnergy` sums translational and rotational kinetic energy, accounts for principal inertia coordinates and locked axes, and converts the result back to Source units.

Code: [object implementation](../vphysics_jolt/vjolt_object.cpp).

### Contact response, load bearing and player collisions

The friction snapshots in the table above cover what the game reads. The solver side has further adaptations:

- Native or compatibility contact paths are chosen by contact object and constraint state, separating first impact from sustained load.
- Compatibility friction maintains tangential slip history, tangent-plane projection, coupled effective mass and pressure limits.
- For eligible free-body load-bearing contacts, the effective mass ratio is limited during solving, so heavy objects rest more stably on light ones; real object masses don't change.
- Player contacts separate overhead, side, ground and existing-load responses, keeping the tangential motion and surface separation that should be allowed.
- Sleeping contact retention, invalidation after object deletion, contact anchors and sequenced BodyIDs are handled so the game never reads dead objects.

These stabilization strategies have conditions; they don't guarantee parity with the original at every mass ratio or geometry.

Code: [contact response](../vphysics_jolt/vjolt_listener_contact.h), [contact cache](../vphysics_jolt/vjolt_friction_contact_cache.h), [player contact planes](../vphysics_jolt/vjolt_player_contact_planes.h).

### Collision models, triangle materials and geometry interfaces

- IVP compact surfaces keep Source's center of mass, inertia, convex structure, vertices, triangles, materials and each ledge's game data.
- The query model returns triangles, vertices and gameData per convex, and implements reading and writing triangle material indices using IVP's 7-bit rule.
- Polyhedron vertices, edges and faces are built from closed triangles, with checks for open or malformed topology.
- Collision geometry serialization, deserialization and scaling maintain the matching metadata; scaled models rebuild geometry and separate original model properties from runtime overrides.
- The `$MATERIAL_INDEX_SHADOW` name check is fixed, and material and sound alias casing follows Source rules.

Code: [collision models](../vphysics_jolt/vjolt_collide.cpp), [query model](../vphysics_jolt/vjolt_querymodel.cpp), [surface properties](../vphysics_jolt/vjolt_surfaceprops.cpp).

### Physics KeyValues, vehicle scripts and ragdoll collision rules

- `MassCenterOverride` only takes effect once a complete vector has been parsed.
- Vehicle wheel parameters accept the correct spelling `Inertia` and keep the historical `Intertia`.
- Nested vehicle parameters, defaults, unknown fields and invalid surface indices are handled; Source's `RollingDrag` is recognized but changes nothing.
- Ragdoll collision pairs are parsed one at a time with number, overflow and format checks, so malformed input can't reuse the previous pair.
- Parsing uses Source-compatible tokens, rejects input that can't be fully parsed, and frees the parse tree.

Code: [physics parameter parsing](../vphysics_jolt/vjolt_parse.cpp), [KeyValues parser](../vphysics_jolt/vjolt_keyvalues_parser.h), [schema](../vphysics_jolt/vjolt_keyvalues_schema.cpp).

### Save and restore

Upstream already has save and restore interfaces. This project extends object relationships and state management:

- Save and restore sessions manage object IDs and the mapping from old pointers to new objects, and check for duplicate IDs and call order.
- State is saved in payloads with format information, which handle read failures and older payloads.
- Links between rigid bodies, shadow controllers, constraint groups and vehicles are restored; a constraint group with no members keeps its active state.
- Saved active and sleep states are applied only after all related types are restored, using BodyIDs to identify objects that are still valid.

This implements the Source save flow. It provides no time-rewind feature and doesn't guarantee full save compatibility with every addon.

Code: [Save/Restore flow in the environment](../vphysics_jolt/vjolt_environment.cpp), [save/restore ID table](../vphysics_jolt/vjolt_saverestore_ids.h), [object state](../vphysics_jolt/vjolt_object.cpp).

### Collision pair management and object changes during callbacks

The object pair hash moved to a separate storage implementation that tracks membership, pair counts and adjacency lists from actual links, so removing one pair doesn't lose the object's other links.

Freezing, shape and mass changes, and object creation or deletion inside collision callbacks wait for a safe point, and event listeners and object lifetimes handle callback reentry and invalidation. No-collide, tool actions, deletion and map cleanup all rely on this behavior.

Code: [object pair storage](../vphysics_jolt/vjolt_objectpairhash_storage.h), [object pair interface](../vphysics_jolt/vjolt_objectpairhash.cpp), [environment lifecycle](../vphysics_jolt/vjolt_environment_lifecycle.cpp).

### Inherited from upstream

Not all Source-related code is new in this project. Compared file by file with the baseline, the air drag controller `vjolt_controller_drag.cpp` is unchanged, and the main calculation of linear and angular drag from projected area already exists in the baseline. Airboats, material inheritance and the basic save interfaces also come from upstream. This page lists only the specific changes in those areas; the features as a whole remain upstream's.

## Constraints and the custom Jolt solver

Fixed, length and ragdoll constraints are solved with Source/Havana's own equations; hinge, ball-socket, slider and similar constraints handle compatibility parameters, motors, angles and breaking rules. The adaptation distinguishes object origin, center of mass and coordinate conversions, as well as what parameters like strength and tau mean for each constraint type.

Constraint groups arrange members and solver work by actual physics island, and handle connectivity, wake propagation and job dependencies. So that compatibility behavior can coexist with native contacts, the custom core reorders the compatibility friction, Havana constraint, contact warm-start and contact iteration phases in the affected islands.

Some of these fixes address issues that only appeared after compatibility solving was introduced, and don't indicate defects in the baseline.

Code: [constraint adaptation](../vphysics_jolt/vjolt_constraints.cpp), [Havana implementation](../vphysics_jolt/vjolt_havana_constraints.cpp), [custom Jolt](https://github.com/schweiziske/JoltPhysics-GMod).

## New features

These are under **Tools → Jolt** and **Weapons → Jolt** in the Q menu, with English and Simplified Chinese interfaces. For full instructions, see the [gameplay guide](../gmsv_jolt/GAMEPLAY.md).

### Soft Body Workshop and plasticity interface

- Create cloth and inflatable soft balls, with adjustable size, mesh quality, total mass, softness and inflation.
- Drag, pin and release vertices; supports collision with the map and rigid bodies, damage impulses and optional player collision.
- Simulates on the server, filters network sync by distance and PVS, and interpolates and renders the mesh on the client.
- The native interface supports dihedral bending, yield thresholds, plastic relaxation and permanent deformation for custom meshes.

The plasticity API doesn't include model import, vehicle shell splitting or a full vehicle damage system. Soft body–soft body collisions, self-collision and tearing aren't supported.

Docs: [soft bodies](../gmsv_joltsoftbody/README.md).

### Black holes and gravity control

- Black hole tool and grenade: pull objects in range together, then fling them away.
- World gravity, per-object gravity factor and direction, and spherical and box-shaped gravity fields.
- Directional, attract and repel modes, with direction editing, visualization, permissions and count limits.

Gravity fields act on dynamic bodies and supported vehicle controllers, not on player movement or NPC AI.

Docs: [black holes](../gmsv_jolt/GAMEPLAY.md#black-holes), [gravity system](../gmsv_joltgravity/README.md).

### Gears and racks

Provides gear transmission, rotation-to-linear conversion, gear ratios and motor control. The tools create their own support constraints and integrate with Sandbox undo, cleanup and count limits. Axes are fixed in the world, which suits stationary machinery; moving bases and vehicle drivetrains aren't supported.

Docs: [mechanical constraints](../gmsv_joltconstraints/README.md).

### 3D path tracks

Objects follow a 3D curve built from editable nodes, driven by a backend path constraint. Supports open back-and-forth runs, closed loops, constant speed, positioning, free sliding, pause, reverse and presets; the tool also offers node editing and a choice of moving reference frame.

Docs: [path tracks](../gmsv_joltconstraints/PATH.md).

## SDK for addons and native modules

Gravity, mechanisms, paths, soft bodies and statistics are organized into versioned public C++ interfaces, with Lua bindings and a feature export example.

- **Shared physics world:** the Bridge operates on the running backend environment, so feature modules don't need their own rigid-body world.
- **Public contracts:** interfaces state units, threading and lifetimes, and don't expose Jolt internals across DLLs.
- **Resource management:** interface version checks, generation- or sequence-tagged handles, and safe destruction during callbacks.
- **On-demand Lua loading:** use the optional features through `require("jolt")`, `IsAvailable` and `Load`.
- **Development example:** shows how to take a new capability from the public headers through the backend Bridge to Lua.

Docs: [Physics API](physics-api.md), [public SDK](../public/vjolt/README.md), [Bridge](../vphysics_jolt/bridge/README.md), [export example](../examples/physics/export/README.md).

## Loading, distribution and game integration

A CPU variant wrapper selects an AVX2, SSE4.2 or SSE2 backend by CPU capability. The project ships matching builds for 32-bit and 64-bit on the Windows default branch, and 64-bit on the `x86-64` beta branch. The default and `x86-64` branches have different 64-bit ABIs, so each needs its own package.

Integration work also covers lifetime handling for object and environment destruction, callback reentry, deferred changes and cross-environment transfers, plus permissions, undo, cleanup, networking and resource limits for Sandbox tools.

Entry points: [CPU wrapper](../vphysics_wrapper/vphysics_wrapper.cpp), [installation guide](../gmsv_jolt/INSTALL.md), [build guide](build.md), [lifecycle implementation](../vphysics_jolt/vjolt_environment_lifecycle.cpp).

## Credits

Thanks to Joshua Ashton, Josh Dowell, RaphaelIT7, Jorrit Rouwe and all contributors to the upstream projects; this project wouldn't exist without their work. The GMod-specific compatibility, solver integration and new features were developed on top of it by schweiziske.
