# Jolt Gears and Racks

[Home](../README.md)

Couple two props through a gear ratio, or turn a pinion's rotation into a rack's sliding motion. Each mechanism includes its own hinges (and the rack's slider), so no separate Axis or Slider constraint is needed. For setup, see the [installation guide](../gmsv_jolt/INSTALL.md); for rails, see the [rail guide](PATH.md).

## Gears / Rack tool

Open **Q → Tools → Jolt → Gears / Rack**.

1. Freeze and position two props, then set **Assembly type** to **Dual gears** or **Rack and pinion**.
2. Left-click an end face of each prop. The face normal sets that prop's rotation axis, or the rack's sliding direction. Axes pass through each prop's center of mass and are fixed in the world.
3. Unfreeze the props to run the mechanism.

| Input | Action |
| --- | --- |
| Right click | Apply the panel's motor settings to your most recent mechanism |
| R | Cancel the selection |
| Z | Undo the mechanism; the props stay |

The tool doesn't change collisions; if the models overlap, use Sandbox's No Collide tool. Each mechanism counts toward `sbox_maxjolt_mechanisms` (default 32, range 0–256) and the standard `constraints` limit. The duplicator doesn't copy mechanisms.

## Lua API

Run on the server. `firstProp` and `secondProp` are existing physics props.

```lua
require("jolt")
local available, reason = jolt.IsAvailable("constraints")
assert(available, reason)

local gears, detail = jolt.constraints.CreateGear(
	firstProp:GetPhysicsObject(),
	secondProp:GetPhysicsObject(),
	{
		axis1 = Vector(0, 0, 1),
		axis2 = Vector(0, 0, 1),
		ratio = 2,
	}
)
assert(gears, detail)
assert(gears:SetMotor(180, 1000000))
firstProp.JoltMechanism = gears -- Keep a reference: garbage collection destroys the mechanism.

local state = assert(gears:GetState())
print(state.angularVelocity1, state.angularVelocity2) -- About 180 and -90 deg/s without load.

-- When finished:
-- gears:Destroy()
-- firstProp.JoltMechanism = nil
```

For a rack and pinion, the first body is the pinion and the second slides along `axis2`:

```lua
local rack, reason = jolt.constraints.CreateRackAndPinion(
	pinionProp:GetPhysicsObject(),
	rackProp:GetPhysicsObject(),
	{
		axis1 = Vector(0, 0, 1),
		axis2 = Vector(1, 0, 0),
		travelPerRevolution = 128,
		minPosition = -256,
		maxPosition = 256,
	}
)
assert(rack, reason)
assert(rack:SetMotor(360, 1000000))
pinionProp.JoltMechanism = rack
```

At one revolution per second the rack moves 128 Source units/s, and it stops at either limit.

`CreateGear(phys1, phys2, settings)` and `CreateRackAndPinion(phys1, phys2, settings)` return a mechanism, or `nil, reason`. `settings` must be a table; omitted keys use the defaults below. Wrong argument types and non-finite numbers in `settings` raise a Lua error.

| Setting | Default | Meaning |
| --- | --- | --- |
| `axis1` | `Vector(0, 0, 1)` | World rotation axis of the first body |
| `axis2` | Gear: `Vector(0, 0, 1)`; rack: `Vector(1, 0, 0)` | World rotation axis of the second gear, or the rack's sliding direction |
| `anchor1` / `anchor2` | Center of mass | World-space attachment points |
| `ratio` | 1 | Gears only: `angle1 + ratio * angle2 = 0`. Absolute value 0.001–1000. |
| `travelPerRevolution` | 128 | Racks only: signed distance the rack moves per positive pinion turn, in Source units. Absolute value at least 0.01; a negative value reverses the direction. |
| `minPosition` / `maxPosition` | −128 / 128 | Racks only: travel limits relative to the starting position. The range must include 0, and min must be less than max. |

Axes are normalized and must not be zero. Vector components and distances must be finite and within ±1,048,576.

For meshing gears, `ratio = teeth2 / teeth1`, so `speed2 = -speed1 / ratio`. With a positive ratio the gears turn in opposite directions about their axes; with a negative ratio they turn the same way. Fractional ratios, positive or negative, keep their phase across full turns.

| Method | Behavior |
| --- | --- |
| `IsValid()` | Whether the mechanism still exists |
| `Destroy()` | Removes all of its constraints together. Returns `true`, or `false` if it was already gone. |
| `SetMotor(speed, maxTorque)` | Drives the first body. Returns `true` or `nil, reason`. |
| `GetState()` | Returns a state table or `nil, reason` |

Motor speed is in degrees/s, within ±3600. Torque is in kg·Source-unit²/s², from 0 to 1,000,000,000. Speed 0 with a nonzero torque brakes; torque 0 turns the motor off but keeps the coupling.

The state table has `kind` (`"gear"` or `"rack_and_pinion"`), `angle1`, `angle2`, `position`, `angularVelocity1`, `angularVelocity2`, `linearVelocity`, `motorSpeed` and `maxTorque`. Angles are in degrees, wrapped to ±180, and angular velocities in degrees/s. `position` and `linearVelocity` are in Source units and Source units/s. `angle2` and `angularVelocity2` are only set for gears; `position` and `linearVelocity` only for racks.

## Limits

- The two bodies must be different and in the same physics environment. Static, trigger, fluid, game-controlled and shadow-controlled objects aren't supported.
- Frozen props can be attached. Creating a mechanism doesn't unfreeze them; it starts working once they are unfrozen.
- Axes are fixed in the world, so mechanisms suit stationary machinery. Moving bases and vehicle drivetrains aren't supported.
- Create mechanisms, set motors and read state outside the physics step. `Destroy()` also works from physics callbacks; the constraints are removed at the next safe point.
- Removing either body, moving it to another environment or destroying the environment removes the whole mechanism. So does replacing a body's collision shape or making it unsupported, at the next physics step.
- The backend holds up to 4096 mechanisms.

C++ interface: `VJoltConstraints001` in [`public/vjolt/constraints.h`](../public/vjolt/constraints.h). It depends on the gear fixes in this repository's Jolt fork.
