# Extra Features

[Home](../README.md)

The `jolt` addon adds five Jolt-powered features to Sandbox. They need the optional modules and the addon from the [installation guide](INSTALL.md).

| Feature | Where to find it (Q menu) | What it does |
| --- | --- | --- |
| [Soft bodies](#soft-bodies) | Tools → Jolt → Soft Body Workshop | Cloth and inflatable soft balls whose vertices you can drag and pin |
| [Black holes](#black-holes) | Tools → Jolt → Black Hole; Weapons → Jolt → Jolt Black Hole Grenade | Pull nearby objects in, then fling them away |
| [Gravity fields](#gravity-fields) | Weapons → Jolt → Jolt Gravity Controller | Change gravity for the whole world, a single prop or an area |
| [Gears and racks](#gears-and-racks) | Tools → Jolt → Gears / Rack | Link two props by a gear ratio, or turn rotation into sliding |
| [Rails](#rails) | Tools → Jolt → Path Track - PathConstraint | Move a prop along a curved 3D track |

Menus follow the game language (`gmod_language`): English and Simplified Chinese are available, and other languages fall back to English. Default settings for gravity fields, black holes and soft bodies are under the **Options** tab of the Q menu → **Jolt**.

H only works while a tool or weapon that uses it is in your hands, and is ignored while a menu, the chat or a text field is open.

## Soft bodies

In **Create soft body** mode, pick cloth or a soft ball and set its size, mass, softness and mesh quality, then:

| Input | Action |
| --- | --- |
| Left click | Place the soft body on the surface you're aiming at |
| Z | Undo |

In **Edit vertices** mode:

| Input | Action |
| --- | --- |
| Hold left click | Drag a vertex; releasing restores its original pinned state |
| Right click | Pin or unpin the selected vertex (pins are fixed in the world) |
| R | Finish dragging |

The regular physics gun can't grab soft bodies; edit them with the workshop. See the [soft-body guide](../gmsv_joltsoftbody/README.md) for details.

## Black holes

A black hole pulls in nearby objects for a while, then blasts them away and disappears. It only affects movable physics objects: it doesn't move players and doesn't deal damage.

| Where | Controls |
| --- | --- |
| Tools → Jolt → Black Hole | Left click places one on the surface you're aiming at; right click removes your most recent one |
| Weapons → Jolt → Jolt Black Hole Grenade | Left click throws; R sets the radius and duration |

Settings live under Options → Jolt → **Black Hole**:

| ConVar | Default | Range | Meaning |
| --- | --- | --- | --- |
| `jolt_blackhole_radius` | 450 | 64–32767 (non-admins: up to 1024) | Effect radius |
| `jolt_blackhole_pull` | 2200 | 100–10000 (non-admins: up to 2000) | Pull acceleration |
| `jolt_blackhole_burst` | 6500 | 100–20000 | Burst acceleration |
| `jolt_blackhole_duration` | 2.5 | 0.1–300 | Pull duration in seconds |

Servers limit how many black holes each player can have at once with `sbox_maxjolt_blackholes` (default 3).

## Gravity fields

| Input | Action |
| --- | --- |
| R | Switch mode: world gravity, prop gravity or area field |
| Hold E and move the mouse | Aim the gravity arrow; release E to keep it |
| Left click | Apply gravity in that direction (prop mode: to the prop under your crosshair; area mode: creates a field there) |
| Right click | Restore the original gravity; in area mode, remove your most recent field |
| H | Switch area fields between sphere and box |
| Z | Undo |

Non-admins can only use area mode. Fields affect physics objects and vehicles, not player movement or NPC behavior. See the [gravity-field guide](../gmsv_joltgravity/README.md).

## Gears and racks

1. Freeze and position two props, then set **Assembly type** to **Dual gears** or **Rack and pinion**.
2. Left-click an end face of each prop. The normal of the face you click sets the rotation axis, or the rack's sliding direction.
3. Unfreeze the props to start the mechanism.

| Input | Action |
| --- | --- |
| Right click | Apply the panel's motor settings to your most recent mechanism |
| R | Cancel the selection |
| Z | Undo the mechanism, keeping the props |

If the models overlap, use Sandbox's No Collide tool on them first. See the [gears and racks guide](../gmsv_joltconstraints/README.md).

## Rails

1. Freeze the prop and orient it the way it should face.
2. Left-click to place track nodes: at least 2, or 3 for a closed loop.
3. Right-click the prop to attach it, then unfreeze it. Open tracks go back and forth by default; closed tracks loop.

| Input | Action |
| --- | --- |
| Left click | Add a node |
| E + left click | Select a node; click again to move it to your crosshair |
| Shift + left click | Insert a node after the selected one |
| R | Delete the selected node, or the last one if none is selected |
| H | Start a new track |
| Right-click a prop | Attach it to the track |
| E + right-click a prop | Load its track for editing |
| Shift + right click | On a prop: use it as a moving reference frame. On the map: fix the track to the world again |
| Z | Undo: removes the track constraint and keeps the prop |

The panel can also pause, reverse, change speed and save presets. See the [rail guide](../gmsv_joltconstraints/PATH.md).
