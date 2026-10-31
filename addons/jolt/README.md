# Jolt Addon

[Home](../../README.md)

Sandbox tools and weapons for GMod Jolt Physics: soft bodies, black holes, gravity fields, gears/racks and rails.

If you are already subscribed to the Jolt addon on the Steam Workshop, you do not need to install the separate Lua addon package from Releases. Use either the Workshop addon or a manual copy, not both. The matching physics and native module DLLs must still be installed separately on the server or single-player host.

| Spawn menu (Q) | Contents |
| --- | --- |
| Tools → Jolt | Black Hole, Gears / Rack, Path Track - PathConstraint, Soft Body Workshop |
| Weapons → Jolt | Jolt Gravity Controller, Jolt Black Hole Grenade |
| Options → Jolt | Gravity Fields, Black Hole, Soft body |

The server needs the Jolt physics backend and the three native modules `gmsv_joltgravity`, `gmsv_joltconstraints` and `gmsv_joltsoftbody`. Players who join a server don't need to install anything: the server sends them the addon's Lua files.

- Installation: [installation guide](../../gmsv_jolt/INSTALL.md)
- Controls and settings: [extra features](../../gmsv_jolt/GAMEPLAY.md)
- Translations: [localization](docs/LOCALIZATION.md)
- Soft-body rendering and networking internals: [soft-body client](docs/SOFTBODY_CLIENT.md)
