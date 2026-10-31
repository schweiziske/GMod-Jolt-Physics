# GMod Jolt Physics

[简体中文](README.zh-CN.md)

GMod Jolt Physics replaces Garry's Mod's IVP/Havok-based physics engine (VPhysics) with [Jolt Physics](https://github.com/jrouwe/JoltPhysics). The goal is to keep the physics behavior Source players know — props, ragdolls, vehicles, constraints and the physics gun work as before — while the simulation runs on Jolt's multithreaded solver.

It also adds five new Sandbox features: **soft bodies, black holes, gravity fields, gears/racks and rails**.

Developed by [schweiziske](https://github.com/schweiziske), building on [Volt (VPhysics-Jolt)](https://github.com/misyltoad/VPhysics-Jolt). See [Credits](#credits).

## Quick start

1. Download the latest version from [Releases](https://github.com/schweiziske/GMod-Jolt-Physics/releases).
2. Copy the files into your game folder as described in the [installation guide](gmsv_jolt/INSTALL.md), then restart the game.

| Game version | Status |
| --- | --- |
| Windows 64-bit, `x86-64` beta branch | Supported |
| Windows 32-bit | Supported |
| Windows 64-bit, default branch | Supported |
| Linux, macOS | Not supported yet |

The default (public) branch and the `x86-64` beta branch both have a 64-bit build, but they use different binary interfaces (ABIs), so **their 64-bit DLLs are not interchangeable**. Choose the package matching your game:

| Package | Game version |
| --- | --- |
| `GMod-Jolt-Physics-Public-Windows-x86.zip` | Default (public) branch, 32-bit |
| `GMod-Jolt-Physics-Public-Windows-x64.zip` | Default (public) branch, 64-bit |
| `GMod-Jolt-Physics-Beta-x86-64-Branch-Windows-x64.zip` | Steam `x86-64` beta branch, 64-bit |

## Features

How GMod Jolt Physics compares with the original VPhysics and with Volt.

| Feature | VPhysics | Volt | GMod Jolt Physics |
| :--- | :---: | :---: | :---: |
| Constraints (except pulleys) | ✔️ | ✔️ | ✔️ |
| Pulleys | ✔️ | ✔️ | ✔️ |
| Breakable constraints | ✔️ | ❌ | ✔️ |
| Motors (motion controllers) | ✔️ | ✔️ | ✔️ |
| Motors (constraints) | ✔️ | ✔️ | ✔️ |
| Ragdolls | ✔️ | ✔️ | ✔️ |
| Triggers | ✔️ | ✔️ | ✔️ |
| Object touch callbacks | ✔️ | ✔️ | ✔️ |
| Prop damage and breaking | ✔️ | ✔️ | ✔️ |
| Fluid events | ✔️ | ✔️ | ✔️ |
| Prop splash effects | ✔️ | ✔️ | ✔️ |
| Wheeled vehicles | ✔️ | ✔️ | ✔️ |
| Raycast vehicles (e.g. the airboat) | ✔️ | ❌ | ✔️ |
| NPC and door shadow controllers | ✔️ | ✔️ | ✔️ |
| Save/restore | ✔️ | ✔️ | ✔️ |
| Per-object collision callbacks (e.g. no-collide) | ✔️ | ✔️ | ✔️ |
| Multithreaded simulation | ❌ | ✔️ | ✔️ |
| Player physics controller | ✔️ | ✔️ | ✔️ |

### New in GMod Jolt Physics

| Feature | What it does |
| --- | --- |
| [Soft bodies](gmsv_joltsoftbody/README.md) | Cloth and inflatable soft balls; drag and pin their vertices |
| [Black holes](gmsv_jolt/GAMEPLAY.md#black-holes) | Pull nearby objects in, then blast them away |
| [Gravity fields](gmsv_joltgravity/README.md) | Change gravity for the whole world, a single prop or an area |
| [Gears and racks](gmsv_joltconstraints/README.md) | Link props by a gear ratio, or turn rotation into sliding |
| [Rails](gmsv_joltconstraints/PATH.md) | Move props along curved 3D tracks |

They appear in the spawn menu under **Tools → Jolt** and **Weapons → Jolt**; see [extra features](gmsv_jolt/GAMEPLAY.md) for controls. Addon authors can use them from Lua through `require("jolt")`.

## For developers

The main repository and the customized [JoltPhysics-GMod](https://github.com/schweiziske/JoltPhysics-GMod) submodule are open source under the MIT license. Clone both with:

```sh
git clone --recurse-submodules https://github.com/schweiziske/GMod-Jolt-Physics.git
```

- [Build guide](docs/build.md): the Lua modules need only `garrysmod_common`; the physics backend also needs a Source/GMod SDK.
- [Physics API](docs/physics-api.md): use the extra features from Lua or C++.
- [Feature export example](examples/physics/export/README.md): expose new Jolt functionality, from the backend all the way to Lua.
- [Backend code map](vphysics_jolt/README.md): where each part of the physics backend lives, and the rules for changing it.

## Reporting bugs

GMod Jolt Physics still has rough edges. Please report problems in the [issue tracker](https://github.com/schweiziske/GMod-Jolt-Physics/issues) and include:

- your game branch, and whether the game runs in 32- or 64-bit;
- your CPU model;
- the map and the addons you have installed;
- steps to reproduce;
- console errors or a crash dump, if any.

## Credits

- **GMod Jolt Physics:** [schweiziske](https://github.com/schweiziske) — GMod compatibility layer, solver integration and the new features.
- **Volt / VPhysics-Jolt:** [Joshua Ashton](https://github.com/Joshua-Ashton) (🐸✨), [Josh Dowell (Slartibarty)](https://github.com/Slartibarty), [RaphaelIT7](https://github.com/RaphaelIT7) and the other upstream contributors.
- **Jolt Physics:** Jorrit Rouwe and the Jolt Physics contributors.

## Demos

These videos come from the original Volt project.

<table>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=gPDQkmfQCsc"><img src="https://img.youtube.com/vi/gPDQkmfQCsc/mqdefault.jpg" alt="Lots of Melons + Dumpster"></a><br>Lots of Melons + Dumpster</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=tVmQTmbSJM0"><img src="https://img.youtube.com/vi/tVmQTmbSJM0/mqdefault.jpg" alt="Physically Simulated Chain"></a><br>Physically Simulated Chain</td>
  </tr>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=tYfiTyRtmz8"><img src="https://img.youtube.com/vi/tYfiTyRtmz8/mqdefault.jpg" alt="Lots of Balls Test"></a><br>Lots of Balls Test</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=5_QbbXbIrg8"><img src="https://img.youtube.com/vi/5_QbbXbIrg8/mqdefault.jpg" alt="Wheels + Weld Car Dupe Test"></a><br>Wheels + Weld Car Dupe Test</td>
  </tr>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=SdEj7HTuJmU"><img src="https://img.youtube.com/vi/SdEj7HTuJmU/mqdefault.jpg" alt="Door + NPC (Physics Shadowed Objects) Test"></a><br>Door + NPC (Physics Shadowed Objects) Test</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=CLVnSwg33Dk"><img src="https://img.youtube.com/vi/CLVnSwg33Dk/mqdefault.jpg" alt="Lots of Cubes + Ragdolls + Funnel"></a><br>Lots of Cubes + Ragdolls + Funnel</td>
  </tr>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=GzW_4bufwEk"><img src="https://img.youtube.com/vi/GzW_4bufwEk/mqdefault.jpg" alt="Slow Mo Cubes"></a><br>Slow Mo Cubes</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=10vvRJVHGQc"><img src="https://img.youtube.com/vi/10vvRJVHGQc/mqdefault.jpg" alt="Propane in Dumpster"></a><br>Propane in Dumpster</td>
  </tr>
</table>
