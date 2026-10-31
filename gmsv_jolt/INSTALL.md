# Installation

[Home](../README.md)

## 1. Check your game version

Release packages are available for the following Windows versions. Choose the package matching your game branch and architecture:

| Game version | Status |
| --- | --- |
| Windows 64-bit, `x86-64` beta branch | Supported — follow this guide |
| Windows 32-bit, default branch | Supported — `Public-Windows-x86` package |
| Windows 64-bit, default branch | Supported — `Public-Windows-x64` package |
| Linux, macOS | Not supported |

To check or switch the branch, right-click Garry's Mod in your Steam library → **Properties** → **Betas**. For the default branch, select **None**; for the beta package, select **x86-64 - Chromium + 64-bit binaries**. Let Steam finish updating, then launch the architecture matching your package.

The default branch and the `x86-64` branch have different 64-bit binary interfaces, so their DLLs are not interchangeable.

## 2. Download

Download the latest package from [Releases](https://github.com/schweiziske/GMod-Jolt-Physics/releases). It contains three parts:

| Files | Purpose | Required |
| --- | --- | --- |
| `vphysics.dll`, `vphysics_jolt_avx2.dll`, `vphysics_jolt_sse42.dll`, `vphysics_jolt_sse2.dll` | Jolt physics engine | Yes |
| `gmsv_joltgravity_win64.dll`, `gmsv_joltconstraints_win64.dll`, `gmsv_joltsoftbody_win64.dll` | Native modules for the extra features | Optional |
| `jolt` folder | Tools, weapons and menus for the extra features | Optional |

Install all four physics DLLs together: `vphysics.dll` detects which instruction sets your CPU supports and loads the matching AVX2, SSE4.2 or SSE2 build. If you only want the new physics engine, you can skip the other two parts. For black holes, soft bodies and the other [extra features](GAMEPLAY.md), install all three.

## 3. Install

Right-click Garry's Mod in Steam → **Manage** → **Browse local files**. This opens the game folder, which contains `bin` and `garrysmod`.

1. Back up `bin/win64/vphysics.dll`, for example by copying it to `vphysics.dll.bak`.
2. Copy the four physics DLLs into `bin/win64/`, replacing the original `vphysics.dll`.
3. Copy the three `gmsv_jolt*_win64.dll` files into `garrysmod/lua/bin/`. Create the folder if it doesn't exist.
4. Copy the `jolt` folder into `garrysmod/addons/`.
5. Quit the game completely and start it again. Changing maps does not reload DLLs.

For the 32-bit public package, use `bin/` instead of `bin/win64/` and the `gmsv_jolt*_win32.dll` modules instead of `*_win64.dll`. Each ZIP already contains the matching directory layout; copy its contents into the game folder.

The result for either 64-bit package should look like this:

```text
GarrysMod/
├─ bin/win64/
│  ├─ vphysics.dll
│  ├─ vphysics_jolt_avx2.dll
│  ├─ vphysics_jolt_sse42.dll
│  └─ vphysics_jolt_sse2.dll
└─ garrysmod/
   ├─ lua/bin/
   │  ├─ gmsv_joltgravity_win64.dll
   │  ├─ gmsv_joltconstraints_win64.dll
   │  └─ gmsv_joltsoftbody_win64.dll
   └─ addons/jolt/
      ├─ addon.json
      └─ lua/ …
```

## 4. Check that it works

Load any map, open the console (`~`; if nothing happens, enable the developer console under **Options** → **Keyboard** → **Advanced**) and enter:

```text
vjolt_substeps
```

If it prints something like `"vjolt_substeps" = "1"`, Jolt is running. If it prints `Unknown command`, the game is still using the original physics; see [Troubleshooting](#troubleshooting).

With the extra features installed, the **Tools** and **Weapons** tabs of the spawn menu (Q) each have a **Jolt** category.

## Dedicated servers

- Install the `x86-64` branch with SteamCMD: `app_update 4020 -beta x86-64 validate`, and start the server with the 64-bit `srcds_win64.exe`.
- Files go in the same places as on a client: `<server>/bin/win64/`, `<server>/garrysmod/lua/bin/` and `<server>/garrysmod/addons/`.
- Players joining the server don't need to install anything; the addon's Lua files are sent to them automatically.
- Physics uses one worker thread per CPU thread, minus one. To override this, set the `VJOLT_WORKER_THREADS` environment variable before starting the server (0–64; 0 disables worker threads).
- The game's simulation thread is pinned to one core (the fastest one by default) and the physics workers run on the remaining cores, always within the process's CPU affinity. To choose that core yourself, set `VJOLT_SOURCE_CPU` to a logical processor number (0–63).

## Updating and uninstalling

- **Update:** overwrite the DLLs. For the addon, delete the old `garrysmod/addons/jolt` before copying the new one so no stale scripts are left behind.
- **Uninstall:** rename `vphysics.dll.bak` back to `vphysics.dll`, or right-click Garry's Mod in Steam → **Properties** → **Installed Files** → **Verify integrity of game files**. Then delete `bin/win64/vphysics_jolt_*.dll`, `garrysmod/lua/bin/gmsv_jolt*.dll` and `garrysmod/addons/jolt`.
- **After a game update:** Steam may restore the original `vphysics.dll`. If `vjolt_substeps` prints `Unknown command` again, copy the four physics DLLs back in.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| The game crashes at startup | Wrong branch, or physics DLLs missing. Make sure you're on `x86-64` and all four physics DLLs are in `bin/win64/`. |
| `vjolt_substeps` prints `Unknown command` | The DLLs are in the wrong folder, or the game was started in 32-bit mode. |
| No Jolt category in the spawn menu | The addon is in the wrong place. The path must be `garrysmod/addons/jolt/addon.json`, without an extra nested folder. |
| Jolt tools say a feature is unavailable | The matching `gmsv_jolt*_win64.dll` is missing from `garrysmod/lua/bin/`, or Jolt physics isn't running. |

Still stuck? [Open an issue](https://github.com/schweiziske/GMod-Jolt-Physics/issues) with your game branch, CPU model and any console errors.
