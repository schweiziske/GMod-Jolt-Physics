# Jolt Lua Addon - Manual Installation

If you are already subscribed to the Jolt addon on the Steam Workshop, you do not need to install this Lua addon package. For a dedicated server, this also applies when the addon is installed through its Workshop collection.

Use either the Workshop addon or this manual copy. If switching to the Workshop version, remove the old manually installed `garrysmod/addons/jolt` folder to avoid duplicate scripts.

## Install without the Workshop

1. Download `GMod-Jolt-Physics-Lua-Addon.zip` from the same GitHub Release as your DLL package.
2. Extract the ZIP into your Garry's Mod or dedicated server root folder (the folder containing `bin` and `garrysmod`). The addon must end up at `garrysmod/addons/jolt/addon.json`.
3. Keep the whole addon together, including `addon.json`, `lua/` and `resource/`. Do not copy only the Lua scripts into the game's top-level `lua` folder.
4. When updating a manual installation, remove the old `garrysmod/addons/jolt` folder before extracting the new package.

## Native DLLs are still required

This package contains no DLLs and works with all three Windows DLL packages. The server or single-player host still needs the physics backend and the three native modules from the DLL package matching its game branch and architecture:

- Install `vphysics.dll` and all three `vphysics_jolt_*.dll` backends in `bin/win64/` for 64-bit games, or `bin/` for 32-bit games.
- Install `gmsv_joltgravity`, `gmsv_joltconstraints` and `gmsv_joltsoftbody` DLLs in `garrysmod/lua/bin/`.
- Back up the original `vphysics.dll` before replacing it, then fully restart the game or server after installing DLLs.

Players joining a server do not need to install these DLLs or this manual addon package; the server sends the addon's Lua files to them.

Downloads: https://github.com/schweiziske/GMod-Jolt-Physics/releases
