# Build Guide

[Home](../README.md)

Most players should download a [release](https://github.com/schweiziske/GMod-Jolt-Physics/releases) and follow the [installation guide](../gmsv_jolt/INSTALL.md). This page is for developers and for anyone who needs a 32-bit build.

The project has two parts that build independently:

| Part | Output | External dependencies |
| --- | --- | --- |
| [Lua modules and addon](#lua-modules-and-addon) | `gmsv_joltgravity`, `gmsv_joltconstraints` and `gmsv_joltsoftbody` DLLs, plus `addons/jolt` | `garrysmod_common` |
| [Physics backend](#physics-backend) | `vphysics.dll` and `vphysics_jolt_avx2/sse42/sse2.dll` | Jolt submodule, Source/GMod SDK |

Both need Windows, Visual Studio 2022 (MSVC v143, with the "Desktop development with C++" workload), CMake 3.24 or newer, and Git.

## Lua modules and addon

All you need is [garrysmod_common](https://github.com/danielga/garrysmod_common). The Source/GMod SDK and the Jolt submodule are not required.

```powershell
git clone https://github.com/danielga/garrysmod_common C:/deps/garrysmod_common
$GmodCommon = "C:/deps/garrysmod_common"

cmake -S gmsv_jolt -B .cache/jolt-modules -G "Visual Studio 17 2022" -A x64 "-DGMOD_COMMON=$GmodCommon"
cmake --build .cache/jolt-modules --config Release --parallel
cmake --install .cache/jolt-modules --config Release --component Runtime `
  --prefix .cache/jolt-package/garrysmod
```

The install step stages everything under `.cache/jolt-package/garrysmod/`, mirroring the game's `garrysmod/` folder:

- `lua/bin/gmsv_joltgravity_win64.dll` and the other two modules
- `addons/jolt/`

Options:

| Option | Effect |
| --- | --- |
| `-A Win32` | Build 32-bit modules (suffix `_win32`). Use a separate build directory. |
| `-DVJOLT_LUA_FEATURES="gravity;constraints"` | Build only some modules: any of `gravity`, `constraints`, `softbody`. The full addon needs all three. |

## Physics backend

### Dependencies

1. Initialize the customized Jolt submodule:

   ```powershell
   git submodule update --init --recursive
   ```

2. Get a Source/GMod SDK that matches the target game's ABI. The SDK is not included in this repository; `gmod/src` from [mini-source-sdk](https://github.com/RaphaelIT7/mini-source-sdk) (`gmod-changes` branch) is one option. The SDK root must:
   - contain `public/vphysics_interface.h`;
   - have `tier0.lib`, `tier1.lib`, `vstdlib.lib`, `mathlib.lib` and `tier2.lib` already built (Release, `/MT`, same architecture as the target) in `lib/public/x64` for 64-bit or `lib/public` for 32-bit.

The default branch's 64-bit build and the `x86-64` beta branch use different ABIs. Pick the headers and libraries for the exact game you target; matching the pointer size is not enough.

### 64-bit release package

```powershell
$SdkRoot = "C:/deps/mini-source-sdk/gmod/src"
./vphysics_jolt/build-package.ps1 -SdkRoot $SdkRoot -BuildRoot "$PWD/.cache/physics-package"
```

The script builds the AVX2, SSE4.2 and SSE2 backends plus the CPU-detecting wrapper `vphysics.dll`. Everything lands in `.cache/physics-package/package/`: the four DLLs, their PDBs and a `manifest.json` with SHA-256 hashes and source revisions.

### Single variant and 32-bit (experimental)

While developing, you can build one CPU variant:

```powershell
cmake -S vphysics_jolt -B .cache/backend-avx2 -G "Visual Studio 17 2022" -A x64 `
  "-DVOLT_GMOD_SDK=$SdkRoot" -DVOLT_CPU_VARIANT=avx2
cmake --build .cache/backend-avx2 --config Release --target vphysics vphysics_jolt_avx2 --parallel
```

| Option | Effect |
| --- | --- |
| `-DVOLT_GMOD_SDK=<path>` | SDK root; required. |
| `-DVOLT_CPU_VARIANT=avx2` | Backend to build: `avx2` (default), `sse42` or `sse2`. Use one build directory per variant. |
| `-DVOLT_JOLT_CHECKED_BUILD=ON` | Debug build of Jolt's internal checks: a failed assertion prints it and exits the process. Still uses the Release runtime the SDK libraries need. Use a separate build directory and never ship it. |

For 32-bit, use `-A Win32` with 32-bit SDK libraries and a separate build directory per variant. The wrapper picks one backend based on the CPU and does not fall back to another variant if that file is missing, so build all three for distribution.

## Installing your build

Files go in the same places as in the [installation guide](../gmsv_jolt/INSTALL.md#3-install):

| Output | 64-bit game | 32-bit game |
| --- | --- | --- |
| `vphysics.dll`, `vphysics_jolt_*.dll` | `bin/win64/` | `bin/` |
| `gmsv_jolt*_win64.dll` / `gmsv_jolt*_win32.dll` | `garrysmod/lua/bin/` | `garrysmod/lua/bin/` |
| `addons/jolt/` | `garrysmod/addons/jolt/` | `garrysmod/addons/jolt/` |

The backend, the modules and the game must share the same architecture and ABI. Restart the game completely after replacing DLLs.

## Next steps

- Develop a new feature: [physics API](physics-api.md) and the [feature export example](../examples/physics/export/README.md)
- Find your way around the backend: [backend code map](../vphysics_jolt/README.md)
