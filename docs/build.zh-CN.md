# 构建指南

[项目首页](../README.zh-CN.md)

普通玩家直接下载 [Releases](https://github.com/schweiziske/GMod-Jolt-Physics/releases) 即可，安装方法见[安装指南](../gmsv_jolt/INSTALL.zh-CN.md)。本页面向开发者，以及需要 32 位版本的用户。

项目分为两部分，可以分别构建：

| 部分 | 产物 | 外部依赖 |
| --- | --- | --- |
| [Lua 模块与 addon](#lua-模块与-addon) | `gmsv_joltgravity`、`gmsv_joltconstraints`、`gmsv_joltsoftbody` 三个 DLL 和 `addons/jolt` | `garrysmod_common` |
| [物理后端](#物理后端) | `vphysics.dll` 和 `vphysics_jolt_avx2/sse42/sse2.dll` | Jolt 子模块、Source/GMod SDK |

公共工具链：Windows、Visual Studio 2022（MSVC v143，安装「使用 C++ 的桌面开发」）、CMake 3.24 及以上、Git。

## Lua 模块与 addon

只需要 [garrysmod_common](https://github.com/danielga/garrysmod_common)，不需要 Source/GMod SDK，也不需要初始化 Jolt 子模块。

```powershell
git clone https://github.com/danielga/garrysmod_common C:/deps/garrysmod_common
$GmodCommon = "C:/deps/garrysmod_common"

cmake -S gmsv_jolt -B .cache/jolt-modules -G "Visual Studio 17 2022" -A x64 "-DGMOD_COMMON=$GmodCommon"
cmake --build .cache/jolt-modules --config Release --parallel
cmake --install .cache/jolt-modules --config Release --component Runtime `
  --prefix .cache/jolt-package/garrysmod
```

安装步骤把产物整理到 `.cache/jolt-package/garrysmod/`，目录结构与游戏的 `garrysmod/` 一致：

- `lua/bin/gmsv_joltgravity_win64.dll` 等三个模块
- `addons/jolt/`

可选配置：

| 选项 | 说明 |
| --- | --- |
| `-A Win32` | 构建 32 位模块（文件名后缀为 `_win32`）。请使用单独的构建目录。 |
| `-DVJOLT_LUA_FEATURES="gravity;constraints"` | 只构建部分模块，可选 `gravity`、`constraints`、`softbody`。完整的 addon 需要全部三个。 |

## 物理后端

### 准备依赖

1. 初始化定制的 Jolt 子模块：

   ```powershell
   git submodule update --init --recursive
   ```

2. 准备与目标游戏二进制接口（ABI）匹配的 Source/GMod SDK。SDK 不随本仓库提供，例如可以使用 [mini-source-sdk](https://github.com/RaphaelIT7/mini-source-sdk)（`gmod-changes` 分支）中的 `gmod/src`。SDK 根目录需要满足：
   - 包含 `public/vphysics_interface.h`；
   - 已经用 Release、`/MT`、与目标相同的架构编译好 `tier0.lib`、`tier1.lib`、`vstdlib.lib`、`mathlib.lib`、`tier2.lib`，64 位放在 `lib/public/x64`，32 位放在 `lib/public`。

默认分支的 64 位程序与 `x86-64` 测试分支的 ABI 不同，必须选用与目标游戏对应的头文件和库，不能只看是不是 64 位。

### 64 位完整包

```powershell
$SdkRoot = "C:/deps/mini-source-sdk/gmod/src"
./vphysics_jolt/build-package.ps1 -SdkRoot $SdkRoot -BuildRoot "$PWD/.cache/physics-package"
```

脚本依次构建 AVX2、SSE4.2、SSE2 三个后端和 CPU 检测包装层 `vphysics.dll`，产物位于 `.cache/physics-package/package/`：四个 DLL、对应的 PDB，以及记录 SHA-256 和源码版本的 `manifest.json`。

### 单个变体与 32 位（实验性）

调试时可以只构建一个 CPU 变体：

```powershell
cmake -S vphysics_jolt -B .cache/backend-avx2 -G "Visual Studio 17 2022" -A x64 `
  "-DVOLT_GMOD_SDK=$SdkRoot" -DVOLT_CPU_VARIANT=avx2
cmake --build .cache/backend-avx2 --config Release --target vphysics vphysics_jolt_avx2 --parallel
```

| 选项 | 说明 |
| --- | --- |
| `-DVOLT_GMOD_SDK=<路径>` | SDK 根目录，必填。 |
| `-DVOLT_CPU_VARIANT=avx2` | 要构建的后端：`avx2`（默认）、`sse42` 或 `sse2`。每个变体使用单独的构建目录。 |
| `-DVOLT_JOLT_CHECKED_BUILD=ON` | 开启 Jolt 内部断言用于调试：断言失败时输出信息并直接退出进程。仍使用 SDK 静态库要求的 Release 运行库。请使用单独的构建目录，不要用于发布。 |

32 位版本改用 `-A Win32` 和 32 位的 SDK 库，每个变体使用单独的构建目录。包装层按 CPU 选择要加载的后端，所选文件缺失时不会退回到其他变体，所以发布时三个变体都要构建。

## 安装构建产物

文件放置位置与[安装指南](../gmsv_jolt/INSTALL.zh-CN.md#3-安装)相同：

| 产物 | 64 位游戏 | 32 位游戏 |
| --- | --- | --- |
| `vphysics.dll`、`vphysics_jolt_*.dll` | `bin/win64/` | `bin/` |
| `gmsv_jolt*_win64.dll` / `gmsv_jolt*_win32.dll` | `garrysmod/lua/bin/` | `garrysmod/lua/bin/` |
| `addons/jolt/` | `garrysmod/addons/jolt/` | `garrysmod/addons/jolt/` |

后端、模块和游戏的架构与 ABI 必须一致。替换 DLL 后要完全重启游戏。

## 下一步

- 开发新功能：[物理 API](physics-api.zh-CN.md)、[功能导出示例](../examples/physics/export/README.zh-CN.md)
- 了解后端代码结构：[后端代码导航](../vphysics_jolt/README.zh-CN.md)
