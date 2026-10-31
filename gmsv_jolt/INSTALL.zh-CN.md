# 安装指南

[项目首页](../README.zh-CN.md)

## 1. 确认游戏版本

以下 Windows 版本均提供安装包，请按游戏分支和位数选择：

| 游戏版本 | 支持情况 |
| --- | --- |
| Windows 64 位，`x86-64` 测试分支 | 支持，按本页安装 |
| Windows 32 位，默认分支 | 支持，下载 `Public-Windows-x86` 安装包 |
| Windows 64 位，默认分支 | 支持，下载 `Public-Windows-x64` 安装包 |
| Linux、macOS | 不支持 |

查看或切换分支：在 Steam 库中右键 Garry's Mod →「属性」→「测试版」。默认分支选择「无」，beta 安装包对应 **x86-64 - Chromium + 64-bit binaries**。等待 Steam 下载完更新，再选择与安装包位数一致的启动方式。

默认分支和 `x86-64` 分支的 64 位版本二进制接口（ABI）不同，DLL 不能混用。

## 2. 下载

从 [Releases](https://github.com/schweiziske/GMod-Jolt-Physics/releases) 下载最新版本。安装包包含三部分：

| 文件 | 作用 | 是否必需 |
| --- | --- | --- |
| `vphysics.dll`、`vphysics_jolt_avx2.dll`、`vphysics_jolt_sse42.dll`、`vphysics_jolt_sse2.dll` | Jolt 物理引擎 | 必需 |
| `gmsv_joltgravity_win64.dll`、`gmsv_joltconstraints_win64.dll`、`gmsv_joltsoftbody_win64.dll` | 额外玩法的原生模块 | 可选 |
| `jolt` 文件夹 | 额外玩法的工具、武器和界面 | 可选 |

四个物理 DLL 必须一起安装：`vphysics.dll` 负责检测 CPU 支持的指令集，再加载对应的 AVX2、SSE4.2 或 SSE2 版本。只想替换物理引擎时，可以不装后两部分；想用黑洞、软体等[额外玩法](GAMEPLAY.zh-CN.md)，则三部分都要装。

## 3. 安装

在 Steam 库中右键 Garry's Mod →「管理」→「浏览本地文件」，打开游戏目录（里面有 `bin` 和 `garrysmod` 两个文件夹）。

1. 备份 `bin/win64/vphysics.dll`，例如复制一份并改名为 `vphysics.dll.bak`。
2. 把四个物理 DLL 复制到 `bin/win64/`，覆盖原有的 `vphysics.dll`。
3. 把三个 `gmsv_jolt*_win64.dll` 复制到 `garrysmod/lua/bin/`。没有这个文件夹就新建一个。
4. 把 `jolt` 文件夹复制到 `garrysmod/addons/`。
5. 完全退出游戏后重新启动。换地图不会重新加载 DLL。

32 位 public 安装包使用 `bin/`，而不是 `bin/win64/`；原生模块使用 `gmsv_jolt*_win32.dll`，而不是 `*_win64.dll`。每个 ZIP 都已包含对应的目录结构，直接把内容复制到游戏目录即可。

两个 64 位安装包装好后的目录结构：

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

## 4. 检查是否生效

进入任意地图，打开控制台（`~` 键；如果没反应，先在「选项」→「键盘」→「高级」里启用开发者控制台），输入：

```text
vjolt_substeps
```

输出 `"vjolt_substeps" = "1"` 之类的内容，说明 Jolt 物理已经接管；输出 `Unknown command` 则说明游戏仍在使用原版物理，请对照下方的常见问题检查。

安装了额外玩法时，按 Q 打开生成菜单，「工具」和「武器」页都应出现 **Jolt** 分类。

## 专用服务器

- 用 SteamCMD 安装 `x86-64` 分支：`app_update 4020 -beta x86-64 validate`，并用 64 位的 `srcds_win64.exe` 启动。
- 文件放置方式与客户端相同：`<服务器目录>/bin/win64/`、`<服务器目录>/garrysmod/lua/bin/` 和 `<服务器目录>/garrysmod/addons/`。
- 玩家加入服务器时无需安装任何东西，addon 的 Lua 文件会自动下发。
- 物理模拟默认使用「CPU 线程数 − 1」个工作线程。启动服务器前设置环境变量 `VJOLT_WORKER_THREADS` 可以指定线程数（0–64，0 表示不使用工作线程）。
- 游戏的模拟线程会被固定在一个核心上（默认选性能最高的核心），物理工作线程使用其余核心，且始终限定在进程的 CPU 亲和性范围内。设置环境变量 `VJOLT_SOURCE_CPU`（逻辑处理器编号 0–63）可以手动指定这个核心。

## 更新与卸载

- **更新**：DLL 直接覆盖；addon 先删除旧的 `garrysmod/addons/jolt` 再复制新版，以免残留旧脚本。
- **卸载**：把备份的 `vphysics.dll.bak` 改回 `vphysics.dll`，或者在 Steam 中右键 Garry's Mod →「属性」→「已安装文件」→「验证游戏文件的完整性」。然后删除 `bin/win64/vphysics_jolt_*.dll`、`garrysmod/lua/bin/gmsv_jolt*.dll` 和 `garrysmod/addons/jolt`。
- **游戏更新后**：Steam 可能会还原 `vphysics.dll`。如果 `vjolt_substeps` 变成 `Unknown command`，重新复制四个物理 DLL 即可。

## 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| 游戏一启动就崩溃 | 分支不对，或物理 DLL 不全。确认在 `x86-64` 分支，且四个物理 DLL 都在 `bin/win64/`。 |
| `vjolt_substeps` 输出 `Unknown command` | DLL 放错了目录，或者游戏是以 32 位启动的。 |
| 生成菜单里没有 Jolt 分类 | addon 位置不对。正确路径是 `garrysmod/addons/jolt/addon.json`，中间不要多套一层文件夹。 |
| 使用 Jolt 工具时提示功能不可用 | `garrysmod/lua/bin/` 里缺少对应的 `gmsv_jolt*_win64.dll`，或者 Jolt 物理没有生效。 |

仍然无法解决，请到 [Issue 区](https://github.com/schweiziske/GMod-Jolt-Physics/issues)反馈，附上游戏分支、CPU 型号和控制台报错。
