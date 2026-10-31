# GMod Jolt Physics

[English](README.md)

GMod Jolt Physics 用 [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 替换 Garry's Mod 原有的、基于 IVP/Havok 的物理引擎（VPhysics）。目标是保留 Source 玩家熟悉的物理手感——道具、布娃娃、载具、约束和物理枪照常工作——同时让物理模拟运行在 Jolt 的多线程求解器上。

此外还新增了五项 Sandbox 玩法：**软体、黑洞、重力场、齿轮/齿条和轨道**。

项目由 [schweiziske](https://github.com/schweiziske) 在 [Volt（VPhysics-Jolt）](https://github.com/misyltoad/VPhysics-Jolt)的基础上开发，详见[致谢](#致谢)。

## 快速开始

1. 从 [Releases](https://github.com/schweiziske/GMod-Jolt-Physics/releases) 下载最新版本。
2. 按 [安装指南](gmsv_jolt/INSTALL.zh-CN.md) 把文件复制到游戏目录，然后重启游戏。

| 游戏版本 | 支持情况 |
| --- | --- |
| Windows 64 位，`x86-64` 测试分支 | 支持 |
| Windows 32 位 | 支持 |
| Windows 64 位，默认分支 | 支持 |
| Linux、macOS |  暂时不支持 |

默认分支（公开版）和 `x86-64` 测试分支都有 64 位版本，但两者的二进制接口（ABI）不同，**64 位 DLL 不能混用**。请按游戏版本选择安装包：

| 安装包 | 游戏版本 |
| --- | --- |
| `GMod-Jolt-Physics-Public-Windows-x86.zip` | 默认（public）分支，32 位 |
| `GMod-Jolt-Physics-Public-Windows-x64.zip` | 默认（public）分支，64 位 |
| `GMod-Jolt-Physics-Beta-x86-64-Branch-Windows-x64.zip` | Steam `x86-64` 测试分支，64 位 |

## 功能

下表对比原版 VPhysics、Volt 和 GMod Jolt Physics。

| 功能 | VPhysics | Volt | GMod Jolt Physics |
| :--- | :---: | :---: | :---: |
| 约束（滑轮除外） | ✔️ | ✔️ | ✔️ |
| 滑轮 | ✔️ | ✔️ | ✔️ |
| 可断裂约束 | ✔️ | ❌ | ✔️ |
| 马达（运动控制器） | ✔️ | ✔️ | ✔️ |
| 马达（约束） | ✔️ | ✔️ | ✔️ |
| 布娃娃 | ✔️ | ✔️ | ✔️ |
| 触发器 | ✔️ | ✔️ | ✔️ |
| 物体触碰回调 | ✔️ | ✔️ | ✔️ |
| 道具受损与破碎 | ✔️ | ✔️ | ✔️ |
| 流体事件 | ✔️ | ✔️ | ✔️ |
| 道具溅水效果 | ✔️ | ✔️ | ✔️ |
| 轮式载具 | ✔️ | ✔️ | ✔️ |
| 射线载具（如气垫船） | ✔️ | ❌ | ✔️ |
| NPC 与门的影子控制器 | ✔️ | ✔️ | ✔️ |
| 存档与读档 | ✔️ | ✔️ | ✔️ |
| 单个物体的碰撞回调（如无碰撞） | ✔️ | ✔️ | ✔️ |
| 多线程物理模拟 | ❌ | ✔️ | ✔️ |
| 玩家物理控制器 | ✔️ | ✔️ | ✔️ |

### 新增玩法

| 玩法 | 简介 |
| --- | --- |
| [软体](gmsv_joltsoftbody/README.zh-CN.md) | 布料和充气软球，可以拖动、固定顶点 |
| [黑洞](gmsv_jolt/GAMEPLAY.zh-CN.md#黑洞) | 先把周围物体吸过来，再弹开 |
| [重力场](gmsv_joltgravity/README.zh-CN.md) | 修改整个世界、单个物体或一片区域的重力 |
| [齿轮与齿条](gmsv_joltconstraints/README.zh-CN.md) | 按传动比联动两个物体，或把转动变成平移 |
| [轨道](gmsv_joltconstraints/PATH.zh-CN.md) | 让物体沿三维曲线运动 |

它们位于 Q 菜单的「工具 → Jolt」和「武器 → Jolt」，操作方法见[额外玩法](gmsv_jolt/GAMEPLAY.zh-CN.md)。addon 作者也可以通过 `require("jolt")` 在 Lua 中调用这些能力。

## 开发者

- [构建指南](docs/build.zh-CN.md)：Lua 模块只需要 `garrysmod_common`；物理后端还需要 Source/GMod SDK。
- [物理 API](docs/physics-api.zh-CN.md)：在 Lua 或 C++ 中使用额外玩法的能力。
- [功能导出示例](examples/physics/export/README.zh-CN.md)：把新的 Jolt 能力从后端一路导出到 Lua。
- [后端代码导航](vphysics_jolt/README.zh-CN.md)：物理后端各部分的位置，以及修改时要遵守的约定。

## 问题反馈

GMod Jolt Physics 仍有不少问题。欢迎在 [Issue 区](https://github.com/schweiziske/GMod-Jolt-Physics/issues)反馈，请尽量附上：

- 游戏分支，以及游戏以 32 位还是 64 位运行；
- CPU 型号；
- 地图和已安装的 addon；
- 复现步骤；
- 控制台报错或崩溃转储（如有）。

## 致谢

- **GMod Jolt Physics**：[schweiziske](https://github.com/schweiziske)，负责 GMod 兼容层、求解器集成和新增玩法。
- **Volt / VPhysics-Jolt**：[Joshua Ashton](https://github.com/Joshua-Ashton)（🐸✨）、[Josh Dowell（Slartibarty）](https://github.com/Slartibarty)、[RaphaelIT7](https://github.com/RaphaelIT7)，以及上游项目的其他贡献者。
- **Jolt Physics**：Jorrit Rouwe 及 Jolt Physics 的各位贡献者。

## 演示视频

以下视频来自 Volt 原项目。

<table>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=gPDQkmfQCsc"><img src="https://img.youtube.com/vi/gPDQkmfQCsc/mqdefault.jpg" alt="大量西瓜与垃圾箱"></a><br>大量西瓜与垃圾箱</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=tVmQTmbSJM0"><img src="https://img.youtube.com/vi/tVmQTmbSJM0/mqdefault.jpg" alt="物理模拟的链条"></a><br>物理模拟的链条</td>
  </tr>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=tYfiTyRtmz8"><img src="https://img.youtube.com/vi/tYfiTyRtmz8/mqdefault.jpg" alt="大量小球测试"></a><br>大量小球测试</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=5_QbbXbIrg8"><img src="https://img.youtube.com/vi/5_QbbXbIrg8/mqdefault.jpg" alt="车轮与焊接车辆 dupe 测试"></a><br>车轮与焊接车辆 dupe 测试</td>
  </tr>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=SdEj7HTuJmU"><img src="https://img.youtube.com/vi/SdEj7HTuJmU/mqdefault.jpg" alt="门与 NPC（物理影子对象）测试"></a><br>门与 NPC（物理影子对象）测试</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=CLVnSwg33Dk"><img src="https://img.youtube.com/vi/CLVnSwg33Dk/mqdefault.jpg" alt="大量方块、布娃娃与漏斗"></a><br>大量方块、布娃娃与漏斗</td>
  </tr>
  <tr>
    <td align="center"><a href="https://www.youtube.com/watch?v=GzW_4bufwEk"><img src="https://img.youtube.com/vi/GzW_4bufwEk/mqdefault.jpg" alt="慢动作方块"></a><br>慢动作方块</td>
    <td align="center"><a href="https://www.youtube.com/watch?v=10vvRJVHGQc"><img src="https://img.youtube.com/vi/10vvRJVHGQc/mqdefault.jpg" alt="垃圾箱里的丙烷罐"></a><br>垃圾箱里的丙烷罐</td>
  </tr>
</table>