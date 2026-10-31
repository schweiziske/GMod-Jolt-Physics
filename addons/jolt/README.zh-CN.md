# Jolt Addon

[项目首页](../../README.zh-CN.md)

GMod Jolt Physics 的 Sandbox 工具和武器：软体、黑洞、重力场、齿轮/齿条和轨道。

| Q 菜单 | 内容 |
| --- | --- |
| 工具 → Jolt | 黑洞、齿轮 / 齿条、路径轨道 · PathConstraint、软体工坊 |
| 武器 → Jolt | Jolt 重力控制器、Jolt 黑洞手雷 |
| 选项 → Jolt | 重力场、黑洞、软体 |

服务端需要 Jolt 物理后端，以及 `gmsv_joltgravity`、`gmsv_joltconstraints`、`gmsv_joltsoftbody` 三个原生模块。玩家加入服务器时无需安装任何东西，服务器会自动下发 addon 的 Lua 文件。

- 安装：[安装指南](../../gmsv_jolt/INSTALL.zh-CN.md)
- 操作与设置：[额外玩法](../../gmsv_jolt/GAMEPLAY.zh-CN.md)
- 翻译维护：[本地化说明](docs/LOCALIZATION.md)（英文）
- 软体渲染与网络同步的实现：[软体客户端](docs/SOFTBODY_CLIENT.zh-CN.md)
