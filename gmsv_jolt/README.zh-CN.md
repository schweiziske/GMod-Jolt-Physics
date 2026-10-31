# Jolt Lua 模块

[项目首页](../README.zh-CN.md)

`require("jolt")` 是服务端 Lua 访问三个原生模块的统一入口：

| 命名空间 | 原生模块 | 提供的能力 | API 文档 |
| --- | --- | --- | --- |
| `jolt.gravity` | `gmsv_joltgravity` | 世界与单个物体的重力、重力场（黑洞基于此实现） | [重力](../gmsv_joltgravity/README.zh-CN.md) |
| `jolt.constraints` | `gmsv_joltconstraints` | 齿轮、齿条和轨道 | [齿轮与齿条](../gmsv_joltconstraints/README.zh-CN.md)、[轨道](../gmsv_joltconstraints/PATH.zh-CN.md) |
| `jolt.softbody` | `gmsv_joltsoftbody` | 布料、充气网格、顶点固定 | [软体](../gmsv_joltsoftbody/README.zh-CN.md) |

`require("jolt")` 本身不加载任何原生模块，各功能在第一次使用时才加载：

| 调用 | 结果 |
| --- | --- |
| `jolt.ListFeatures()` | 返回 `{"gravity", "constraints", "softbody"}`，不加载模块 |
| `jolt.IsAvailable(name)` | 加载模块并检查后端，返回 `true`，或 `false, reason` |
| `jolt.Load(name)` | 返回功能表，或 `nil, reason` |
| `jolt.gravity` 等命名空间 | 首次访问时加载；不可用时抛出 Lua 错误 |

原生功能只在服务端运行，客户端调用 `jolt.Load` 会返回 `nil, reason`。如果你的 addon 在没有 Jolt 时也要正常工作，请先用 `IsAvailable` 检查：

```lua
require("jolt")
local available, reason = jolt.IsAvailable("gravity")
if not available then
    print("Jolt 重力不可用：" .. reason)
    return
end
```

所有模块都直接操作正在运行的 Jolt 后端里的物理世界，不会另建 Jolt 世界。

- 构建模块：[构建指南](../docs/build.zh-CN.md#lua-模块与-addon)
- API 总览与 C++ 接入：[物理 API](../docs/physics-api.zh-CN.md)
- 编写自己的模块：[功能导出示例](../examples/physics/export/README.zh-CN.md)
