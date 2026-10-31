# Jolt 重力

[项目首页](../README.zh-CN.md)

修改整个世界、单个物体或一片区域的重力。Lua API 还能创建吸引场和排斥场，黑洞就建立在它们之上。安装方法见[安装指南](../gmsv_jolt/INSTALL.zh-CN.md)。

## 重力控制器

在 **Q 菜单 → 武器 → Jolt → Jolt 重力控制器**（`weapon_jolt_gravity`）中取用。

| 操作 | 效果 |
| --- | --- |
| 按住 E 并移动鼠标 | 调整方向箭头（初始朝下），松开 E 保存 |
| 左键 | 沿箭头方向施加重力。世界模式修改世界重力，物体模式覆盖准星所指物体的重力，区域模式在准星处放置重力场。 |
| 右键 | 世界模式恢复地图原有重力；物体模式清除覆盖并把倍率恢复为 1；区域模式移除自己最近放置的重力场。 |
| R | 按世界 → 物体 → 区域循环切换模式（仅限管理员；按住 E 时无效） |
| H | 区域重力场在球形和盒形之间切换 |
| Z | 撤销；物体模式下物体本身保留 |

手持控制器时，E 键不会触发「使用」。非管理员只能使用区域模式，强度最高 1500，半径或半边长最大 512。物体模式会同时修改实体的全部物理骨骼。

玩家死亡后重力场仍会保留；撤销、断线和清理地图会移除重力场，清理地图还会重置世界重力和物体重力。服务端 ConVar `vjolt_gravity_fields_per_player`（默认 16，范围 1–128）限制每名玩家的重力场数量。

个人设置位于 **Q 菜单 → 选项 → Jolt → 重力场**：

| ConVar | 默认值 | 范围 | 含义 |
| --- | --- | --- | --- |
| `vjolt_gravity_strength` | 600 | 0–10000 | 加速度，单位 Source 单位/s² |
| `vjolt_gravity_radius` | 256 | 16–32767 | 球形半径或盒形半边长，单位 Source 单位 |
| `vjolt_gravity_show_fields` | 1 | 0–1 | 在自己的画面上显示重力场轮廓 |
| `vjolt_gravity_show_direction` | 1 | 0–1 | 在自己的画面上显示方向箭头；调整方向时始终显示 |

## 规则

- 物体有覆盖重力时用覆盖值；没有时用其质心所在重力场的重力；不在任何重力场中时用世界重力。结果再乘以物体自身的倍率。
- 重叠的重力场不会叠加：`priority` 最高者生效，相同时 `slot` 最小者生效。控制器放置的重力场优先级为 0，黑洞为 50。
- 重力场作用于动态物理物体和载具，不影响玩家移动和 NPC AI。不能单独修改玩家或 NPC 的重力。
- `EnableGravity(false)` 不会清除倍率和覆盖值，重新开启重力后照常生效。
- Source 存档和复制工具（duplicator）不会保存物体的自定义重力。

## Lua API

仅限服务端。位置使用 Source 世界坐标，加速度单位为 Source 单位/s²。

```lua
require("jolt")
local available, reason = jolt.IsAvailable("gravity")
assert(available, reason)
local gravity = jolt.gravity

local original = gravity.GetWorld() -- 物理环境 0
gravity.SetWorld(Vector(0, 0, -300))

local phys = ent:GetPhysicsObject()
gravity.SetFactor(phys, 2)
gravity.SetGravity(phys, Vector(600, 0, 0)) -- 覆盖重力
phys:EnableGravity(false)
phys:EnableGravity(true)      -- 倍率 2 和覆盖值仍然保留
gravity.SetGravity(phys, nil) -- 恢复为重力场或世界重力，倍率仍为 2

local previous = JoltGravity.SetEntity(ent, Vector(0, 600, 0), 1) -- 全部物理骨骼
JoltGravity.RestoreObjects(previous)

local field = gravity.CreateField({
    shape = 0, mode = 0, -- 球形，定向
    center = Vector(0, 0, 128), radius = 256,
    gravity = Vector(0, 600, 0), priority = 10,
})
field:Update({mode = 1, strength = 600}) -- 改为吸引
local config = field:GetConfig()
field:Destroy()
gravity.SetWorld(original)
```

| 函数 | 说明 |
| --- | --- |
| `Available()` | 是否已加载配套的后端 |
| `GetWorld([env])` / `SetWorld(vector[, env])` | 世界重力；`env` 是物理环境索引，默认为 0 |
| `GetObject(phys)` | `{factor, enabled, gravity}`；没有覆盖值时 `gravity` 为 `nil` |
| `Configure(phys, {factor, enabled, gravity})` | 整体替换物体配置；省略的键分别取倍率 1、启用、无覆盖 |
| `SetFactor(phys, factor)` | 重力倍率，范围 −1024 到 1024 |
| `SetGravity(phys, vector 或 nil)` | 设置或清除覆盖重力 |
| `GetEffectiveGravity(phys)` | 当前实际加速度，已乘以倍率；重力关闭时为零 |
| `Sample(position[, env])` | 某点的世界重力或重力场重力，不含任何物体的覆盖值和倍率 |
| `CreateField(config[, env])` | 创建重力场并返回句柄 |
| `GetFields([env])` | 重力场配置数组，格式同 `GetConfig()` |
| `GetRevision()` | 世界、重力场或物体配置有任何变化时都会改变的计数 |
| `Reset([env])` | 移除全部重力场、覆盖值和倍率；保留启用状态和世界重力 |

| 重力场键 | 默认值 | 含义 |
| --- | --- | --- |
| `shape` | 0 | 0 球形，1 盒形 |
| `mode` | 0 | 0 定向，1 吸引，2 排斥 |
| `center` | `Vector(0, 0, 0)` | 中心点 |
| `radius` | 256 | 球形半径 |
| `halfExtent` | `Vector(256, 256, 256)` | 盒形半边长；盒形与世界坐标轴对齐 |
| `gravity` | `Vector(0, 0, -600)` | 定向重力场内的加速度 |
| `strength` | 600 | 吸引或排斥的加速度，0–10,000,000；场内各处大小相同，正中心处为零 |
| `priority` | 0 | 整数，±1,000,000 |

重力场句柄提供 `IsValid()`、`GetConfig()`、`Update(config)` 和 `Destroy()`：`Update` 只修改传入的键；重力场已不存在时 `Destroy()` 返回 `false`。`GetConfig()` 额外返回 `slot` 和字符串 `id`；Lua 数值无法精确表示所有 64 位整数，所以 `id` 要始终当作字符串使用。句柄被垃圾回收时重力场也会销毁，务必持有引用。后端最多容纳 1024 个重力场。

参数类型错误、配置表中出现非有限数值、`shape`／`mode`／`priority` 无效或环境索引无效时会抛出 Lua 错误；其他失败返回 `nil, reason`。物理步进期间（例如碰撞回调中）除 `field:Destroy()` 外的调用都会失败，`Destroy()` 会在下一次步进前完成。

`JoltGravity` 是 addon 在 `addons/jolt/lua/autorun/jolt_gravity.lua` 中定义的辅助表，与原生的 `jolt.gravity` 是两回事。`JoltGravity.SetEntity(ent, gravity, factor)` 一次性为全部物理骨骼设置覆盖重力（传 `nil` 清除）和倍率（省略则不变），失败时整体回滚。它不接受玩家和 NPC，返回修改前的配置（供 `JoltGravity.RestoreObjects(previous)` 撤销），或 `nil, reason`。

C++ 接口：[`public/vjolt/gravity.h`](../public/vjolt/gravity.h) 中的 `VJoltGravityBridge001`。
