# Jolt 软体

[项目首页](../README.zh-CN.md)

软体工坊可以在 Sandbox 中放置布料和充气软球，`jolt.softbody` Lua API 则能用自己的三角网格创建软体。安装方法见[安装指南](../gmsv_jolt/INSTALL.zh-CN.md)；工坊如何同步和绘制软体，见[软体客户端说明](../addons/jolt/docs/SOFTBODY_CLIENT.zh-CN.md)。

## 软体工坊

打开 **Q 菜单 → 工具 → Jolt → 软体工坊**，在面板中选择模式。

**创建软体**：选择布料或软球，设置尺寸、网格精度、总质量（默认 4 kg，由全部顶点平分）、柔软程度、软球充气量、布料固定点（不固定、顶部两角或整条顶边）、颜色、旋转角度和玩家碰撞。其下的求解参数受服务端设置限制。面板会显示顶点数、三角形数和每名附近玩家占用的带宽。左键点击 600 单位以内的表面即可放置；Z 和 Sandbox 的清理功能都能移除软体。

柔软程度按对数比例调整边约束和弯曲约束的柔度。充气量由软球的质量和尺寸换算成气体能量 nRT，既不是以帕斯卡为单位的压强，也不是目标体积。

**编辑顶点**：瞄准 600 单位以内的软体。蓝色是当前选中的顶点，金色是固定点。

| 操作 | 效果 |
| --- | --- |
| 按住左键 | 拖动选中的顶点；拖动期间它被临时固定，松开后恢复原来的固定状态 |
| 右键 | 固定或松开选中的顶点；固定点锚定在世界中 |
| R，或切换工具 | 结束拖动 |

只有所有者或管理员能编辑软体，并且要通过 CanTool 和 CPPI 检查；同一时间只能有一名玩家拖动它。服务端会重新计算瞄准射线，并检查墙体遮挡和距离，客户端不会上传顶点编号或坐标。玩家死亡、断线、软体被删除或 Lua 重载时，拖动都会结束。物理枪抓不起软体。

开启「玩家碰撞」时，软体会挡住玩家：软体实体会针对实时网格响应 Source 射线检测，服务端检测原生软体，客户端检测自己绘制的网格。默认关闭，玩家可以穿过软体，服务端以有限频率检查重叠，并根据玩家的质量和速度给软体施加冲量。这只是近似，并非连续的接触求解。子弹、爆炸等伤害的冲击力也会在命中点附近推动软体。

| 网格精度 | 布料顶点 / 三角形 | 软球顶点 / 三角形 | 每名附近玩家的带宽（KiB/s，布料 / 软球） |
| --- | --- | --- | --- |
| 轻量 | 64 / 98 | 42 / 80 | 6.2 / 4.2 |
| 标准 | 256 / 450 | 162 / 320 | 23.4 / 15.0 |
| 精细 | 576 / 1058 | 642 / 1280 | 52.1 / 58.0 |

带宽指软体运动时按 15 Hz 发送的快照数据量，不含网络开销。精度越高，服务端模拟、带宽和客户端绘制的开销都越大。

距玩家 1200 单位以内的软体最高以 15 Hz 更新，更远的以 5 Hz 更新，超过 3000 单位或离开玩家的 PVS 后不再更新。静止的软体不发送数据。客户端以约 100 ms 的延迟插值显示，不做软体物理预测。复制工具（duplicator）不会复制软体。

### 服务端设置

管理员也可以在 **Q 菜单 → 选项 → Jolt → 软体**中修改。尺寸和求解参数上限只影响之后新建的软体；网络和玩家推动设置立即生效。

| ConVar | 默认值 | 含义 |
| --- | --- | --- |
| `sbox_maxjolt_softbodies` | 3 | 每名玩家的软体数量 |
| `jolt_softbody_maxrecords` | 32 | 全服软体数量；addon 实际最多管理 32 个 |
| `jolt_softbody_maxvertices` | 4096 | 全服顶点总数 |
| `jolt_softbody_maxvertices_per_body` / `jolt_softbody_maxtriangles_per_body` | 1024 / 2048 | 单个软体的顶点数 / 三角形数 |
| `jolt_softbody_iterations` | 5 | 求解迭代次数上限 |
| `jolt_softbody_damping` | 0.35 | 阻尼上限 |
| `jolt_softbody_friction` / `jolt_softbody_restitution` | 0.6 / 0.05 | 摩擦力 / 回弹系数上限 |
| `jolt_softbody_vertex_radius` | 0.5 | 顶点碰撞半径上限 |
| `jolt_softbody_max_velocity` | 2000 | 顶点速度上限，单位 Source 单位/秒 |
| `jolt_softbody_gravity_factor` | 1 | 允许的重力倍率绝对值上限（至少为 1） |
| `jolt_softbody_player_push_rate` | 15 | 每名玩家对每个软体每秒的重叠检查次数 |
| `jolt_softbody_player_push_budget` | 32 | 每个 Tick 检查的玩家与软体组合数 |
| `jolt_softbody_player_push_impulse` | 1 | 玩家推动冲量倍率 |
| `jolt_softbody_net_kib` | 96 | 每名玩家的网络预算，单位 KiB/s |
| `jolt_softbody_near_distance` / `jolt_softbody_view_distance` | 1200 / 3000 | 按近处频率更新的距离 / 同步距离 |
| `jolt_softbody_near_rate` / `jolt_softbody_far_rate` | 15 / 5 | 近处 / 远处的快照频率，单位 Hz |
| `jolt_softbody_interest_rate` | 4 | 每名玩家附近软体列表的刷新频率，单位 Hz |

### 客户端设置

位于 **Q 菜单 → 选项 → Jolt → 软体**；前三项也出现在工具面板的「显示与性能」中。

| ConVar | 默认值 | 含义 |
| --- | --- | --- |
| `jolt_softbody_viewdistance` | 3000 | 可见距离，256–16384 |
| `jolt_softbody_drawrate` | 0 | 每秒插值刷新次数，最高 144；0 表示跟随帧率，低于 15 的值按 15 处理 |
| `jolt_softbody_wireframe` | 0 | 在「编辑顶点」模式下显示所选软体的网格线 |
| `jolt_softbody_collision_profile` | 0 | 统计客户端碰撞查询耗时，供 `jolt_softbody_client_stats` 输出 |

## Lua API

仅限服务端。`jolt.softbody` 提供 `Create(vertices, triangles[, settings])`、`Available()`、`GetVersion()` 和 `GetLimits()`；`GetLimits()` 返回 `{verticesPerBody, trianglesPerBody, bodies, totalVertices}`。

```lua
require("jolt")
local available, reason = jolt.IsAvailable("softbody")
assert(available, reason)

-- 布料：固定两个角。
local body, err = jolt.softbody.Create({
    {pos = Vector(-32, -32, 0), mass = 1, pinned = true},
    {pos = Vector( 32, -32, 0), mass = 1, pinned = true},
    {pos = Vector( 32,  32, 0), mass = 1},
    {pos = Vector(-32,  32, 0), mass = 1},
}, {
    {1, 2, 3}, {1, 3, 4},
}, {
    environment = 0,
    pos = Vector(0, 0, 128),
    angles = Angle(0, 0, 0),
    edgeCompliance = 0.00001,
    bendCompliance = 0.001,
    pressure = 0,
})
assert(body, err)

-- prop 是持有该软体的现有实体：保存引用，并在实体删除时一起销毁软体。
prop.MySoftBody = body
prop:CallOnRemove("MySoftBody", function() body:Destroy() end)
local snapshot = {positions = {}, pinned = {}}
assert(body:ReadSnapshot(snapshot, false))
body:AddImpulse(Vector(0, 0, 100))
```

`Create` 接受由 3–8192 个顶点、1–16384 个三角形组成的连通三角网格。顶点可以写成 `Vector`（1 kg，不固定），也可以写成 `{pos = Vector, mass = number, pinned = boolean}`，质量范围 0.001–10000 kg；固定顶点请用 `pinned`，不要用极大的质量代替。顶点位置相对于 `pos` 和 `angles`；初始速度和读取到的所有数据都使用世界坐标。Lua 索引从 **1** 开始，C++ 索引从 **0** 开始。输入数组会被复制。

网格必须连通，并用到每一个顶点。面不能重复，共用边的绕序必须一致，一条边最多属于两个面。退化三角形、过短的边和近乎共线的面会被拒绝。开放网格适合做布料；封闭网格必须朝外绕序。所有顶点的最终位置都必须在 ±1,048,576 Source 单位以内。

最小的封闭网格是一个充压的四面体：

```lua
local ball, reason = jolt.softbody.Create({
    Vector(0, 0, 0), Vector(32, 0, 0),
    Vector(0, 32, 0), Vector(0, 0, 32),
}, {
    {1, 3, 2}, {1, 2, 4}, {1, 4, 3}, {2, 3, 4},
}, {
    pos = Vector(0, 0, 128),
    pressure = 10000,
    edgeCompliance = 0.01,
    bendCompliance = 0.01,
})
assert(ball, reason)
```

`pressure` 是 Jolt 的气体能量 nRT，单位 kg·Source 单位²/s²；除以当前体积才得到压强，所以它既不是帕斯卡，也不是固定体积。开放网格在创建时和调用 `SetPressure` 时都不接受非零压力。

| 参数 | 默认值 | 范围与含义 |
| --- | --- | --- |
| `environment` | 0 | VPhysics 环境索引；软体始终属于该环境 |
| `pos`、`angles` | 零 | 初始位置和 Source 欧拉角 |
| `velocity` | 零 | 未固定顶点的初始世界速度，单位 Source 单位/秒，不超过 `maxVelocity` |
| `iterations` | 5 | 求解迭代次数，1–32 |
| `edgeCompliance` | 0.00001 | 边及自动生成的剪切边的柔度，0–1，单位 s²/kg |
| `bendCompliance` | 0.001 | 弯曲柔度，0–1；距离弯曲的单位为 s²/kg |
| `dihedralBend` | `false` | 改用相邻三角面之间的夹角计算弯曲 |
| `pressure` | 0 | 封闭网格的 nRT，0–1e9 |
| `gravityFactor` | 1 | 世界重力倍率，−100 到 100 |
| `damping` | 0.1 | 线性阻尼，0–10 |
| `friction`、`restitution` | 0.5、0 | 0–1 |
| `vertexRadius` | 0.5 | 顶点碰撞半径，0–64 Source 单位 |
| `maxVelocity` | 4000 | 顶点速度上限，1–100000 Source 单位/秒 |
| `allowSleeping` | `true` | 是否允许软体休眠 |
| `doubleSided` | `true` | 碰撞查询中是否把三角面视为双面 |

柔度是刚度的倒数：值越大越软，0 表示刚性约束。启用 `dihedralBend` 后，`bendCompliance` 作用于角度，单位是以 Jolt 米制计的 s²/(kg·m²)，不能与距离弯曲的数值直接比较。

| 方法 | 行为 |
| --- | --- |
| `IsValid()` / `Destroy()` | 检查句柄或销毁软体；重复销毁返回 `false` |
| `GetState()` | 顶点、三角形和固定点数量，以及 `active`、`closed`、`mass`、`pressure`、`gravityFactor`、`volume`、`pos`、`boundsMin`、`boundsMax`、`simulationTime` |
| `ReadState(out)` | 向复用的表写入各项数量、`active`、`pressure`、`simulationTime` 和包围盒 `minX` … `maxZ` |
| `GetVertices([first, count])` | 新建 `{pos, velocity, mass, pinned}` 数组；每个顶点都会产生分配 |
| `GetTriangles([first, count])` | 新建索引三元组数组 |
| `ReadSnapshot(out[, skipSleeping])` | 原地写入 `out.positions`（扁平的 x、y、z 数组）和 `out.pinned`，以及 `ReadState` 的各字段和 `updated` |
| `ReadVertex(index, out)` | 原地写入 `x`、`y`、`z`、`vx`、`vy`、`vz`、`mass`、`pinned` |
| `SetVertexPinned(index, pinned)` | 固定或释放顶点；两种操作都会清零其速度，释放时恢复创建时的质量 |
| `SetVertexVelocity(index, velocity)` | 设置单个顶点的世界速度；对固定点同样有效 |
| `SetVelocity(velocity)` | 设置所有未固定顶点的世界速度 |
| `AddImpulse(impulse)` | 按质量把冲量分给所有未固定顶点，使它们获得相同的速度变化 |
| `AddVertexImpulse(index, impulse)` | 对单个未固定顶点施加冲量 |
| `SetPressure(value)` / `SetGravityFactor(value)` | 修改压力或重力倍率并唤醒软体；布料只接受压力 0 |
| `Wake()` | 唤醒软体 |
| `GetHandle()` | 以十进制字符串返回原生句柄；不要转换成数值 |
| `Trace(start, delta[, extents][, out])` | 可选。对软体表面做射线检测（传入 `extents` 时为盒形检测），返回 Source 风格的结果：`Hit`、`Fraction`、`HitPos`、`HitNormal`、`StartSolid`、`AllSolid`、`FractionLeftSolid`，以及从 1 开始的 `Triangle` 和 `Vertex`（未命中时为 0） |
| `SetPlasticity(yield, creep, maxStrain)` | 可选。永久形变，见下文 |
| `AddImpulseAt(position, impulse, radius)` | 可选。在 `radius`（0.01–4096）内按距离线性衰减，把冲量分给未固定顶点，总冲量不变 |

可选方法只在后端提供 `VJoltSoftBodyTools001` 时存在。

`Create` 成功时返回软体，失败时返回 `nil, reason`；其他调用失败时同样返回 `nil, reason`，参数类型错误也不例外；`IsValid` 和 `Destroy` 返回布尔值。垃圾回收、模块卸载和环境销毁都会释放软体，但仍应随所属实体显式销毁。所有调用都应在物理步进之外进行；物理回调中只能使用 `IsValid` 和 `Destroy`，移除会推迟到安全时机。

冲量单位为 kg·Source 单位/s。只要有一个受影响的顶点会超过 `maxVelocity`，整次调用就会失败，什么都不会改变。固定点不受力的作用，但 `SetVertexVelocity` 仍能让它移动，直到把速度设回零。

`GetState().pos` 是 Jolt 中软体的原点，会随模拟移动，并不是固定的模型原点。读取到的顶点位置是世界坐标，顺序与输入一致。快照只是副本，修改快照不会移动软体。需要反复读取时，请复用输出表，不要每帧为每个顶点分配：

```lua
local snapshot = {positions = {}, pinned = {}}
local vertex = {}
assert(body:ReadSnapshot(snapshot, false))
assert(body:ReadVertex(1, vertex))
-- 之后继续复用 snapshot、snapshot.positions、snapshot.pinned 和 vertex。
-- positions 是扁平的世界坐标数组：x1, y1, z1, x2, y2, z2, ...
```

只有在同一软体完整读取过一次之后，才能传入 `skipSleeping = true`：软体休眠时，调用照常成功，但不会改动数组，并把 `snapshot.updated` 设为 `false`。

### 壳体弯曲与塑性

`dihedralBend = true` 适合面板和壳体，不适合布料。`body:SetPlasticity(yield, creep, maxStrain)` 让形变成为永久形变：应变超过屈服值后，软体的静止形状会逐渐跟着改变。软体工坊两者都不使用，只生成布料和软球。

| 参数 | 范围 | 含义 |
| --- | --- | --- |
| `yield` | 0.001–1 | 开始产生永久形变的边应变；二面角弯曲时表示 π 弧度的比例 |
| `creep` | 0–100 | 每秒的松弛速率；0 表示关闭塑性 |
| `maxStrain` | 0–0.95 | 边长相对原长的最大永久变化；二面角弯曲时，角度限制在原角度 ±`maxStrain * pi` 弧度以内 |

```lua
assert(body.SetPlasticity, "需要提供 VJoltSoftBodyTools001 的后端")
assert(body:SetPlasticity(0.04, 30, 0.25))
```

屈服值 0.04 即 4% 的边应变，二面角弯曲时对应 7.2°。实际效果取决于网格密度、质量、碰撞和求解参数。`jolt.softbody.SupportsDihedralBend` 只表示本模块认识这个选项，较旧的后端仍可能拒绝它。

## 交互与限制

- 软体与刚体和地图双向碰撞，但不是 `IPhysicsObject`，所以物理枪、约束、Source 碰撞回调和存档都看不到它。`Create` 创建的软体没有实体，Source 射线检测也碰不到它；软体工坊为此额外创建了实体。
- 玩家控制器的接触查询会忽略软体，车轮也不能压在软体上，但车身仍会与它碰撞。
- 软体使用世界重力乘以 `gravityFactor`，不受重力场和单个物体重力设置的影响。
- 不支持软体之间的碰撞、自碰撞、撕裂、蒙皮和四面体体积约束。软体不能接入齿轮等刚体约束；可以用 `SetVertexVelocity` 移动固定点，作为会动的附着点。
- 后端上限（`GetLimits()` 也会返回）：单个软体 8192 个顶点、16384 个三角形，全局 256 个软体、共 131,072 个顶点。工坊的预算低得多；软体较多时，请降低网格精度或预算。

C++ 接口：[`public/vjolt/softbody.h`](../public/vjolt/softbody.h) 中的 `VJoltSoftBody001`，以及 [`public/vjolt/softbody_tools.h`](../public/vjolt/softbody_tools.h) 中可选的 `VJoltSoftBodyTools001`；后端实现位于 `vphysics_jolt/bridge/`。
