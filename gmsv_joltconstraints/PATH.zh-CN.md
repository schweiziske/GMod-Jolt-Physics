# 路径轨道 · PathConstraint

[项目首页](../README.zh-CN.md)

让物体沿平滑的三维曲线运动：往返、绕闭环循环、恒速行驶、移动到目标位置，或自由滑行。安装方法见[安装指南](../gmsv_jolt/INSTALL.zh-CN.md)。

## 路径轨道工具

打开 **Q 菜单 → 工具 → Jolt → 路径轨道 · PathConstraint**。

1. 冻结物体，摆好它要朝向的方向。
2. 左键依次放置节点：至少 2 个，闭合轨道至少 3 个。箭头表示正方向；节点默认离瞄准表面 8 单位。
3. 右键点击物体完成绑定。默认会把物体质心吸附到轨道上最近的位置，物体保持冻结。
4. 解冻物体。开放轨道默认在两端之间自动往返，闭合轨道默认循环；「沿轨道运动」中还可以选择恒定速度、到达目标位置，或关闭电机自由滑行。

| 操作 | 效果 |
| --- | --- |
| 左键 | 在末尾添加节点 |
| E + 左键 | 选中附近的节点；再按一次把它移到准星处 |
| Shift + 左键 | 在所选节点之后插入节点 |
| R | 删除所选节点，未选中时删除最后一个 |
| H | 开始新轨道，清空草稿和当前选择 |
| 右键物体 | 绑定物体；若它正是已选轨道上的物体，则把草稿应用到该轨道 |
| E + 右键物体 | 载入它的轨道进行编辑（仅限自己的轨道） |
| Shift + 右键物体 | 以该物体为参考系，轨道随它移动和旋转 |
| Shift + 右键地图 | 恢复为固定在世界中的轨道 |
| Z，或清理 Jolt 路径轨道 | 移除轨道约束，保留物体 |

面板中列出全部节点，可以选择、调整顺序或直接输入 XYZ 坐标。「应用轨道与全部设置」会重建约束；「只更新运动与可见性」不改动节点、参考系和旋转方式。切换工具时草稿会保留，待移动的节点则取消移动。

「暂停保持 / 恢复运动」用位置电机把物体保持在原地，因此电机最大力量必须大于 0；电机力量不足时，物体仍可能被碰撞推开。「反向运动 / 镜像目标位置」把速度取反，在目标位置模式下还会把目标 `p` 改为 `1 - p`。自由滑行只关闭驱动力，物体仍被约束在轨道上。

### 朝向与参考系

「随轨道转向」会保持绑定时物体相对轨道的朝向，所以请先摆好模型。「物体旋转」还可以设为自由旋转、只能绕切线、法线或副法线旋转，或保持相对参考物体的朝向不变。

参考物体可以冻结，也可以运动，并承受约束的反作用力。两个物体都需要通过 CanTool／CPPI 权限检查；不能使用玩家、NPC、载具、触发器和带 shadow controller 的物体。工具不会修改碰撞关系，相互重叠的部件请用 Sandbox 的「无碰撞」工具处理。

关闭质心吸附后，物体会保持与轨道之间的偏移，转弯时绕轨道摆动；偏移过大可能降低稳定性。

### 预设与复制

预设保存在本机的 `data/jolt_paths/*.json` 中，记录全部设置，以及相对第一个节点和保存时视线水平朝向的各节点位置。载入时，第一个节点放到准星处，整体按当前视线朝向加上「载入预设时额外旋转（度）」旋转；绑定物体之前不会产生任何物理效果。名称只能使用字母、数字、下划线和连字符，最长 48 个字符。

标准复制工具会把物体、轨道和参考物体一起复制。粘贴出的物体保持保存时的姿态，不会再次吸附。

## Lua API

仅限服务端。

```lua
require("jolt")
assert(jolt.constraints.PathsAvailable(), "需要支持路径的物理后端和 gmsv_joltconstraints")

local path, reason = jolt.constraints.CreatePath(prop:GetPhysicsObject(), {
    Vector(0, 0, 100), Vector(256, 0, 100), Vector(512, 128, 200),
}, {
    closed = false, snap = true, rotation = "follow", smoothing = 0.75,
    mode = "velocity", speed = 128, maxForce = 100000,
})
assert(path, reason)
prop.MyPath = path -- 必须持有引用：被垃圾回收时约束会被销毁。

local state = assert(path:GetState())
print(state.progress, state.length, state.velocity)
assert(path:SetMotor({mode = "position", target = 0.5, maxForce = 100000,
    frequency = 2, damping = 1}))
local samples = assert(path:Sample(129)) -- 世界坐标中的位置、切线和法线。
-- path:Destroy()
```

`CreatePath(followerPhys, worldNodes[, settings][, referencePhys])` 成功时返回路径对象，失败时返回 `nil, reason`。不传参考物体时轨道固定在世界中；传入时，节点（创建时的世界坐标）会随该物体一起运动。端点自动往返是工具的功能：工具运行速度电机，到达端点时把速度取反。

| 参数 | 默认值 | 范围 |
| --- | --- | --- |
| `closed` / `snap` | `false` / `true` | 布尔值 |
| `rotation` | `"follow"` | `"free"`、`"tangent"`、`"normal"`、`"binormal"`、`"follow"`、`"fixed"` |
| `smoothing` | 0.75 | 0.05–1 |
| `mode` | `"off"` | `"off"`、`"velocity"`、`"position"` |
| `speed` | 128 | ±32767 Source 单位/秒 |
| `target` | 0 | 0–1，占轨道总长度的比例 |
| `maxForce` | 100000 | 0–1e9，单位 kg·Source 单位/s²；为 0 时关闭驱动 |
| `friction` | 0 | 0–1e9，单位同上；仅在电机关闭时生效 |
| `frequency` / `damping` | 2 Hz / 1 | 0.1–20 / 0–5；仅用于位置模式 |

参数类型错误、`closed`／`snap` 不是布尔值、`rotation`／`mode` 名称未知或数值非有限时会抛出 Lua 错误；数值超出范围时返回 `nil, reason`。

`SetMotor(table)` 接受 `mode`、`speed`、`target`、`maxForce`、`friction`、`frequency` 和 `damping`，省略的键取上表默认值，所以不传 `mode` 会关闭电机。只想改一个值时，修改 `GetState().motor` 后整体传回。位置模式不受 `speed` 限制，到位过程由 `maxForce`、`frequency` 和 `damping` 决定。

`GetState()` 返回 `kind`（`"path"`）、`enabled`、`closed`、`nodeCount`、`progress`（0–1）、`distance`、`length`、`velocity` 和 `motor`。`Sample(count)` 返回 2–513 个（默认 129 个）沿轨道均匀分布的点，每个点包含世界坐标中的 `position`、`tangent` 和 `normal`。长度单位均为 Source 单位。`IsValid()` 和 `Destroy()` 返回布尔值，重复销毁是安全的。`jolt.constraints.GetPathVersion()` 返回后端接口版本。

请在物理步进之外调用这些 API；`Destroy()` 也可以在物理回调中调用。任一物体被删除、碰撞形状被替换或变为不支持的类型时，约束会被移除。

## 限制

- 每条轨道 2–32 个节点，闭合轨道至少 3 个。相邻节点（包括闭合轨道的首尾节点）至少相距 1 单位，轨道也不能几乎原路折返。
- 后端最多容纳 256 条轨道。工具默认每名玩家 8 条（`sbox_maxjolt_paths`，范围 0–64），同时计入 `constraints` 限额，且每个物理对象只能绑定一条。玩家断线时，其轨道会被移除。
- 工具的节点坐标和操作距离限制在 ±32767 单位内。API 接受的坐标上限为 ±1,048,576，轨道总长度上限为 262,144。
- 客户端最多绘制附近 24 条开启了「这条轨道对玩家可见」的轨道；`jolt_path_show 0` 可以隐藏它们，但自己的草稿始终可见。

C++ 接口：[`public/vjolt/path.h`](../public/vjolt/path.h) 中的 `VJoltPathBridge001`。
