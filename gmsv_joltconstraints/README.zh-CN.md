# Jolt 齿轮与齿条

[项目首页](../README.zh-CN.md)

让两个物体按传动比联动，或把小齿轮的转动变成齿条的平移。每个机构自带转轴约束（齿条还带滑轨约束），无需另加 Axis 或 Slider。安装方法见[安装指南](../gmsv_jolt/INSTALL.zh-CN.md)，轨道见[路径轨道指南](PATH.zh-CN.md)。

## 齿轮 / 齿条工具

打开 **Q 菜单 → 工具 → Jolt → 齿轮 / 齿条**。

1. 冻结并摆好两个物体，在「组件类型」中选择「双齿轮」或「齿轮齿条」。
2. 依次左键点击两个物体的端面。点中表面的法线决定该物体的转轴，或齿条的滑动方向。转轴穿过各自的质心，并固定在世界中。
3. 解冻物体，机构开始运转。

| 操作 | 效果 |
| --- | --- |
| 右键 | 把面板中的电机设置应用到自己最近创建的机构 |
| R | 取消选择 |
| Z | 撤销机构，保留物体 |

工具不会修改碰撞关系；两个模型相互重叠时，请用 Sandbox 的「无碰撞」工具处理。每个机构都计入 `sbox_maxjolt_mechanisms`（默认 32，范围 0–256）和标准的 `constraints` 限额。复制工具（duplicator）不会复制机构。

## Lua API

以下代码在服务端运行，`firstProp` 和 `secondProp` 是已存在的物理道具。

```lua
require("jolt")
local available, reason = jolt.IsAvailable("constraints")
assert(available, reason)

local gears, detail = jolt.constraints.CreateGear(
	firstProp:GetPhysicsObject(),
	secondProp:GetPhysicsObject(),
	{
		axis1 = Vector(0, 0, 1),
		axis2 = Vector(0, 0, 1),
		ratio = 2,
	}
)
assert(gears, detail)
assert(gears:SetMotor(180, 1000000))
firstProp.JoltMechanism = gears -- 必须持有引用：被垃圾回收时机构会被销毁。

local state = assert(gears:GetState())
print(state.angularVelocity1, state.angularVelocity2) -- 无负载时约为 180 和 -90 度/秒。

-- 用完后：
-- gears:Destroy()
-- firstProp.JoltMechanism = nil
```

创建齿轮齿条时，第一个物体是小齿轮，第二个物体沿 `axis2` 滑动：

```lua
local rack, reason = jolt.constraints.CreateRackAndPinion(
	pinionProp:GetPhysicsObject(),
	rackProp:GetPhysicsObject(),
	{
		axis1 = Vector(0, 0, 1),
		axis2 = Vector(1, 0, 0),
		travelPerRevolution = 128,
		minPosition = -256,
		maxPosition = 256,
	}
)
assert(rack, reason)
assert(rack:SetMotor(360, 1000000))
pinionProp.JoltMechanism = rack
```

小齿轮每秒转一圈时，齿条以 128 Source 单位/秒移动，到达任一端的限位后停止。

`CreateGear(phys1, phys2, settings)` 和 `CreateRackAndPinion(phys1, phys2, settings)` 成功时返回机构，失败时返回 `nil, reason`。`settings` 必须是表，省略的键取下表中的默认值。参数类型错误或 `settings` 中出现非有限数值时会抛出 Lua 错误。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `axis1` | `Vector(0, 0, 1)` | 第一个物体在世界坐标中的转轴 |
| `axis2` | 齿轮：`Vector(0, 0, 1)`；齿条：`Vector(1, 0, 0)` | 第二个齿轮的世界转轴，或齿条的滑动方向 |
| `anchor1` / `anchor2` | 质心 | 世界坐标中的安装点 |
| `ratio` | 1 | 仅齿轮：`angle1 + ratio * angle2 = 0`。绝对值 0.001–1000。 |
| `travelPerRevolution` | 128 | 仅齿条：小齿轮正转一圈时齿条移动的有符号距离，单位 Source 单位。绝对值至少 0.01；取负值时方向相反。 |
| `minPosition` / `maxPosition` | −128 / 128 | 仅齿条：相对起始位置的行程限位。范围必须包含 0，且下限小于上限。 |

转轴会自动归一化，不能为零向量。向量分量和距离必须是有限值，绝对值不超过 1,048,576。

两个齿轮相互啮合时 `ratio = teeth2 / teeth1`，因此 `speed2 = -speed1 / ratio`。传动比为正时，两个齿轮绕各自转轴反向转动；为负时同向转动。无论正负，小数传动比转过整圈后相位都不会错开。

| 方法 | 行为 |
| --- | --- |
| `IsValid()` | 机构是否仍然存在 |
| `Destroy()` | 一并删除机构的全部约束。返回 `true`；机构已不存在时返回 `false`。 |
| `SetMotor(speed, maxTorque)` | 驱动第一个物体。成功返回 `true`，失败返回 `nil, reason`。 |
| `GetState()` | 返回状态表，失败时返回 `nil, reason` |

电机转速单位为度/秒，范围 ±3600。扭矩单位为 kg·Source 单位²/s²，范围 0–1,000,000,000。转速为 0、扭矩不为 0 时起制动作用；扭矩为 0 时关闭电机，但传动关系仍然保留。

状态表包含 `kind`（`"gear"` 或 `"rack_and_pinion"`）、`angle1`、`angle2`、`position`、`angularVelocity1`、`angularVelocity2`、`linearVelocity`、`motorSpeed` 和 `maxTorque`。角度单位为度，范围 ±180；角速度单位为度/秒。`position` 和 `linearVelocity` 的单位分别是 Source 单位和 Source 单位/秒。`angle2` 和 `angularVelocity2` 只对齿轮有意义，`position` 和 `linearVelocity` 只对齿条有意义。

## 限制

- 两个物体必须不同，且处于同一物理环境。不支持静态物体、触发器、流体、由游戏控制的物体和带 shadow controller 的物体。
- 冻结的物体也可以安装。创建机构不会解冻物体，解冻后机构才开始运动。
- 转轴固定在世界中，适合原地运转的机械。不支持移动底座和车辆传动系统。
- 创建机构、设置电机和读取状态都必须在物理步进之外进行。`Destroy()` 也可以在物理回调中调用，约束会在下一个安全时机移除。
- 删除任一物体、把它转移到其他物理环境或销毁环境，都会移除整个机构。物体的碰撞形状被替换或变为不支持的类型时，机构会在下一次物理步进时被移除。
- 后端最多容纳 4096 个机构。

C++ 接口：[`public/vjolt/constraints.h`](../public/vjolt/constraints.h) 中的 `VJoltConstraints001`。它依赖本仓库 Jolt 分支中的齿轮修复。
