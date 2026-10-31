# 物理 API

[项目首页](../README.zh-CN.md)

从这里开始，可以在 Lua 或 C++ 中调用额外的物理功能，也可以自己导出新的 Jolt 能力。

## 导出新的 Jolt 功能

要开发类似 `gmsv_joltgravity`、`gmsv_joltconstraints` 的功能，请参照[功能导出示例](../examples/physics/export/README.zh-CN.md)。示例把 Jolt 的物体计数逐层导出到 Lua：

1. 在 `public/vjolt/` 定义带版本号的接口及跨 DLL 数据结构。
2. 在 `vphysics_jolt/bridge/` 中基于当前环境的 `JPH::PhysicsSystem` 实现接口，再用 `EXPOSE_SINGLE_INTERFACE_GLOBALVAR` 注册到后端的 `CreateInterface` 工厂。
3. 把源文件加入后端的 CMake 源文件清单。
4. 在自己的 `gmsv_jolt*` 模块中发现并校验接口，然后绑定 Lua 函数、参数检查和资源清理。
5. 按需在 `jolt` Lua 总入口登记命名空间。后端和模块要一起构建、配套发布。

Jolt 对象始终留在后端，模块只借用后端提供的接口。

## 调用现有功能

原生功能只在服务端运行。Lua 代码需要服务端装好后端、对应功能的 `gmsv_jolt*` 模块和 `addons/jolt`；C++ 代码只依赖后端，链接 `VJolt::SDK` 后在运行时发现接口即可。

`require("jolt")` 本身不加载原生模块。`jolt.IsAvailable(name)` 加载指定功能并检查后端接口，返回 `true`，或 `false, reason`；`jolt.Load(name)` 返回功能表，或 `nil, reason`；直接读取 `jolt.<name>` 同样会加载功能，但不可用时抛出 Lua 错误。客户端只能用 `jolt.ListFeatures()` 列出功能。

位置使用 Source 单位，质量使用 kg，加速度使用 Source 单位/s²。各接口的线程、单位和生命周期约定写在 `public/vjolt/` 对应的头文件中。下文示例中的 `prop` 指由你的 addon 控制的实体。

### Lua：吸引场

```lua
require("jolt")
local available, reason = jolt.IsAvailable("gravity")
assert(available, reason)

local field = assert(jolt.gravity.CreateField({
    center = Vector(0, 0, 128), radius = 256,
    shape = 0, mode = 1, -- 球形、吸引
    strength = 600, priority = 10,
}))
-- 必须保存引用：没有引用的力场会被垃圾回收销毁。
assert(field:Update({center = Vector(256, 0, 128)}))
-- 用完后调用 field:Destroy()。
```

黑洞就是在这种吸引场上实现的，计时、视觉效果、权限和网络同步由 addon 负责。力场只作用于动态刚体，不影响软体、玩家移动和 NPC AI。[`attractor.lua`](../examples/physics/lua/attractor.lua) 封装了持有与清理逻辑。

### Lua：把物体放到轨道上

```lua
require("jolt")
assert(jolt.constraints.PathsAvailable())
local path, reason = jolt.constraints.CreatePath(prop:GetPhysicsObject(), {
    Vector(0, 0, 100), Vector(256, 0, 100), Vector(512, 128, 200),
}, {
    mode = "velocity", speed = 128, maxForce = 100000,
    rotation = "follow", snap = true,
})
assert(path, reason)
prop.MyJoltPath = path
prop:CallOnRemove("MyJoltPath", function()
    path:Destroy()
end)
-- 创建轨道不会解冻 prop。path:Destroy() 只移除轨道，保留 prop。
```

## 可用功能

| 功能 | Lua 命名空间 | Lua 文档 | C++ 头文件 |
| --- | --- | --- | --- |
| 世界与单个物体的重力；定向、吸引、排斥力场 | `jolt.gravity` | [重力](../gmsv_joltgravity/README.zh-CN.md) | `vjolt/gravity.h` |
| 齿轮、齿条及其电机 | `jolt.constraints` | [齿轮与齿条](../gmsv_joltconstraints/README.zh-CN.md) | `vjolt/constraints.h` |
| 沿曲线运动的轨道及电机 | `jolt.constraints` | [轨道](../gmsv_joltconstraints/PATH.zh-CN.md) | `vjolt/path.h` |
| 布料、封闭充气网格 | `jolt.softbody` | [软体](../gmsv_joltsoftbody/README.zh-CN.md) | `vjolt/softbody.h`、`vjolt/softbody_tools.h` |

## 软体

`jolt.softbody.Create(vertices, triangles[, settings])` 成功时返回软体 userdata，失败时返回 `nil, reason`。顶点位置相对于 `settings.pos` 和 `settings.angles`，三角形索引从 1 开始。

```lua
require("jolt")
assert(jolt.IsAvailable("softbody"))
local body, reason = jolt.softbody.Create({
    {pos = Vector(-32, 0, 0), mass = 1, pinned = true},
    {pos = Vector(32, 0, 0), mass = 1, pinned = true},
    Vector(-32, 0, -64), Vector(32, 0, -64),
}, {{1, 3, 2}, {2, 3, 4}}, {
    pos = Vector(0, 0, 128), iterations = 5, pressure = 0,
})
assert(body, reason)
prop.MySoftBody = body
prop:CallOnRemove("MySoftBody", function() body:Destroy() end)
local snapshot = {positions = {}, pinned = {}}
assert(body:ReadSnapshot(snapshot, false)) -- 完整读取，软体休眠时也复制
-- 之后调用 body:ReadSnapshot(snapshot, true)：复用数组，软体休眠时跳过复制。
-- positions 是世界坐标的扁平数组：x1, y1, z1, x2, y2, z2, ...
```

直接写成 `Vector` 的顶点质量为 1、不固定。网格必须连通且绕序一致；`pressure > 0` 要求朝外绕序的封闭网格，布料保持 0。

| 创建参数 | 含义 |
| --- | --- |
| `pos`、`angles`、`velocity` | 初始世界位置、`Angle` 朝向、初始世界速度 |
| `environment` | 物理环境索引，默认 0 |
| `iterations` | 求解迭代次数，1–32 |
| `edgeCompliance`、`bendCompliance` | 边与弯曲柔度，越小越硬 |
| `pressure` | 气体能量 nRT，单位 kg·Source-unit²/s²，不是帕斯卡 |
| `gravityFactor`、`damping` | 重力倍率、线性阻尼 |
| `friction`、`restitution` | 摩擦、回弹 |
| `vertexRadius`、`maxVelocity` | 顶点碰撞半径、顶点速度上限 |
| `allowSleeping`、`doubleSided`、`dihedralBend` | 开关：允许休眠、双面三角形、二面角弯曲 |

省略的参数取 `softbody.h` 中的默认值，所有取值都由后端校验。`jolt.softbody.GetLimits()` 返回原生上限：每个软体 8192 个顶点、16384 个三角形，全局最多 256 个软体、共 131072 个顶点。项目自带的 Sandbox 工具另用 ConVar 设定了更低的限额。

| 方法分组 | 方法 |
| --- | --- |
| 状态与清理 | `IsValid()`、`Destroy()`、`GetState()`、`GetHandle()` |
| 写入可复用的表 | `ReadState(out)`、`ReadSnapshot(out, skipSleeping)`、`ReadVertex(index, out)` |
| 每次分配新表 | `GetVertices([first, count])`、`GetTriangles([first, count])` |
| 单个顶点 | `SetVertexPinned(index, pinned)`、`SetVertexVelocity(index, velocity)`、`AddVertexImpulse(index, impulse)` |
| 整个软体 | `SetVelocity(velocity)`、`AddImpulse(impulse)`、`SetPressure(value)`、`SetGravityFactor(value)`、`Wake()` |
| 仅当后端提供 `VJoltSoftBodyTools001` 时可用 | `Trace(start, delta[, extents][, out])`、`SetPlasticity(yield, creep, maxStrain)`、`AddImpulseAt(position, impulse, radius)` |

`Create` 不会生成渲染、网络同步或 Source 玩家碰撞。可复用读取的写法见 [`cloth.lua`](../examples/physics/lua/cloth.lua)，完整的呈现方式见项目自带的软体 addon。默认值、取值范围、单位和错误行为见[软体文档](../gmsv_joltsoftbody/README.zh-CN.md)。

## C++：接入正在运行的后端

```cmake
include("${VJOLT_ROOT}/cmake/VJoltSDK.cmake")  # VJOLT_ROOT：GMod-Jolt-Physics 源码目录
# 使用已安装的 SDK 时改为：find_package(VJoltSDK CONFIG REQUIRED)
target_link_libraries(my_toy PRIVATE VJolt::SDK)
```

```cpp
#include <vjolt/gravity.h>
#include <vjolt/loader.h>

auto bridge = VJoltSDK::DiscoverVersionedInterface<IVJoltGravityBridge>(
    VJOLT_GRAVITY_BRIDGE_VERSION, kVJoltGravityBridgeVersion);
if (!bridge) {
    // bridge.ErrorMessage() 给出失败原因。
    return;
}
VJoltGravityField settings;
settings.mode = VJoltGravityAttract;
settings.center = {0, 0, 128};
settings.halfExtent = {256, 256, 256};
settings.strength = 600;
const auto field = bridge.api->CreateField(0, settings);
// 返回 0 表示失败。成功后保存句柄，用完调用 bridge.api->DestroyField(field)。
```

`VJolt::SDK` 是纯头文件目标，要求 C++17。`loader.h` 仅支持 Windows：它在已加载的后端中查找接口，核对接口名和版本，返回借用的指针；不会加载 DLL、创建世界或链接 Jolt。后端卸载前，必须销毁自己持有的句柄并结束所有调用。完整的 C++ 消费者示例见 [`native/`](../examples/physics/native/)，全部接口见[公共接口说明](../public/vjolt/README.zh-CN.md)。

## 持有与清理

- 在拥有该物理环境的服务端线程（即运行 Lua 的线程）上、物理步之外调用。`Destroy` 也可以在物理回调中调用，实际移除会推迟到安全时机；其他例外见各头文件。
- 自己创建的力场、机械、轨道和软体都要保存引用，并在实体删除、清图或功能关闭时销毁。Entity 和 PhysObj 只是借用，使用前先检查有效性。
- C++ 句柄是带代次或序号的 64 位值，过期句柄只会调用失败，不会误指向新对象；返回 0 表示创建失败。Lua 持有的则是模块返回的 userdata。重力场的 `id`、软体 `GetHandle()` 等字符串 ID 不要转换成数字。
- 软体是独立的原生对象，没有 `IPhysicsObject`。顶点和三角形索引在 C++ 中从 0 开始，在 Lua 中从 1 开始。读取快照时复用表和缓冲区，并控制读取频率。
