# 物理示例

[项目首页](../../README.zh-CN.md)

| 示例 | 内容 |
| --- | --- |
| [`export/`](export/README.zh-CN.md) | 完整导出一个新的 Jolt 功能：公共接口、后端实现、`gmsv_joltfeature` 模块和 Lua 调用 |
| [`lua/attractor.lua`](lua/attractor.lua) | 在 Lua 中持有一个吸引场 |
| [`lua/cloth.lua`](lua/cloth.lua) | 创建一块布料，并读取到可复用的表中 |
| [`native/`](native/) | 在 C++ 中通过 `VJolt::SDK` 控制吸引场 |

要新增能力，从 `export/` 开始。其余示例调用的是现有功能，参数说明见[物理 API](../../docs/physics-api.zh-CN.md)。

## Lua：吸引场

把 `lua/` 中的文件复制到自己的 addon，例如 `lua/my_toy/`，然后在服务端使用：

```lua
local Attractor = include("my_toy/attractor.lua")
local toy, reason = Attractor.Create({
    center = prop:GetPos(), radius = 256, strength = 600, priority = 10,
})
assert(toy, reason)
prop.MyAttractor = toy
prop:CallOnRemove("MyAttractor", function() toy:Destroy() end)
-- 移动吸引中心：toy:SetCenter(newPosition)
-- 提前停止：toy:Destroy()
```

`prop` 是持有该吸引场的实体，实体删除时吸引场随之销毁；只靠垃圾回收的话，释放时间无法确定。`SetCenter` 复用同一张更新表。`Create` 和 `Destroy` 的错误信息原样返回；若 `Destroy` 失败而吸引场仍然有效，封装会保留它，便于重试。

## Lua：布料

```lua
local Cloth = include("my_toy/cloth.lua")
local cloth, reason = Cloth.Create({
    pos = Vector(0, 0, 128), width = 64, height = 64, vertexMass = 1,
    iterations = 5, damping = 0.1,
})
assert(cloth, reason)
prop.MyCloth = cloth
prop:CallOnRemove("MyCloth", function() cloth:Destroy() end)
local snapshot = assert(cloth:Read())
print(snapshot.positions[1], snapshot.positions[2], snapshot.positions[3])
-- 之后的 Read() 复用同一份快照及其数组，布料休眠时跳过顶点复制。
-- cloth.body:AddImpulse(Vector(0, 100, 0))
```

这是一块四个顶点、两个三角形的原生布料，上方两个顶点固定；需要更细的网格时，请自行生成更密的连通网格。本例只演示服务端的创建、读取和清理，客户端渲染与网络同步可参考 `addons/jolt/lua/jolt_softbody/`。不要在没有预算控制的情况下每帧调用 `Read`。

## C++：吸引场

`native/` 构建静态库 `vjolt_attractor`，其中的 `Attractor` 类通过 `VJolt::SDK` 创建、移动和销毁吸引场，不涉及 Lua 绑定。把它链接进自己的模块，并让 `Attractor` 实例与玩法对象同生共灭；析构时会销毁吸引场。

```cpp
Attractor toy;
VJoltGravityField field;
field.center = {0, 0, 128};
field.halfExtent = {256, 256, 256};
field.strength = 600;
const char *error;
if (!toy.Create(0, field, error)) { /* report error */ }
// toy.SetCenter({256, 0, 128});
// toy.Destroy();
```

单独构建（Windows、MSVC）：

```powershell
cmake -S examples/physics/native -B .cache/physics-consumer -A x64
cmake --build .cache/physics-consumer --config Release
```

该库使用静态 CRT（`/MT`）。通过 `add_subdirectory` 加入你的项目时，它会复用已有的 `VJolt::SDK` 目标；否则，位于本仓库内时直接包含 `cmake/VJoltSDK.cmake`，位于仓库外（例如随 SDK 安装的副本）时改用 `find_package(VJoltSDK CONFIG REQUIRED)`，此时需把 `CMAKE_PREFIX_PATH` 设为 SDK 的安装前缀。`Attractor` 须在环境所属线程上、物理步之外使用，并在后端卸载前销毁。
