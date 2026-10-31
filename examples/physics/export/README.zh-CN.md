# 导出一个 Jolt 功能

[项目首页](../../../README.zh-CN.md)

本例导出 Jolt 的 `GetNumBodies()` 和 `GetNumActiveBodies()`，读取指定环境中的物体总数、活跃刚体数和活跃软体数。它走的是现有 `gmsv_jolt*` 功能的同一条链路：

`Lua → gmsv_joltfeature → IVJoltBodyCounts（后端）→ 当前环境的 JPH::PhysicsSystem`

| 层 | 示例源码 | 接入位置与职责 |
| --- | --- | --- |
| 公共契约 | [body_counts.h](public/vjolt/body_counts.h) | `public/vjolt/body_counts.h`：接口名、版本、内存布局和线程约定 |
| 后端实现 | [vjolt_body_counts.cpp](backend/vjolt_body_counts.cpp) | `vphysics_jolt/bridge/vjolt_body_counts.cpp`：调用 Jolt、注册接口 |
| Lua 模块 | [main.cpp](gmsv_joltfeature/source/main.cpp) | 发现接口、检查参数、发布 Lua 函数 |
| 模块构建 | [CMakeLists.txt](gmsv_joltfeature/CMakeLists.txt) | 构建 `gmsv_joltfeature_win64.dll`（32 位为 `_win32.dll`） |
| Lua 调用 | [body_counts.lua](lua/body_counts.lua) | 在服务端读取计数 |

## 1. 定义公共接口

工厂接口名 `VJoltBodyCounts001` 与 `GetVersion() == 1` 共同构成版本契约。`Read(environment, out)` 返回是否成功，并写入一个只含定长整数的结构体；后端用结构体的 `size` 字段核对布局。调用须在环境所属线程上进行，避开物理步和接触回调。

每个新功能都应定义自己的接口。不要跨 DLL 传递 Jolt 对象、STL 容器，或需要由另一侧释放的内存。接口单例归后端所有，模块只借用它，所以它的析构函数是 protected 的。修改虚函数或数据布局必须升级版本；新增能力也可以放进单独的接口（`path.h`、`softbody_tools.h` 就是这样做的），这样已有模块不受影响。

## 2. 在后端实现并注册

把 `public/vjolt/body_counts.h` 和 `backend/vjolt_body_counts.cpp` 分别复制到仓库的 `public/vjolt/body_counts.h` 和 `vphysics_jolt/bridge/vjolt_body_counts.cpp`。

`Read` 通过 `JoltPhysicsInterface::GetInstance().GetActiveEnvironmentByIndex(index)` 取得已有环境；该环境正在模拟或处于接触回调中时直接返回失败，否则通过 `GetPhysicsSystem()` 查询 Jolt。`total` 统计物理系统中的全部物体，包括静态物体；活跃刚体和活跃软体分别计数。这些数字都不是 Source 实体数量。

文件末尾把接口注册到后端已有的 `CreateInterface` 工厂：

```cpp
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
    BodyCountsBridge, IVJoltBodyCounts, VJOLT_BODY_COUNTS_VERSION, g_BodyCounts);
```

在 [`vphysics_jolt/CMakeLists.txt`](../../../vphysics_jolt/CMakeLists.txt) 的 `set(volt_sources ...)` 中加入 `bridge/vjolt_body_counts.cpp`，它会沿用后端的 SDK、Jolt、包含目录和强制包含头文件。

然后按[构建指南](../../../docs/build.zh-CN.md#物理后端)重新构建后端。正式发布的后端不含这个示例接口，模块配合它使用时发现不了该接口。

## 3. 编写 Lua 绑定

模块在 `GMOD_MODULE_OPEN` 中调用：

```cpp
auto found = VJoltSDK::DiscoverVersionedInterface<IVJoltBodyCounts>(
    VJOLT_BODY_COUNTS_VERSION, kVJoltBodyCountsVersion);
```

加载器在已加载的后端中查找该接口并核对版本，失败原因可通过 `found.ErrorMessage()` 取得。随后模块用 `PushCFunction` 和 `SetField` 发布全局表 `joltfeature`。

`GetBodyCounts([environment])` 检查环境索引（0–4096 的整数，默认 0），调用后端，并把结果复制成 Lua 表。失败时返回 `nil, reason`；参数不是数字时抛出 Lua 错误。`Available()` 返回 `true`，或 `false, reason`。由于单例归后端所有，`GMOD_MODULE_CLOSE` 只清除借用的指针。

这个例子只读取数据，不持有任何对象。若功能会创建力场、约束或软体，还需要：

- 不会被复用的句柄（带代次或序号）；
- 显式的 `Destroy` 和 userdata 的 `__gc`，并在 `GMOD_MODULE_CLOSE` 中销毁模块仍持有的句柄；
- 物体或环境销毁时的清理；
- 物理步进行期间的延迟移除。

[`gmsv_joltconstraints`](../../../gmsv_joltconstraints/source/main.cpp) 及其[后端实现](../../../vphysics_jolt/bridge/vjolt_constraints_bridge.cpp)完整地做到了这些。

Jolt 访问留在后端，Lua 参数处理留在模块。新功能的公共头文件要写明坐标、单位、输入范围、线程和生命周期约定。不要在 Jolt 工作线程中调用 Lua 或 Source。

## 4. 构建模块

模块只依赖外部的 `garrysmod_common`（提供 GMod Lua 头文件，获取方式见[构建指南](../../../docs/build.zh-CN.md#lua-模块与-addon)）和本仓库的 `VJolt::SDK`，不需要 Source/GMod SDK，也不需要 Jolt 子模块。在仓库根目录执行：

```powershell
$GmodCommon = "C:/deps/garrysmod_common"
cmake -S examples/physics/export/gmsv_joltfeature -B .cache/joltfeature-win64 `
  -G "Visual Studio 17 2022" -A x64 "-DGMOD_COMMON=$GmodCommon"
cmake --build .cache/joltfeature-win64 --config Release
cmake --install .cache/joltfeature-win64 --config Release --prefix .cache/joltfeature-package/garrysmod
```

模块会暂存到 `.cache/joltfeature-package/garrysmod/lua/bin/gmsv_joltfeature_win64.dll`。32 位构建（实验性）使用 `-A Win32` 和单独的构建目录。如果把 `gmsv_joltfeature/` 移出本仓库，需要传入 `-DVJOLT_ROOT=<源码目录>`；除非头文件已复制到该源码目录的 `public/vjolt/`，否则还要保留同级的 `public/` 目录。

模块、后端和游戏的架构与 ABI 必须一致，详见[构建指南](../../../docs/build.zh-CN.md)。

## 5. 从 Lua 调用，按需接入总入口

安装模块和配套的后端后，在服务端执行：

```lua
require("joltfeature")
local available, reason = joltfeature.Available()
assert(available, reason)
local counts, detail = joltfeature.GetBodyCounts(0)
assert(counts, detail)
print(counts.total, counts.activeRigid, counts.activeSoft)
```

独立模块到这里就能用了。如果还想通过 `jolt.feature` 访问，在 [`jolt.lua`](../../../addons/jolt/lua/includes/modules/jolt.lua) 的 `features` 表中加入下面这一项，并在 `order` 列表中加入 `"feature"`：

```lua
feature = {module = "joltfeature", global = "joltfeature", probe = "Available"},
```

之后 `jolt.IsAvailable("feature")` 会按需加载模块，`jolt.feature.GetBodyCounts()` 即可调用。本例不会自动登记。

要与项目自带的模块一起构建，请遵循 [`gmsv_jolt/CMakeLists.txt`](../../../gmsv_jolt/CMakeLists.txt) 的约定：模块目录为仓库根目录下的 `gmsv_jolt<name>/`，目标名为 `gmsv_jolt<name>_win64` 或 `gmsv_jolt<name>_win32`，并把 `<name>` 加入 `vjolt_supported_lua_features`。正式并入项目的功能还要补充安装规则和中英文文档。

借用的接口不能在后端卸载后继续使用：先结束所有调用、卸载模块，再卸载后端。
