# VJolt 公共物理接口

[项目首页](../../README.zh-CN.md)

`public/vjolt/` 是本项目自己的物理接口，不是 Source/GMod SDK：供消费者使用的头文件既不依赖 Source，也不依赖 Jolt；消费者通过它们访问正在运行的后端中的 Jolt 世界，无需再链接一份 Jolt。

| 头文件 | 接口 | 使用方 |
| --- | --- | --- |
| `gravity.h` | `VJoltGravityBridge001` | `jolt.gravity`：重力场与黑洞 |
| `constraints.h` | `VJoltConstraints001` | `jolt.constraints`：齿轮与齿条 |
| `path.h` | `VJoltPathBridge001` | `jolt.constraints`：轨道 |
| `softbody.h` | `VJoltSoftBody001` | `jolt.softbody` |
| `softbody_tools.h` | `VJoltSoftBodyTools001` | `jolt.softbody` 的可选方法 `Trace`、`SetPlasticity`、`AddImpulseAt` |
| `stats.h` | `VJoltStats001` | 仅限 C++：按需开启的物理步耗时，以及物体、约束、接触计数 |
| `loader.h` | — | 在已加载的后端中查找上述接口 |

`cpu_policy.h`（Source 模拟线程与 Jolt 工作线程的 CPU 亲和性）和 `source_abi.h`（Source SDK 内存布局检查，需要 Source SDK）供后端构建使用，消费者用不到。

## 使用头文件

```cmake
include("${VJOLT_ROOT}/cmake/VJoltSDK.cmake")  # 使用源码目录时
# 或使用已安装的 SDK：find_package(VJoltSDK CONFIG REQUIRED)
target_link_libraries(my_module PRIVATE VJolt::SDK)
```

`VJolt::SDK` 是纯头文件目标，要求 C++17。编译时使用的头文件要与实际运行的后端一致。安装 SDK 时，配置 `gmsv_jolt` 后安装其 `SDK` 组件，会写入 `include/vjolt/`、`lib/cmake/VJoltSDK/`，以及 `share/doc/VJoltSDK/` 下的开发文档。只打包公共契约时，配置参数加上 `-DVJOLT_BUILD_LUA_MODULES=OFF`，此时不需要 `garrysmod_common`。

## 发现接口与生命周期

`loader.h` 仅支持 Windows。`DiscoverVersionedInterface<T>(name, version)` 依次检查已经加载的后端（`vphysics_jolt_avx2`、`_sse42`、`_sse2`、无后缀的 `vphysics_jolt`，最后是 `vphysics.dll`），返回第一个通过 `CreateInterface` 提供 `name` 且 `GetVersion()` 匹配的后端：借用的 `api` 指针，以及模块句柄和名称；失败时返回错误码，可用 `ErrorMessage()` 取得说明。同一 DLL 中的可选扩展接口，用 `QueryInterface<T>(result.module, name)` 获取并自行核对版本，`gmsv_joltsoftbody` 就是这样做的。加载器不会加载 DLL，也不会延长后端的生命周期；后端卸载前，必须销毁自己的句柄并结束所有调用。

除非头文件另有说明：

- 在环境所属线程上调用，避开物理步和接触回调。
- 自己创建的句柄都要保存，并在实体删除、清图或代码关闭时销毁。Entity 和 `IPhysicsObject` 只是借用。
- 软体的顶点和三角形索引在 C++ 中从 0 开始，在 Lua 中从 1 开始。

用法见[物理 API](../../docs/physics-api.zh-CN.md)；新增接口请参照[功能导出示例](../../examples/physics/export/README.zh-CN.md)。
