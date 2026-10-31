# VPhysics Jolt 后端代码导航

[项目首页](../README.zh-CN.md)

本目录在 Jolt 之上实现 Source 的 `IPhysics*` 接口，负责物理状态、求解调用、查询和对象生命周期。额外玩法的实现位于 [`bridge/`](bridge/README.zh-CN.md)，公共契约位于 [`public/vjolt/`](../public/vjolt/README.zh-CN.md)，Lua 绑定则是各个 `gmsv_jolt*` 模块。

## 按职责查找

| 职责 | 文件 |
| --- | --- |
| DLL 初始化、Jolt 全局资源 | [vjolt_interface.cpp](vjolt_interface.cpp) |
| CPU 变体加载、线程亲和性 | [vphysics_wrapper.cpp](../vphysics_wrapper/vphysics_wrapper.cpp) 选择后端 DLL；[cpu_policy.h](../public/vjolt/cpu_policy.h) 提供线程与 CPU 分配策略 |
| 世界、模拟步序和 Source 回调阶段 | [vjolt_environment.cpp](vjolt_environment.cpp)：创建物体、控制器和约束，`Simulate`，查询，物体转移，存档与读档，弹簧 |
| 求解期间的同步回调、延迟事件 | [vjolt_owner_callback_jobs.h](vjolt_owner_callback_jobs.h) 把同步游戏回调交回模拟调用线程；[vjolt_environment_events.h](vjolt_environment_events.h) 管理延迟事件与监听器代次 |
| 物体删除与世界销毁 | [vjolt_environment_lifecycle.cpp](vjolt_environment_lifecycle.cpp)：删除队列、重入保护、碰撞数据释放、环境内剩余物体的清理 |
| 单个 Source 物理对象 | [vjolt_object.cpp](vjolt_object.cpp)：包装 Jolt Body，换算质量、惯量、速度与冲量，管理控制器和销毁监听器 |
| 碰撞事件与接触兼容 | [vjolt_listener_contact.cpp](vjolt_listener_contact.cpp)、[vjolt_friction.cpp](vjolt_friction.cpp)：碰撞过滤、回调、接触与摩擦查询；缓存见 [vjolt_should_collide_cache.h](vjolt_should_collide_cache.h)、[vjolt_friction_contact_cache.h](vjolt_friction_contact_cache.h) |
| 玩家与影子控制器 | [vjolt_controller_player.cpp](vjolt_controller_player.cpp)、[vjolt_controller_shadow.cpp](vjolt_controller_shadow.cpp)、[vjolt_shadow_control.h](vjolt_shadow_control.h) |
| 运动、阻力、浮力与载具控制器 | `vjolt_controller_motion.cpp`、`vjolt_controller_drag.cpp`、`vjolt_controller_fluid.cpp`、`vjolt_controller_vehicle.cpp` |
| 约束与约束组 | [vjolt_constraints.cpp](vjolt_constraints.cpp)；[vjolt_havana_constraints.cpp](vjolt_havana_constraints.cpp) 用 Source/Havana 原有的方程求解固定、长度和布娃娃约束 |
| 碰撞几何与 Trace | [vjolt_collide.cpp](vjolt_collide.cpp)、[vjolt_querymodel.cpp](vjolt_querymodel.cpp) 管理几何与查询模型；[vjolt_collide_trace.cpp](vjolt_collide_trace.cpp) 的 Trace 入口调用 [vjolt_source_trace.inl](vjolt_source_trace.inl) 和 [vjolt_source_trace_solver.inl](vjolt_source_trace_solver.inl) 中的 Source 兼容实现 |
| 材质与物理 KeyValues | [vjolt_surfaceprops.cpp](vjolt_surfaceprops.cpp)、[vjolt_parse.cpp](vjolt_parse.cpp)、[vjolt_keyvalues_parser.h](vjolt_keyvalues_parser.h)、[vjolt_keyvalues_schema.cpp](vjolt_keyvalues_schema.cpp) |
| 重力场、齿轮与齿条、轨道、软体、统计 | [bridge/](bridge/README.zh-CN.md) |
| 构建与 SDK ABI 兼容 | [CMakeLists.txt](CMakeLists.txt)、[GModSDK.cmake](../cmake/GModSDK.cmake)、[gmod_convar.cmake](../cmake/gmod_convar.cmake)、[sdk_prefix.h](compat/sdk_prefix.h)、[branch_overrides.h](compat/branch_overrides.h) |

## 删除与回调约定

- `DestroyObject` 标记并入队；模拟、清理或启用删除队列期间延迟释放。`CleanupDeleteList` 不在模拟期间或嵌套清理中释放对象。
- 删除回调可能追加队列或删除其他监听器。调用前先移除当前条目，返回后重新读取容器，不跨回调持有引用或迭代器。
- 先释放约束和物体，再释放共享的 `CPhysCollide`。
- 环境销毁使用独立的 BodyID 快照；`m_CachedBodies` 会被游戏回调中的查询覆盖。清理 Source 物体前，先移除扩展物体并解除激活监听器。
- 求解期间的同步回调在模拟调用线程执行，此时 Jolt 仍持有 Body 锁。沿用环境的回调感知查询入口、`NoLock` 路径及物理修改延迟机制。

## 修改代码

- 新增或移动后端 `.cpp` 时，更新 [CMake 源文件清单](CMakeLists.txt) 中的 `volt_sources`。
- 保留 Source/IVP 兼容行为，例如 `ClampVelocityChanges` 按待提交的**线**速度大小缩放待提交的角速度。
- 构建方法见[构建指南](../docs/build.zh-CN.md#物理后端)。修改求解或接触行为后，要在游戏里用能触发相关行为的场景实际验证。
