# 扩展功能 Bridge

[项目首页](../../README.zh-CN.md)

本目录的每个文件都基于当前 Source 物理环境已有的 Jolt 世界，实现一个[公共接口](../../public/vjolt/README.zh-CN.md)。

| 文件 | 接口 | 职责 |
| --- | --- | --- |
| `vjolt_gravity_bridge.cpp` | `VJoltGravityBridge001` | 世界与单个物体的重力；定向、吸引、排斥力场 |
| `vjolt_constraints_bridge.cpp` | `VJoltConstraints001` | 齿轮、齿条组件及其电机 |
| `vjolt_path_bridge.cpp`、`vjolt_path_spline.h` | `VJoltPathBridge001` | 轨道约束与电机，路径是按弧长参数化的样条 |
| `vjolt_softbody_bridge.cpp` | `VJoltSoftBody001`、`VJoltSoftBodyTools001` | 软体的生命周期、状态、受力和塑性 |
| `vjolt_softbody_mesh.cpp` | — | 校验网格与创建参数，生成 Jolt 共享设置 |
| `vjolt_softbody_query.cpp` | — | 软体 Trace |
| `vjolt_stats_bridge.cpp` | `VJoltStats001` | 按需开启的后端统计，没有 Lua 模块使用 |

各 `*_internal.h` 声明后端内部的物理步、物体释放与转移、环境销毁钩子。新增实现文件时，加入 [CMake](../CMakeLists.txt) 的 `volt_sources` 清单。

- 任一端物体被销毁或转移到其他环境，或所在环境被销毁时，齿轮、齿条和轨道约束都会随之移除。
- 在模拟期间或物理回调中调用 `Destroy`，实际移除会推迟到物理步中的安全时机。
- 软体没有 Source `IPhysicsObject`，由 addon 的 [`sent_jolt_softbody`](../../addons/jolt/lua/entities/sent_jolt_softbody/shared.lua) 实体提供 Trace 和玩家碰撞，不能由物理枪直接抓取。环境销毁时释放其中的软体。

原版载具、玩家、NPC 与门的影子控制器以及物理枪属于[基础后端](../README.zh-CN.md)，不在 Bridge 中实现。
