# 软体客户端

[项目首页](../../../README.zh-CN.md)

本文说明软体工坊如何把软体同步到客户端并绘制出来。操作、设置和 Lua API 见[软体指南](../../../gmsv_joltsoftbody/README.zh-CN.md)。客户端只运行 addon 的 Lua，原生模块只在服务端加载。

## 同步

- **采样**：服务端每秒最多 15 次用 `ReadSnapshot` 读取每个软体，休眠的软体最多每秒完整读取一次。运动中的软体每次采样都生成新帧，休眠的软体只在量化后的姿态或固定点变化时才生成。此外，服务端每个 Tick 都用 `ReadState` 刷新实体的碰撞包围盒，让射线检测能跟上快速运动的软体。
- **关注范围**：服务端每隔 1/`jolt_softbody_interest_rate` 秒为每名玩家重新判断一次：软体距玩家视点不超过 `jolt_softbody_view_distance` 且通过 `TestPVS` 时才算可见；在 `jolt_softbody_near_distance` 以内按 `jolt_softbody_near_rate` 发送，以外按 `jolt_softbody_far_rate` 发送。软体移出范围时发送移除消息，重新进入时再次发送元数据。
- **带宽预算**：每名玩家有一个令牌桶，以 `jolt_softbody_net_kib` KiB/s 的速度补充，最多积攒半秒的额度。每帧的开销是数据本身加 96 字节，首次发送时再加 18 字节元数据。各软体轮流发送；额度不够发送下一个软体时，调度器会停在它身上等额度攒够，小网格因此不会长期挤占大网格的带宽。
- **可靠性**：元数据、玩家收到的某个软体的第一帧，以及最后的休眠姿态使用可靠消息；其余帧使用不可靠消息，允许丢失。
- **加入与重载**：客户端最多发送四次就绪消息，每次间隔 3 秒。服务端收到后清空该玩家的接收记录，重新发送全部内容。

## 消息格式

| 消息 | 内容 |
| --- | --- |
| `jolt_softbody_meta_v1` | 协议版本、软体 ID、实体索引、形状、精度、宽度、高度、颜色、固定点方案和玩家碰撞开关。客户端据此重新生成拓扑，所以帧中只传位置。 |
| `jolt_softbody_frame_v1` | 软体 ID（32 位）、序号（16 位）、服务端时间、中心点（3 个 float）、缩放（float）、顶点数（16 位）、活动标志，随后每个顶点三个有符号 16 位坐标和一个固定位，共 225 + 49 × 顶点数 位 |
| `jolt_softbody_remove_v1` | 软体 ID |

坐标按 `center + q * scale` 解码，其中 `scale = max(1/16, halfSize / 32760, maxCoord / 8388608)`；`halfSize` 是包围盒最长边的一半，`maxCoord` 是中心点坐标绝对值的最大值。量化步长通常为 1/16 单位；软体特别大或远离地图原点、float 精度不够时，步长会随之增大。

## 插值

- 每个软体保留四个快照的环形缓冲。只接受序号更新的帧（正确处理 16 位序号回绕），迟到和重复的帧会被丢弃。服务端时间间隔超过 0.75 秒时，缓冲和时钟都会重置。
- 收到第一帧时，把服务端时间对应到客户端的游戏时间（`CurTime()`），之后按游戏时间的增量推进，因此暂停和时间缩放都能正常生效。播放速度在 ±5% 内微调以跟上漂移，而不是直接跳变。
- 延迟为 `max(100 ms, 1.4 × 收包间隔 + 2 × 抖动)`，并且不超过 350 ms 和三个收包间隔：15 Hz 时约为 100 ms。
- 播放进度不会越过最新的快照，数据中断时停在最后一个姿态。不做外推，延迟变大时也不会倒退，也没有本地物理预测。

## 绘制

- 运动中的软体每帧用动态 `mesh.Begin(MATERIAL_TRIANGLES, ...)` 绘制。休眠且插值稳定后，姿态写入一份静态 `IMesh`，软体被唤醒前一直绘制这份网格；下次休眠时改写同一份网格。软体停止同步、被删除或 Lua 重载时，这份网格会被显式销毁。
- 材质为 `UnlitGeneric`，使用顶点颜色和一个 UV 通道。光照由平滑法线和 `render.GetLightColor` 计算后烘焙进顶点颜色，软体运动时每 0.5 秒采样一次。这样可以避开 `IMesh` 在 `VertexLitGeneric` 下的光照差异，也不需要切线、法线贴图、PBR、计算着色器或额外的渲染目标。
- 表面保持不透明。不要启用 `$vertexalpha`：即使 alpha 为 255，它也会让 `UnlitGeneric` 不写入深度。创建预览和网格线使用另一种半透明线框材质。
- 每个软体最多 3840 个渲染角点（1280 个三角形），远低于 mesh API 的 65535 顶点上限。
- 创建预览只在形状、精度、尺寸、颜色或固定点方案变化时重建；移动预览只更新一个复用的 `Matrix`。

## 内存分配与剔除

- `ReadSnapshot` 和 `ReadVertex` 直接写入调用方提供的表，常规同步不会为每个顶点新建表或 Vector。渲染缓存、法线、量化数组、接收缓冲和发送名单都会复用，数值循环保持可被 LuaJIT 跟踪。addon 从不停止、强制执行或调整垃圾回收；眼睛位置、光照等引擎接口仍会产生少量分配。
- 包围盒在 `Think` 而不是 `Draw` 中更新，这样被剔除的软体还能重新进入视野。距离剔除使用 `LocalPlayer():EyePos()`，因为全局的 `EyePos()` 描述的是当前渲染过程，在 `Think` 中可能为零。

## 客户端碰撞

在客户端，实体的 `TestCollision` 针对正在显示的网格做射线检测，因此移动预测与玩家看到的一致；服务端则检测原生软体。拓扑不会变化，所以每个软体只构建一次包围体层次树，显示姿态变化时重新拟合即可。`jolt_softbody_collision_profile 1` 会记录射线检测耗时，供 `jolt_softbody_client_stats` 输出。

## 诊断

- `jolt_softbody_server_stats`（管理员或服务端控制台）：软体数、顶点数、已连接玩家数、已发送的帧数和数据量（KiB）。
- `jolt_softbody_client_stats`：软体数、顶点数、帧数、接收量（KiB）和被拒绝的数据包数；DX9 网格数、静态写入次数和动态绘制次数；碰撞耗时。其中的 Lua 堆大小统计的是整个客户端 Lua 状态，不只是软体。

## 代码分工

脚本位于 `addons/jolt/lua/`，由 `autorun/jolt_softbody.lua` 加载。

| 文件 | 职责 |
| --- | --- |
| `jolt_softbody/config.lua` | 限额、预设、ConVar 定义、带宽估算 |
| `jolt_softbody/geometry.lua` | 布料与二十面体细分球的生成、放置、法线、射线检测 |
| `jolt_softbody/buffer.lua` | 可复用的快照缓冲、四槽环形缓冲、播放时钟 |
| `jolt_softbody/protocol.lua` | 量化与消息格式 |
| `jolt_softbody/server.lua` | 原生软体、实体生命周期、采样、玩家推动、服务端设置 |
| `jolt_softbody/replication.lua` | 关注范围、中途加入、带宽调度、`jolt_softbody_server_stats` |
| `jolt_softbody/editing.lua` | 编辑权限、选点、固定、拖动 |
| `jolt_softbody/client.lua` | 接收、实体绑定、插值、剔除、`jolt_softbody_client_stats` |
| `jolt_softbody/renderer.lua` | 顶点光照与 DX9 网格生命周期 |
| `jolt_softbody/collision.lua`、`collision_tree.lua` | 客户端针对显示网格的射线检测 |
| `jolt_softbody/tool_client.lua`、`panel.lua` | 工具预览、顶点标记、HUD、工具面板 |
| `jolt_softbody/options.lua` | **Q 菜单 → 选项 → Jolt → 软体**页面 |
| `entities/sent_jolt_softbody/` | 实体：射线检测、伤害冲量、绘制 |
| `weapons/gmod_tool/stools/jolt_softbody.lua` | Sandbox 工具 |

只有工坊预设的布料网格和二十面体细分球会被同步，因为客户端要根据元数据重建它们。直接用 `jolt.softbody.Create` 创建的软体既不会同步，也不会被绘制。
