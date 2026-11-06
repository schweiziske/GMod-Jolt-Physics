# 与上游的区别

[项目首页](../README.zh-CN.md)

本文说明 GMod Jolt Physics 的来源，以及它在 RaphaelIT7 版 VPhysics-Jolt 的基础上做了哪些改动。

## 项目来源

GMod Jolt Physics 的代码源自 [RaphaelIT7/VPhysics-Jolt](https://github.com/RaphaelIT7/VPhysics-Jolt)，后者是 [Volt（VPhysics-Jolt）](https://github.com/misyltoad/VPhysics-Jolt) 的 fork。

本仓库不是 GitHub 上的 fork，也不包含上游的提交历史：开发过程中的改动范围广、提交记录杂乱，公开时重新建立了提交历史。Volt 与 RaphaelIT7 版贡献者的提交记录请查阅上述上游仓库。上游的许可证随源码保留在 [LICENSES](../LICENSES/Volt-MIT.txt) 中。

定制 Jolt 基于 Jorrit Rouwe 的 [JoltPhysics](https://github.com/jrouwe/JoltPhysics)，保留了上游的提交历史和贡献者记录，本项目的改动在此基础上继续提交。

Volt 与 RaphaelIT7 版已经提供了 VPhysics 接口适配、基础刚体、碰撞查询、约束、玩家与影子控制器、车辆接口、触发器和多线程模拟。本项目的工作建立在这些基础之上，主要集中在三个方面：

- 让已有物理接口保持 Source/IVP 的参数含义和执行时序。
- 让兼容约束、摩擦与 Jolt 的接触求解协同工作。
- 通过公共接口、Lua 模块和 Sandbox 工具提供额外的物理能力。

## 比较范围

| 对象 | 版本 |
| --- | --- |
| 本项目 | [`b61edbe`](https://github.com/schweiziske/GMod-Jolt-Physics/tree/b61edbe2d8849c9312cf63f4b9d93ad055fc9159) |
| 定制 Jolt 子模块 | [`bca5609`](https://github.com/schweiziske/JoltPhysics-GMod/tree/bca5609eaebf54e48340229ab22efafec0af7b1c) |
| 基线（RaphaelIT7 版 `fixed` 分支） | [`cfabd9b`](https://github.com/RaphaelIT7/VPhysics-Jolt/tree/cfabd9b3dcff90227ff2f342e8970fc89f2a638e) |

下文只描述这几个版本之间的差异，不代表上游之后的版本。

## Source/GMod 物理行为兼容

| 领域 | 基线 | 本项目 | 对游戏行为的影响 |
| --- | --- | --- | --- |
| 模拟时间 | 在外部调用中运行控制器，再调用 Jolt 更新及碰撞子步 | 累积时间并执行完整物理步，每步依次运行控制器、提交速度、求解；`SetSimulationTimestep` 决定实际步长 | 游戏输入、控制器与物理响应的节奏一致 |
| 控制器顺序 | 沿用原有更新流程 | 按优先级和稳定顺序调度，并处理回调期间的挂接、解绑和删除 | 减少多个控制器共同作用时的顺序差异 |
| 质量与惯量 | 主要按 Jolt 形状计算惯量 | 优先采用 Source 碰撞模型中的质心与单位质量惯量，并处理惯量倍率和旋转惯量限制 | 旋转、偏心冲量和约束响应以原版数据为依据 |
| 施力与速度 | 将接口操作直接映射到 Jolt | 暂存待提交的线速度与局部角速度变化，区分读取、立即设置、累积与提交 | 保留 Source 中多次施力及控制器输出的叠加语义 |
| 材质弹性 | 材质弹性的乘积直接作为恢复系数 | 将 IVP 的能量保留比例换算为速度恢复系数，并解析接触面的材质 | 投掷和落地反弹的参数含义与原版一致 |
| 摩擦快照 | 通过附近几何查询构造接触信息 | 读取已发布的接触与求解状态，保存法向冲量、压力、接触点及相关历史 | 游戏层查询到的是实际发生的接触和承重 |
| 弹簧 | 软距离约束 | 按 IVP 执行器处理自然长度、刚度、轴向及相对阻尼和只拉伸，统一采样后施力 | 保留弹簧参数的受力含义，并减少遍历顺序的影响 |
| 玩家物理 | 额外创建 Character，再同步到公开物理对象 | 直接驱动公开动态刚体，处理移动平台、相对速度、推物与接触限制 | 游戏查询的对象就是实际受力的对象 |
| Shadow 控制 | 主要通过运动学移动或直接设置变换 | 动态路径按误差、到达时间、速度上限及阻尼驱动，共用同一套 Shadow 计算 | 改善受控物体与周围物体之间的负载传递 |
| 碰撞回调 | 更新结束后补发 Pre/Post 事件 | 协调 PreCollision、回调改速、实际响应和 PostCollision 的顺序，结构变更延迟到安全时机执行 | 回调中的修改能影响对应的碰撞响应 |
| 物理 Trace | 基于 Jolt 查询实现 | 保留 Source 查询几何，通过兼容的 sweep/simplex 路径计算命中比例、法线和重叠状态 | 支撑台阶判断、离开接触面和工具命中等游戏逻辑 |

代码：[环境与步序](../vphysics_jolt/vjolt_environment.cpp)、[物体与待提交速度](../vphysics_jolt/vjolt_object.cpp)、[接触材质](../vphysics_jolt/vjolt_listener_contact.h)、[摩擦快照](../vphysics_jolt/vjolt_friction.cpp)、[玩家控制器](../vphysics_jolt/vjolt_controller_player.cpp)、[Shadow 控制](../vphysics_jolt/vjolt_shadow_control.h)、[Trace](../vphysics_jolt/vjolt_collide_trace.cpp)。

### 浮力、水阻与流体中的旋转

基线通过 Jolt 的 `ApplyBuoyancyImpulse` 统一处理浮力与阻力。本项目保留 Jolt 的浸没体积查询，其余受力改为 Source 风格的计算，影响物体入水、漂浮姿态、翻滚、水流推动和水中减速：

- 基于表面三角形计算部分浸没表面的压力、摩擦与力矩；球体单独按投影面积计算。
- 由浸没体积、流体密度、物体浮力倍率和重力得出浮力，再由浮心相对质心的位置产生力矩。
- 根据表面相对流速、迎流面和角速度计算响应。物体上一步的运动对周围流体保留有限影响，浸没面积增加时这部分影响会被稀释。
- 流速按 Source 接口的方向转换，不当作位置或距离缩放；施力使用对应的物理步长。

代码：[流体控制器](../vphysics_jolt/vjolt_controller_fluid.cpp)。

### 原版车辆接口

这里指 Source 原有的车辆接口，与额外的车辆玩法模块无关。基线已经实现轮式车和气垫船，本项目补充了参数和运行时行为：

- 动力配置按车轴扭矩分配、每轴轮数及 Jolt 差速器约束转换。
- `SetWheelFriction` 以构造时的摩擦曲线为基准设置绝对倍率，重复调用不会累乘。
- 轮胎接触材质从对应的接触对象及子形状解析。
- 气垫船区分设置的弹簧长度与运行时射线长度，并补充浮筒参数和调试查询。
- 车辆恢复时保存必要的配置和状态并重建接触，不沿用旧物理世界的 BodyID 和预热冲量。

代码：[车辆控制器](../vphysics_jolt/vjolt_controller_vehicle.cpp)、[车辆约束](../vphysics_jolt/vjolt_vehicle_constraint.h)。

### 冻结、质量、惯量和球体半径

物理对象在运行时被修改，也要符合 Source 的语义：

- 冻结与解冻按 IVP 的 pinned 行为处理速度及待提交变化，并保留普通惯量供恢复运动时使用。
- `SetMass` 按质量比例缩放已有惯量，并正确更新冻结状态下的质量、逆质量与逆惯量。
- `SetInertia` 换算 Source 物体坐标轴与 Jolt 惯量表示，主轴重排不会改变接口含义。
- 修改球体半径时替换不可变的 Jolt 形状，保留原有质量、惯量和速度，并处理 Jolt 不接受的半径输入。
- `GetEnergy` 将平移动能与转动动能求和，考虑主惯量坐标和锁定轴，最后换回 Source 单位。

代码：[物体实现](../vphysics_jolt/vjolt_object.cpp)。

### 接触响应、承重与玩家碰撞

上表的摩擦快照针对游戏读取的结果，求解一侧还有以下适配：

- 按接触对象和约束状态选择原生或兼容接触路径，区分初次撞击与持续承重。
- 兼容摩擦维护切向滑移历史、切平面投影、耦合有效质量和压力限制。
- 对符合条件的自由刚体承重接触，限制求解时的有效质量比，让重物压轻物更稳定；物体的真实质量不变。
- 玩家接触区分头顶、侧面、地面和既有负载的响应，保留允许的切向运动和离开表面的运动。
- 正确处理已休眠接触的保留、物体删除后的失效、接触锚点和带序号的 BodyID，游戏不会读到失效对象。

这些稳定化策略有适用条件，并不保证在任意质量比或几何下都与原版等价。

代码：[接触响应](../vphysics_jolt/vjolt_listener_contact.h)、[接触缓存](../vphysics_jolt/vjolt_friction_contact_cache.h)、[玩家接触平面](../vphysics_jolt/vjolt_player_contact_planes.h)。

### 碰撞模型、三角形材质与几何接口

- 从 IVP compact surface 保留 Source 质心、惯量、凸体结构、顶点、三角形、材质和每个 ledge 的游戏数据。
- 查询模型按凸体返回三角形、顶点及 gameData，并实现三角形材质索引的读写；索引遵循 IVP 的 7 位规则。
- 从闭合三角形构造多面体的顶点、边和面信息，并检查开放或异常的拓扑。
- 碰撞几何的序列化、反序列化和缩放都会维护相应元数据；缩放模型按重建几何处理，区分原始模型属性与运行时覆盖值。
- 修正 `$MATERIAL_INDEX_SHADOW` 的名称判断，材质和声音别名的大小写遵循 Source 规则。

代码：[碰撞模型](../vphysics_jolt/vjolt_collide.cpp)、[查询模型](../vphysics_jolt/vjolt_querymodel.cpp)、[表面属性](../vphysics_jolt/vjolt_surfaceprops.cpp)。

### 物理 KeyValues、车辆脚本和布娃娃碰撞规则

- `MassCenterOverride` 只在完整解析出向量后才启用。
- 车辆轮胎参数接受正确拼写的 `Inertia`，同时保留历史拼写 `Intertia`。
- 处理嵌套车辆参数、默认值、未知字段和无效表面索引；识别 Source 的 `RollingDrag`，但不改变行为。
- 布娃娃碰撞对逐项解析，校验编号、溢出和格式，畸形输入不会复用上一对编号。
- 使用与 Source 兼容的 token 解析，拒绝无法完整解析的输入，并释放解析树。

代码：[物理参数解析](../vphysics_jolt/vjolt_parse.cpp)、[KeyValues 解析器](../vphysics_jolt/vjolt_keyvalues_parser.h)、[Schema](../vphysics_jolt/vjolt_keyvalues_schema.cpp)。

### 存档与恢复

上游已有保存和恢复接口，本项目扩展了对象关系和状态管理：

- 保存与恢复会话管理对象 ID 及原指针到新对象的映射，并校验重复 ID 和调用顺序。
- 状态通过带格式信息的负载保存，能处理读取失败和旧负载。
- 恢复刚体、影子控制器、约束组和车辆之间的关联；约束组没有成员时也保留活动状态。
- 相关类型全部恢复后，才应用保存的活动与休眠状态，并通过 BodyID 识别仍有效的对象。

这些是 Source 存档流程的实现，并不提供时间倒流功能，也不保证所有 addon 的存档都完全兼容。

代码：[环境中的 Save/Restore 流程](../vphysics_jolt/vjolt_environment.cpp)、[保存恢复 ID 表](../vphysics_jolt/vjolt_saverestore_ids.h)、[对象状态](../vphysics_jolt/vjolt_object.cpp)。

### 碰撞对管理与回调期间的对象操作

对象对哈希改用独立的存储实现，按实际关联维护对象的归属、配对数量和邻接列表，移除一个配对不会丢失该对象的其他关联。

碰撞回调中的冻结、形状和质量修改、对象创建或删除都会延迟到安全时机执行，事件监听器和对象生命周期也处理了回调重入和对象失效。no-collide、工具操作、删除和清图等流程都依赖这些行为。

代码：[对象对存储](../vphysics_jolt/vjolt_objectpairhash_storage.h)、[对象对接口](../vphysics_jolt/vjolt_objectpairhash.cpp)、[环境生命周期](../vphysics_jolt/vjolt_environment_lifecycle.cpp)。

### 继承自上游的能力

并非所有与 Source 相关的代码都是本项目新增的。与基线逐文件比较，空气阻力控制器 `vjolt_controller_drag.cpp` 没有变化，按投影面积构造线性与角阻力的主要计算也已存在于基线。气垫船、材质继承和基础存档接口同样来自上游。本文只列出这些领域中的具体修改，整体功能仍归属上游。

## 约束与定制 Jolt 求解器

固定、长度和布娃娃约束改用 Source/Havana 的约束方程求解；铰链、球窝和滑轨等约束则处理兼容参数、马达、角度与断裂规则。适配时区分物体原点、质量中心及坐标转换，也区分各类约束中 strength、tau 等参数的用途。

约束组按实际物理岛安排成员和求解工作，并处理连接关系、唤醒传播和任务依赖。为了让兼容行为与原生接触共存，定制内核调整了相关物理岛中兼容摩擦、Havana 约束、接触预热和接触迭代的阶段顺序。

其中部分修复针对的是引入兼容求解之后才出现的问题，并不代表 基线存在这些缺陷。

代码：[约束适配](../vphysics_jolt/vjolt_constraints.cpp)、[Havana 实现](../vphysics_jolt/vjolt_havana_constraints.cpp)、[定制 Jolt](https://github.com/schweiziske/JoltPhysics-GMod)。

## 新增玩法

以下玩法位于 Q 菜单的「工具 → Jolt」和「武器 → Jolt」，界面支持简体中文和英文。完整操作见[额外玩法](../gmsv_jolt/GAMEPLAY.zh-CN.md)。

### 软体工坊与塑性接口

- 创建布料和充气软球，可调整尺寸、网格精度、总质量、柔软程度和充气量。
- 拖动、固定和释放顶点；支持与地图和刚体碰撞、伤害冲量，以及可选的玩家碰撞。
- 服务端模拟，按距离与 PVS 筛选网络同步，客户端插值并渲染网格。
- 原生接口支持二面角弯曲、屈服阈值、塑性松弛和永久形变，可用于自定义网格。

塑性 API 不包含模型导入、车辆外壳拆分或完整的车辆损伤系统。目前不支持软体互撞、自碰撞和撕裂。

文档：[软体](../gmsv_joltsoftbody/README.zh-CN.md)。

### 黑洞与重力控制

- 黑洞工具和黑洞手雷：把范围内的物体吸过来，再弹开。
- 世界重力、单个物体的重力倍率和方向，以及球形、盒形区域重力场。
- 定向、吸引和排斥模式，并提供方向编辑、可视化、权限与数量限制。

重力场作用于动态刚体和已接入的车辆控制器，不影响玩家移动和 NPC AI。

文档：[黑洞](../gmsv_jolt/GAMEPLAY.zh-CN.md#黑洞)、[重力系统](../gmsv_joltgravity/README.zh-CN.md)。

### 齿轮与齿条

提供齿轮传动、旋转到直线运动的转换、传动比和电机控制。工具自带支撑约束，并接入 Sandbox 的撤销、清理和数量限制。轴固定在世界中，适合固定式机械，不支持移动底座和车辆传动。

文档：[机械约束](../gmsv_joltconstraints/README.zh-CN.md)。

### 三维路径轨道

物体沿可编辑节点构成的三维曲线运动，由后端路径约束驱动。支持开放往返、闭环、恒速、定位、自由滑行、暂停、反向和预设，工具还提供节点编辑和移动参考系选择。

文档：[路径轨道](../gmsv_joltconstraints/PATH.zh-CN.md)。

## 面向 addon 和原生模块的 SDK

重力、机械、路径、软体及统计能力整理成了带版本的公共 C++ 接口，并提供 Lua 绑定和功能导出示例。

- **共用物理世界**：Bridge 直接操作运行中的后端环境，功能模块无需另建刚体世界。
- **公共契约**：接口写明单位、线程和生命周期，不跨 DLL 暴露 Jolt 内部对象。
- **资源管理**：接口版本校验、带代次或序号的句柄，以及回调期间的安全销毁。
- **Lua 按需加载**：通过 `require("jolt")`、`IsAvailable` 和 `Load` 使用配套功能。
- **开发示例**：演示如何把一项新能力从公共头文件、后端 Bridge 一路导出到 Lua。

文档：[物理 API](physics-api.zh-CN.md)、[公共 SDK](../public/vjolt/README.zh-CN.md)、[Bridge](../vphysics_jolt/bridge/README.zh-CN.md)、[导出示例](../examples/physics/export/README.zh-CN.md)。

## 加载、分发与游戏集成

CPU 变体包装层按 CPU 能力选择 AVX2、SSE4.2 或 SSE2 后端。项目为 Windows 默认分支的 32 位和 64 位，以及 `x86-64` 测试分支的 64 位提供配套构建；默认分支与 `x86-64` 分支的 64 位 ABI 不同，需要使用各自的安装包。

集成工作还包括物理对象和环境销毁、回调重入、延迟修改、跨环境转移等生命周期处理，以及 Sandbox 工具的权限、撤销、清理、网络和资源限制。

入口：[CPU 包装层](../vphysics_wrapper/vphysics_wrapper.cpp)、[安装指南](../gmsv_jolt/INSTALL.zh-CN.md)、[构建指南](build.zh-CN.md)、[生命周期实现](../vphysics_jolt/vjolt_environment_lifecycle.cpp)。

## 致谢

感谢 Joshua Ashton、Josh Dowell、RaphaelIT7、Jorrit Rouwe 以及上游项目的所有贡献者，没有他们的工作就没有这个项目。GMod 专项兼容、求解器集成和新增玩法由 schweiziske 在此基础上开发。
