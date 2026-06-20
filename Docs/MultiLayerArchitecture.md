# 极地科考破冰船模拟仿真环境 - 多层架构通信调试输出文档

## 文档版本
- 版本: 1.0.0
- 日期: 2026-06-20
- 作者: Ice Physics Engineering Team

---

## 目录
1. [系统架构概述](#1-系统架构概述)
2. [层级详细设计](#2-层级详细设计)
3. [跨层通信协议](#3-跨层通信协议)
4. [调试输出系统](#4-调试输出系统)
5. [数据流向分析](#5-数据流向分析)
6. [性能监控指标](#6-性能监控指标)
7. [常见问题排查](#7-常见问题排查)
8. [API 参考手册](#8-api-参考手册)

---

## 1. 系统架构概述

### 1.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                     Unity 应用层 (C#)                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │  游戏逻辑层   │  │  UI/HUD 层   │  │  可视化渲染层        │  │
│  │ IcebreakerShip│ │  DebugHUD    │  │  IceSheet/IceFragment│  │
│  └───────┬──────┘  └──────┬───────┘  └──────────┬───────────┘  │
│          │                │                       │               │
│          └────────────────┼───────────────────────┘               │
│                           │                                       │
│              ┌────────────▼─────────────┐                         │
│              │   C# 集成层 (P/Invoke)   │                         │
│              │   IcePhysicsInterop.cs    │                         │
│              └────────────┬─────────────┘                         │
└───────────────────────────┼─────────────────────────────────────────┘
                            │  [DLL 边界]
┌───────────────────────────▼─────────────────────────────────────────┐
│                   C++ 物理引擎层 (原生 DLL)                          │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    API 接口层 (IcePhysicsAPI.h)              │  │
│  │           IP_* 系列导出函数 (40+ 个 C 接口函数)              │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                                 │                                  │
│  ┌──────────────────────────────▼───────────────────────────────┐  │
│  │                     物理世界管理层                           │  │
│  │                 PhysicsWorld / DebugLogger                  │  │
│  └───────────────┬──────────────────────────┬───────────────────┘  │
│                  │                          │                      │
│  ┌───────────────▼──────────┐   ┌───────────▼──────────────┐    │
│  │     刚体动力学系统       │   │       碰撞检测系统        │    │
│  │  RigidBody / Manager     │   │  Broad/Narrow Phase      │    │
│  │  InertiaMomentumSolver   │   │  GJK / EPA / SAT         │    │
│  └───────────────┬──────────┘   │  ContactSolver           │    │
│                  │               └───────────┬──────────────┘    │
│                  │                           │                      │
│  ┌───────────────▼──────────┐   ┌───────────▼──────────────┐    │
│  │     水动力学系统         │   │       冰层系统            │    │
│  │  HydrodynamicSystem      │   │  IceSheet / Manager      │    │
│  │  IceResistanceCalculator │   │  VoronoiGenerator       │    │
│  └──────────────────────────┘   └──────────────────────────┘    │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │                      数学工具库                          │    │
│  │              Vector/Quaternion/Matrix                    │    │
│  └──────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 层级职责划分

| 层级 | 主要职责 | 技术栈 | 代码位置 |
|------|---------|--------|----------|
| **Unity 应用层** | 游戏逻辑、用户交互、可视化渲染、UI/HUD | C# / Unity API | `UnityProject/Assets/Scripts/` |
| **C# 集成层** | P/Invoke 绑定、类型转换、内存管理、事件转发 | C# / .NET Interop | `IcePhysicsInterop.cs` |
| **C++ API 层** | 导出 C 接口、DLL 入口点、参数验证 | C++ / extern "C" | `IcePhysicsAPI.h / .cpp` |
| **物理世界层** | 模拟主循环、时间步进、子系统协调、调试日志 | C++ | `PhysicsWorld.h / .cpp` |
| **刚体动力学层** | 牛顿-欧拉方程求解、惯性张量、动量守恒 | C++ | `RigidBody.h / .cpp` |
| **碰撞检测层** | 宽相/窄相检测、GJK/EPA/SAT 算法、接触求解 | C++ | `CollisionDetection.h / .cpp` |
| **水动力学层** | 阻力计算、螺旋桨推力、舵力、冰阻力 | C++ | `Hydrodynamics.h / .cpp` |
| **冰层系统层** | 冰层破碎、Voronoi 图、碎块物理 | C++ | `Voronoi.h / .cpp` |
| **数学工具层** | 向量/矩阵运算、几何计算、数值方法 | C++ (Header-only) | `MathUtils.h` |

---

## 2. 层级详细设计

### 2.1 Unity 应用层

#### 2.1.1 核心组件

| 组件类 | 职责 | 关键方法 |
|--------|------|---------|
| `IcePhysicsManager` | 物理引擎生命周期管理、全局参数设置、事件分发 | `InitializePhysics()`, `Simulate()`, `CreateRigidBody()` |
| `IcebreakerShip` | 破冰船控制、输入处理、状态更新 | `HandleInput()`, `UpdateNativePhysics()`, `SetThrottle()` |
| `IceSheet` | 冰层模型创建、压力分布可视化、破碎事件处理 | `InitializeIceSheet()`, `UpdatePressureDistribution()`, `SpawnFragments()` |
| `IceFragment` | 碎冰块物理、浮力模拟、生命周期管理 | `Initialize()`, `ApplyBuoyancy()`, `GenerateFragmentMesh()` |
| `DebugHUD` | 调试信息显示、日志输出、性能监控 | `UpdateStatsDisplay()`, `OnDebugLogHandler()`, `ToggleHUD()` |

#### 2.1.2 事件系统

```csharp
// 事件定义
public event Action<DebugLogEntry> OnDebugLog;
public event Action<CollisionEventType, uint, uint, CollisionManifold[]> OnCollisionEvent;
public event Action<uint, Vector3, IceFragmentDesc[]> OnIceBreak;
```

### 2.2 C# 集成层 (P/Invoke)

#### 2.2.1 数据结构编组

| C++ 类型 | C# 类型 | 编组特性 |
|----------|---------|---------|
| `Vector3` | `struct Vector3` | `[StructLayout(LayoutKind.Sequential)]` |
| `Quaternion` | `struct Quaternion` | `[StructLayout(LayoutKind.Sequential)]` |
| `Matrix3x3` | `struct Matrix3x3` | `[MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]` |
| `Vertex` | `struct Vertex` | `[StructLayout(LayoutKind.Sequential)]` |
| `Triangle` | `struct Triangle` | `[MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]` |
| `DebugLogEntry` | `struct DebugLogEntry` | `[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]` |

#### 2.2.2 委托回调

```csharp
// 回调委托定义
public delegate void DebugLogCallback(ref DebugLogEntry entry);
public delegate void CollisionEventCallback(CollisionEventType type, uint bodyA, uint bodyB, IntPtr manifold, uint manifoldCount);
public delegate void IceBreakCallback(uint iceSheetId, Vector3 impactPoint, uint fragmentCount, IntPtr fragments);

// 防止 GC 回收的引用持有
private DebugLogCallback m_debugLogCallback;
private CollisionEventCallback m_collisionEventCallback;
private IceBreakCallback m_iceBreakCallback;
```

#### 2.2.3 内存管理模式

1. ** blittable 类型直接传递**: `int`, `float`, `Vector3` 等简单类型直接复制
2. **数组参数**: 接收时先查询大小，分配内存后再次调用
3. **指针编组**: 使用 `Marshal.PtrToStructure<T>()` 从非托管内存读取
4. **回调机制**: C++ 调用 C# 委托时需要保持委托引用不被 GC 回收

### 2.3 C++ API 层

#### 2.3.1 导出函数分类

| 类别 | 函数数量 | 命名前缀 |
|------|---------|---------|
| 初始化/关闭 | 2 | `IP_Initialize`, `IP_Shutdown` |
| 回调设置 | 3 | `IP_Set*Callback` |
| 模拟控制 | 1 | `IP_Simulate` |
| 刚体管理 | 13 | `IP_*RigidBody*`, `IP_Apply*` |
| 碰撞器 | 5 | `IP_Create*Collider`, `IP_SetCollider*` |
| 冰层 | 6 | `IP_CreateIceSheet`, `IP_GetIce*` |
| 水动力学 | 7 | `IP_Set*Params`, `IP_Calculate*` |
| 统计查询 | 5 | `IP_Get*Count`, `IP_GetSimulationTime` |
| 错误处理 | 2 | `IP_GetLastError`, `IP_ClearLastError` |

#### 2.3.2 调用约定

```cpp
// 所有导出函数使用 C 调用约定
extern "C"
{
    ICEPHYSICS_API bool IP_Initialize(float fixedTimestep, DebugLevel debugLevel);
    // ...
}
```

### 2.4 物理世界层

#### 2.4.1 主循环流程

```
IP_Simulate(deltaTime)
    │
    ├─► 时间累加器更新 (固定时间步长)
    │
    ├─► 每固定时间步: Step(fixedTimestep)
    │   │
    │   ├─► 应用水动力 (HydrodynamicSystem)
    │   ├─► 碰撞检测 (CollisionWorld)
    │   │   ├─► 宽相检测 (BroadPhase)
    │   │   ├─► 窄相检测 (NarrowPhase - GJK/EPA)
    │   │   └─► 接触求解 (ContactSolver)
    │   ├─► 冰碰撞检测 (CheckIceCollisions)
    │   ├─► 冰冲击处理 (ProcessIceImpacts)
    │   ├─► 刚体积分 (IntegrateAll)
    │   └─► 更新冰层 (IceSheetManager)
    │
    └─► 更新统计信息
```

#### 2.4.2 时间步进策略

```cpp
void PhysicsWorld::Simulate(float deltaTime)
{
    m_accumulator += deltaTime;

    while (m_accumulator >= m_fixedTimestep)
    {
        Step(m_fixedTimestep);
        m_accumulator -= m_fixedTimestep;
        m_simulationTime += m_fixedTimestep;
        m_frameCount++;
    }
}
```

### 2.5 刚体动力学层

#### 2.5.1 牛顿-欧拉方程

**线性运动方程**:
```
F = m · a
dv/dt = F / m
```

**角运动方程**:
```
τ = I · α + ω × (I · ω)
dω/dt = I⁻¹ · (τ - ω × (I · ω))
```

**数值积分**:
```cpp
void RigidBody::IntegrateLinear(float dt)
{
    Vector3 acceleration = m_forceAccumulator * m_invMass;
    m_linearVelocity += acceleration * dt;
    m_linearVelocity *= (1.0f - m_linearDamping * dt);
    m_position += m_linearVelocity * dt;
}
```

#### 2.5.2 惯性张量计算

对于长方体刚体（如破冰船）：
```cpp
// Ix = (1/12) · m · (y² + z²)
// Iy = (1/12) · m · (x² + z²)  
// Iz = (1/12) · m · (x² + y²)
float Ix = (1.0f / 12.0f) * mass * (beam*beam + draft*draft);
float Iy = (1.0f / 12.0f) * mass * (length*length + draft*draft);
float Iz = (1.0f / 12.0f) * mass * (length*length + beam*beam);
```

### 2.6 碰撞检测层

#### 2.6.1 碰撞检测流水线

```
1. 宽相检测 (BroadPhase - O(n log n))
   ├─► AABB 代理更新
   ├─► 潜在碰撞对生成
   └─► 自碰撞/静态碰撞过滤

2. 窄相检测 (NarrowPhase)
   ├─► 盒体-盒体: SAT 算法 (15个分离轴)
   ├─► 凸体-凸体: GJK 算法 + EPA 算法
   ├─► 盒体-网格: 逐三角形 SAT
   └─► 生成碰撞流形 (Manifold)

3. 接触求解 (ContactSolver)
   ├─► 速度约束求解 (多次迭代)
   ├─► 位置约束求解 (Baumgarte 稳定)
   ├─► 库仑摩擦模型
   └─► 恢复系数应用
```

#### 2.6.2 GJK 算法核心

**距离查询**:
```
function GJK(shapeA, shapeB):
    simplex = []
    direction = initial_direction
    
    while true:
        support = GetSupport(shapeA, shapeB, direction)
        if Dot(support, direction) < 0:
            return false  // 不碰撞
        
        add support to simplex
        
        if ContainsOrigin(simplex, direction):
            return true   // 碰撞
```

**EPA 算法 (穿透深度计算)**:
```
function EPA(simplex, shapeA, shapeB):
    polytope = BuildInitialPolytope(simplex)
    
    while iterations < max_iterations:
        face = FindClosestFace(polytope)
        support = GetSupport(shapeA, shapeB, face.normal)
        
        if Dot(support, face.normal) - face.distance < tolerance:
            return face.distance  // 穿透深度
            
        ExpandPolytope(polytope, support)
    
    return closest_depth
```

### 2.7 水动力学层

#### 2.7.1 阻力计算公式

**ITTC 1957 摩擦阻力公式**:
```
Cf = 0.075 / (log10(Rn) - 2)²
Rf = 0.5 · ρ · V² · S · Cf
```

其中:
- `Rn = V·L/ν` (雷诺数)
- `ρ` = 水密度 (1025 kg/m³)
- `V` = 船速 (m/s)
- `L` = 船长 (m)
- `S` = 湿表面积 (m²)
- `ν` = 水的运动粘度 (m²/s)

**弗劳德数**:
```
Fn = V / √(g·L)
```

**兴波阻力**:
```
Rw = 0.5 · ρ · V² · S · Cw(Fn, Cp, hull_form)
```

**冰阻力 (Lindqvist 公式)**:
```
Rice = R_breaking + R_bending + R_crushing + R_submergence
```

#### 2.7.2 螺旋桨推力公式

```
T = (P · ηp · ηh) / V
```

其中:
- `T` = 推力 (N)
- `P` = 引擎功率 (W)
- `ηp` = 螺旋桨效率 (0.7-0.8)
- `ηh` = 船体效率 (1.1-1.2)
- `V` = 船速 (m/s)

#### 2.7.3 舵力计算

```
Cl = 2π · AR · α / (AR + 2)
F_rudder = 0.5 · ρ · V² · A_rudder · Cl
```

### 2.8 冰层系统层

#### 2.8.1 Voronoi 图算法

**Fortune 算法流程**:
```
1. 事件队列初始化 (所有站点事件)
2. 海滩线初始化 (空)
3. 处理事件:
   ├─► 站点事件: 在海滩线插入新弧
   │   └─► 检查新弧的相邻弧，可能产生圆事件
   └─► 圆事件: 弧消失，生成 Voronoi 顶点
       └─► 更新相邻弧，可能产生新圆事件
4. 清理剩余边，裁剪到边界
5. 构建 Voronoi 单元
```

**Lloyd 松弛优化**:
```
for i = 1 to iterations:
    for each cell:
        centroid = CalculateCentroid(cell_vertices)
        site.position = centroid
    
    regenerate_voronoi(sites)
```

#### 2.8.2 冰层破碎判定

**应力计算**:
```
// 弯曲应力 (梁理论)
σ_bending = (6 · M) / (b · h²)

// 剪切应力
τ_shear = F / (b · h)

// 接触压强
p_contact = F_contact / A_contact
```

**破碎条件**:
```
if max(σ_bending, τ_shear) > yield_strength:
    BreakIce(impact_point, pressure_distribution)
```

#### 2.8.3 碎块生成流程

```
1. 检测到超屈服应力的撞击
2. 在撞击点周围生成 Voronoi 种子点 (径向分布)
3. 生成 Voronoi 图切分冰层网格
4. 每个 Voronoi 单元生成独立碎块:
   ├─► 拉伸 2D 轮廓为 3D 网格 (上下表面 + 侧面)
   ├─► 计算质量属性 (体积、质心、惯性张量)
   ├─► 初始化刚体状态 (速度、角速度、位置)
   ├─► 应用浮力和水阻力
   └─► 添加到物理世界
5. 原冰层隐藏，碎块接管渲染
```

---

## 3. 跨层通信协议

### 3.1 调用栈分析

#### 3.1.1 模拟主循环调用链

```
Unity FixedUpdate()
    │
    └─► IcePhysicsManager.Update()
        │
        └─► [P/Invoke] IP_Simulate(deltaTime)
            │   [DLL 边界]
            └─► PhysicsWorld::Simulate(deltaTime)
                │
                ├─► Step(fixedTimestep)
                │   ├─► UpdateShipState() → 更新船舶状态到 C++
                │   ├─► CalculateHydrodynamicForces() → 计算水动力
                │   ├─► CollisionWorld::Step() → 碰撞检测+求解
                │   ├─► CheckIceCollisions() → 冰碰撞检测
                │   ├─► ProcessIceImpacts() → 冰破碎处理
                │   ├─► RigidBodyManager::IntegrateAll() → 数值积分
                │   └─► IceSheetManager::Update() → 更新碎块
                │
                └─► UpdateStats() → 更新统计信息
```

#### 3.1.2 冰破碎事件回调链

```
C++: ProcessIceImpacts()
    │
    ├─► 检测到超阈值冰碰撞
    ├─► IceSheet::BreakAtPoint() → Voronoi 切分
    │   └─► 生成 IceFragmentDesc 数组
    │
    └─► 调用回调: IceBreakCallback(iceSheetId, impactPoint, count, fragments)
        │   [DLL 边界 - 反向调用]
        │
C#: OnIceBreakHandler(uint, Vector3, uint, IntPtr)
    │
    ├─► Marshal 数组: IcePhysicsInterop.MarshalFragments()
    │   └─► 从非托管内存复制到托管数组
    │
    ├─► 触发事件: OnIceBreak?.Invoke()
    │
    └─► IceSheet.OnIceBreakHandler()
        ├─► SpawnFragments() → 创建 Unity GameObject
        └─► 每个碎块: AddComponent<IceFragment>() → Initialize()
```

### 3.2 数据流时序图

#### 3.2.1 破冰船控制数据流

```
用户输入 (键盘/手柄)
    │
    ▼
IcebreakerShip.HandleInput() [Unity Update]
    ├─► Input.GetAxis("Vertical") → throttle
    └─► Input.GetAxis("Horizontal") → rudder
    │
    ▼
UpdateControls()
    ├─► 平滑过渡: current = SmoothDamp(current, target)
    └─► 设置 m_currentThrottle, m_currentRudderAngle
    │
    ▼
UpdateNativePhysics() [Unity FixedUpdate]
    ├─► 构建 ShipState 结构体
    ├─► [P/Invoke] IP_UpdateShipState(bodyId, ref state)
    │   │
    │   ▼ [C++]
    │   PhysicsWorld::UpdateShipState()
    │   └─► 存储到 m_shipStates[bodyId]
    │
    ├─► [P/Invoke] IP_CalculateHydrodynamicForces(bodyId, out force, out torque)
    │   │
    │   ▼ [C++]
    │   HydrodynamicSystem::CalculateForces()
    │   ├─► CalculateThrustForce(throttle, speed)
    │   ├─► CalculateRudderForce(rudder, speed)
    │   ├─► CalculateManeuveringForce(rudder, speed, drift)
    │   └─► 返回合力和合力矩
    │
    ├─► [P/Invoke] IP_GetRigidBodyPosition(bodyId, out pos)
    ├─► [P/Invoke] IP_GetRigidBodyRotation(bodyId, out rot)
    ├─► [P/Invoke] IP_GetRigidBodyLinearVelocity(bodyId, out vel)
    └─► 更新 Unity Transform: transform.position/rotation = pos/rot
```

#### 3.2.2 冰碰撞破碎数据流

```
碰撞检测阶段 [C++]
    │
    ▼
NarrowPhase::TestHullMesh(shipCollider, iceCollider)
    ├─► GJK 检测是否碰撞
    └─► EPA 计算穿透深度和接触点
    │
    ▼
ContactSolver::Solve()
    ├─► 计算碰撞冲量
    └─► 应用到刚体速度
    │
    ▼
PhysicsWorld::CheckIceCollisions()
    ├─► 遍历活跃碰撞
    ├─► 识别船-冰碰撞对
    └─► 构建 IceImpactData
        ├─► impactPoint = manifold.point
        ├─► impactNormal = manifold.normal
        ├─► impactVelocity = relativeVelocity · normal
        ├─► impactForce = impulse / dt
        └─► pressure = force / contactArea
    │
    ▼
PhysicsWorld::ProcessIceImpacts()
    ├─► IceSheet::CheckImpact(impact, yieldStrength)
    │   ├─► 计算弯曲应力、剪切应力
    │   └─► 判定是否超过屈服强度
    │
    ├─► 如果破碎:
    │   ├─► IceSheet::BreakAtPoint(impact, yieldStrength)
    │   │   ├─► GenerateVoronoiSites(impactPoint, radius, count)
    │   │   ├─► VoronoiGenerator::Generate(sites, bounds)
    │   │   ├─► 对每个 Voronoi Cell:
    │   │   │   ├─► GenerateFragmentMesh() → 3D 网格
    │   │   │   ├─► CalculateFragmentMassProperties()
    │   │   │   ├─► CalculateFragmentInertia()
    │   │   │   ├─► InitializeFragmentPhysics()
    │   │   │   └─► 添加到 m_fragments
    │   │   └─► 返回 fragmentCount + fragmentDescs
    │   │
    │   └─► 调用回调: m_iceBreakCallback(iceSheetId, impactPoint, count, descs)
    │       │   [DLL 边界]
    │       ▼
    │   C# OnIceBreakHandler()
    │   ├─► Marshal 碎片数据
    │   └─► SpawnFragments() → 创建 Unity 对象
    │
    └─► 更新冰层激活状态
```

### 3.3 内存边界协议

#### 3.3.1 C# → C++ 数据传递

**值类型 (blittable)**:
```csharp
// 直接复制，不需要编码
Vector3 pos = transform.position;
IcePhysicsInterop.IP_SetRigidBodyPosition(bodyId, ref pos);
```

**数组类型**:
```csharp
// 1. 先查询需要的大小
uint vertexCount = 0;
uint triangleCount = 0;
IcePhysicsInterop.IP_GetIceSheetMesh(iceSheetId, out vertexCount, IntPtr.Zero, out triangleCount, IntPtr.Zero);

// 2. 分配非托管内存
int vertexSize = Marshal.SizeOf<Vertex>();
IntPtr vertexPtr = Marshal.AllocHGlobal((int)vertexCount * vertexSize);

try
{
    // 3. 实际调用获取数据
    if (IcePhysicsInterop.IP_GetIceSheetMesh(iceSheetId, out vertexCount, vertexPtr, ...))
    {
        // 4. 从非托管内存复制到托管数组
        Vertex[] vertices = IcePhysicsInterop.MarshalArrayFromIntPtr<Vertex>(vertexPtr, (int)vertexCount);
    }
}
finally
{
    // 5. 释放非托管内存
    Marshal.FreeHGlobal(vertexPtr);
}
```

#### 3.3.2 C++ → C# 回调数据

**C++ 侧**:
```cpp
void PhysicsWorld::OnIceBreak(uint iceSheetId, Vector3 impactPoint, uint fragmentCount, const IceFragmentDesc* fragments)
{
    if (m_iceBreakCallback)
    {
        // 传递原始指针，C# 负责编组
        m_iceBreakCallback(iceSheetId, impactPoint, fragmentCount, (IntPtr)fragments);
    }
}
```

**C# 侧**:
```csharp
private void OnIceBreakHandler(uint iceSheetId, Vector3 impactPoint, uint fragmentCount, IntPtr fragmentsPtr)
{
    // 从非托管内存数组读取
    IceFragmentDesc[] fragments = IcePhysicsInterop.MarshalFragments(fragmentsPtr, fragmentCount);
    
    // 触发事件
    OnIceBreak?.Invoke(iceSheetId, impactPoint, fragments);
}
```

### 3.4 线程安全协议

**当前设计限制**:
- 所有 C++ 物理引擎调用必须在 Unity 主线程执行
- 不支持多线程调用物理 API
- 回调函数在物理模拟线程（主线程）中调用

**未来可扩展**:
- 添加线程安全的 API 队列
- 物理模拟与游戏逻辑解耦到独立线程
- 使用 lock-free 队列进行跨线程通信

---

## 4. 调试输出系统

### 4.1 日志系统架构

```
C++ 代码
    │
    ├─► DebugLogger::Log(level, subsystem, format, ...)
    │   ├─► 级别过滤 (level <= m_level)
    │   ├─► 格式化消息 (vsnprintf)
    │   ├─► 获取时间戳
    │   ├─► 构建 DebugLogEntry
    │   └─► 调用回调: m_callback(&entry)
    │       │
    │       ▼ [DLL 边界]
    │       C# DebugLogCallback(ref entry)
    │
    └─► 内部错误存储: SetLastError(message)
        └─► 通过 IP_GetLastError() 查询


C# 代码
    │
    ├─► IcePhysicsManager.OnDebugLogHandler(ref entry)
    │   ├─► 根据级别输出到 Unity Console
    │   │   ├─► Error → Debug.LogError()
    │   │   ├─► Warning → Debug.LogWarning()
    │   │   └─► Info/Verbose/Debug → Debug.Log()
    │   └─► 触发 OnDebugLog 事件
    │
    └─► DebugHUD.OnDebugLogHandler(entry)
        ├─► 格式化带颜色的 HTML 标签
        ├─► 添加到日志队列 (最多 100 条)
        └─► 更新 UI 文本显示
```

### 4.2 调试级别定义

```cpp
enum class DebugLevel : uint32_t
{
    None = 0,      // 不输出任何日志
    Error = 1,     // 致命错误，系统无法继续
    Warning = 2,   // 潜在问题，但可继续运行
    Info = 3,      // 重要运行信息（默认级别）
    Verbose = 4,   // 详细运行信息
    Debug = 5      // 调试细节（性能影响大）
};
```

### 4.3 子系统划分

| 子系统名称 | 描述 | 典型日志内容 |
|-----------|------|-------------|
| `PhysicsWorld` | 物理世界主循环 | 初始化、关闭、步进时间 |
| `RigidBody` | 刚体动力学 | 刚体创建/销毁、力应用、积分 |
| `Collision` | 碰撞检测 | 碰撞对生成、流形计算、求解迭代 |
| `Hydrodynamics` | 水动力学 | 阻力计算、推力计算、舵力 |
| `IceSheet` | 冰层系统 | 冰层创建、破碎判定、碎块生成 |
| `Voronoi` | Voronoi 算法 | 图生成、松弛迭代、单元构建 |
| `API` | API 接口层 | 函数调用、参数验证、错误信息 |

### 4.4 调试日志示例

```
[14:32:15.234] [INFO] [PhysicsWorld] Initializing physics engine with 60Hz timestep
[14:32:15.245] [INFO] [IceSheet] Creating ice sheet 1: 200x200m, thickness 1.5m
[14:32:15.312] [INFO] [RigidBody] Created body 1 (icebreaker) mass=20000t
[14:32:15.456] [INFO] [Collision] BroadPhase: 2 proxies, 1 potential pairs
[14:32:16.789] [WARNING] [Hydrodynamics] Ship speed exceeds design limit (21 knots)
[14:32:18.123] [INFO] [IceSheet] Impact detected at (50.0, 0.0, 100.0), pressure=2.45 MPa
[14:32:18.124] [INFO] [IceSheet] Ice sheet 1 exceeded yield strength (2.45 > 1.5 MPa)
[14:32:18.125] [INFO] [Voronoi] Generating diagram with 12 sites
[14:32:18.156] [INFO] [IceSheet] Broken into 12 fragments
[14:32:18.234] [ERROR] [API] IP_GetIceFragmentData called with invalid fragment index 99
```

### 4.5 错误处理机制

#### 4.5.1 错误信息流

```
C++ 函数检测错误
    │
    ├─► DebugLogger::SetLastError("详细错误描述")
    │
    ├─► 返回错误值 (false, 0, nullptr 等)
    │
    ▼
C# 调用方检查返回值
    │
    ├─► 失败时:
    │   ├─► string error = IcePhysicsInterop.GetLastErrorString()
    │   └─► Debug.LogError($"[IcePhysics] 操作失败: {error}")
    │
    └─► 可选: IcePhysicsInterop.IP_ClearLastError()
```

#### 4.5.2 常见错误代码

| 错误信息 | 原因 | 解决方法 |
|---------|------|---------|
| `Physics world not initialized` | 调用 API 前未调用 IP_Initialize | 确保 IcePhysicsManager 已启用并初始化 |
| `Invalid rigid body id: 0` | 使用了无效的刚体 ID | 检查刚体是否已创建且未销毁 |
| `Null pointer passed for required parameter` | 传递了空指针给必填参数 | 检查 C# 代码中的 ref/out 参数 |
| `Invalid parameter: fixedTimestep must be > 0` | 时间步长参数无效 | 传递正数作为时间步长 |
| `Ice sheet mesh is empty` | 冰层网格数据为空 | 检查冰层描述参数是否正确 |
| `Voronoi diagram generation failed` | Voronoi 图生成失败 | 检查站点点是否共线或重合 |

---

## 5. 数据流向分析

### 5.1 输入数据流

```
用户输入 (WASD/方向键)
    │
    ├─► Input.GetAxis() [Unity]
    │   └─► 轴值映射到 [-1, 1]
    │
    ├─► IcebreakerShip.HandleInput()
    │   ├─► throttle = VerticalAxis → [-1, 1]
    │   └─► rudder = HorizontalAxis → [-maxRudder, maxRudder]
    │
    ▼
ShipState 结构体 (每帧更新)
    ├─► position
    ├─► rotation
    ├─► linearVelocity
    ├─► angularVelocity
    ├─► throttle
    ├─► rudderAngle
    ├─► speedKnots
    ├─► headingDegrees
    └─► driftAngle
    │
    ▼ [P/Invoke]
IP_UpdateShipState(bodyId, ref shipState)
    │
    ▼ [C++]
PhysicsWorld::m_shipStates[bodyId] = state
```

### 5.2 物理状态数据流

```
物理模拟 Step(dt)
    │
    ├─► 水动力计算
    │   ├─► 输入: ShipState (油门、舵角、速度)
    │   ├─► 计算: 推力、阻力、舵力、力矩
    │   └─► 输出: totalForce, totalTorque
    │
    ├─► 碰撞检测
    │   ├─► 输入: 所有刚体位置/速度
    │   ├─► 宽相 → 窄相 → 求解
    │   └─► 输出: 碰撞冲量、位置修正
    │
    ├─► 冰碰撞检测
    │   ├─► 输入: 船-冰碰撞流形
    │   ├─► 计算: 接触压强、应力分布
    │   └─► 输出: 破碎判定、冲击数据
    │
    ├─► 冰层破碎
    │   ├─► 输入: 冲击点、压强
    │   ├─► Voronoi 切分 → 碎块生成
    │   └─► 输出: 碎块描述数组
    │
    └─► 刚体积分
        ├─► 输入: 合力、合力矩、冲量
        ├─► 数值积分 (显式欧拉)
        └─► 输出: 新的位置、旋转、速度
        │
        ▼ [P/Invoke]
IP_GetRigidBody*() 系列函数
    │
    ▼
Unity Transform 同步
    ├─► transform.position = newPosition
    └─► transform.rotation = newRotation
```

### 5.3 视觉渲染数据流

```
冰层/碎块几何数据 [C++]
    │
    ├─► IceSheet::GenerateMesh()
    │   ├─► 生成网格顶点、法线、UV
    │   ├─► 生成三角面索引
    │   └─► 存储到 m_vertices, m_triangles
    │
    ├─► [P/Invoke] IP_GetIceSheetMesh()
    │   └─► 复制到非托管内存
    │
    ▼ [C#]
MarshalArrayFromIntPtr<Vertex/Triangle>()
    │
    ├─► 转换到 Unity 格式
    │   ├─► Vector3[] meshVertices
    │   ├─► int[] meshTriangles
    │   └─► Vector2[] meshUVs
    │
    └─► Mesh 构建
        ├─► mesh.vertices = meshVertices
        ├─► mesh.triangles = meshTriangles
        ├─► mesh.RecalculateBounds()
        └─► meshFilter.sharedMesh = mesh
        │
        ▼
GPU 渲染
    └─► 材质 + 光照 + 后处理 → 屏幕显示
```

### 5.4 调试信息数据流

```
物理模拟过程 [C++]
    │
    ├─► 各子系统调用 DebugLogger::Log()
    │   ├─► 时间戳生成
    │   ├─► 级别过滤
    │   └─► 调用 C# 回调
    │
    ▼ [DLL 边界]
C# DebugLogCallback
    │
    ├─► IcePhysicsManager: 输出到 Unity Console
    │   ├─► Debug.Log/LogWarning/LogError
    │   └─► Editor Console 窗口显示
    │
    └─► DebugHUD: 屏幕显示
        ├─► 格式化带颜色的日志条目
        ├─► 维护滚动队列 (最多 100 条)
        └─► UGUI Text 更新
```

---

## 6. 性能监控指标

### 6.1 性能统计结构体

```cpp
struct PhysicsWorldStats
{
    float   simulationTime;         // 总模拟时间 (秒)
    uint64_t frameCount;            // 总帧数
    uint32_t rigidBodyCount;        // 活跃刚体数
    uint32_t colliderCount;         // 活跃碰撞器数
    uint32_t iceSheetCount;         // 活跃冰层数
    uint32_t activeCollisions;      // 活跃碰撞数
    uint32_t iceFragmentCount;      // 碎块总数
    float   broadPhaseTimeMs;       // 宽相检测耗时 (毫秒)
    float   narrowPhaseTimeMs;      // 窄相检测耗时 (毫秒)
    float   solverTimeMs;           // 接触求解耗时 (毫秒)
    float   hydrodynamicTimeMs;     // 水动力计算耗时 (毫秒)
    float   totalStepTimeMs;        // 单步总耗时 (毫秒)
};
```

### 6.2 性能监控指标

| 指标 | 目标值 | 警告阈值 | 说明 |
|------|--------|---------|------|
| 总步进时间 | < 16ms | > 33ms | 60fps 要求每步 < 16ms |
| 宽相检测时间 | < 2ms | > 5ms | 与碰撞体数量 O(n log n) |
| 窄相检测时间 | < 5ms | > 10ms | 与碰撞对数线性相关 |
| 接触求解时间 | < 3ms | > 8ms | 与迭代次数×接触点相关 |
| 水动力计算时间 | < 1ms | > 3ms | 通常很快，公式计算 |
| 刚体数量 | < 500 | > 2000 | 包括碎块刚体 |
| 碎块数量 | < 200 | > 500 | 过多会影响性能 |
| 碰撞对数量 | < 100 | > 500 | 宽相输出 |

### 6.3 性能优化建议

1. **冰层网格细分**:
   -  subdivisions 参数控制网格密度
   - 建议值: 32-64 (约 1000-4000 顶点)
   - 过密会大幅增加碰撞检测时间

2. **Voronoi 碎块数量**:
   - 每次破碎控制在 8-15 个碎块
   - 碎块生命周期结束后自动销毁
   - 可设置最大同时存在碎块数

3. **碰撞检测优化**:
   - 合理设置 AABB，减少宽相误报
   - 远距离冰层禁用碰撞检测
   - 静态冰层使用复合碰撞体

4. **调试级别**:
   - 发布版本设置 DebugLevel::Warning 或 None
   - Verbose/Debug 级别有显著性能开销

---

## 7. 常见问题排查

### 7.1 DLL 加载问题

**问题**: `DllNotFoundException: IcePhysicsEngine`

**排查步骤**:
1. 确认 DLL 存在于 `UnityProject/Assets/Plugins/` 目录
2. 检查 DLL 架构 (x64 vs x86) 与 Unity 构建设置匹配
3. 确认 Visual C++ Redistributable 已安装
4. 检查 Unity 导入设置: Platform 设置为 Windows, CPU 设置为 x86_64
5. 查看 Unity Editor.log 详细错误信息

### 7.2 模拟不稳定问题

**问题**: 刚体抖动、穿透、爆炸

**排查步骤**:
1. 减小固定时间步长 (如从 1/60 改为 1/120)
2. 增加接触求解迭代次数 (默认 8→16)
3. 检查刚体质量比 (最大:最小 < 10:1)
4. 确认碰撞器形状匹配渲染网格
5. 检查是否有零质量或零惯性张量的刚体
6. 降低 restitution 系数 (0.0-0.2 适合冰场景)

### 7.3 冰层不破碎问题

**问题**: 船撞上冰层但没有破碎

**排查步骤**:
1. 检查冰层 yieldStrength 设置 (默认 1.5e6 Pa)
2. 确认船速足够产生超阈值压力
3. 查看调试日志是否有 `Impact detected` 消息
4. 检查碰撞层设置，确保船和冰层在同一层
5. 确认 `m_targetShip` 在 IceSheet 组件中已赋值

### 7.4 碎块显示异常

**问题**: 碎块为黑色、不可见、或形状错误

**排查步骤**:
1. 检查材质 Shader 是否正确 (Standard)
2. 确认网格法线方向正确 (朝外)
3. 检查三角面缠绕顺序 (顺时针为正面)
4. 查看碎块位置是否在摄像机视锥内
5. 确认回退盒形网格生成正常工作

### 7.5 水动力计算异常

**问题**: 船速异常高或低、阻力为零

**排查步骤**:
1. 检查水动力参数: hullWettedArea, hullLength 等是否合理
2. 确认 EngineThrustParams 中 totalPower > 0
3. 查看调试日志中的阻力计算值
4. 确认水密度、温度参数合理
5. 检查螺旋桨效率和船体效率在合理范围 (0.7-1.2)

### 7.6 性能问题

**问题**: 帧率低、卡顿

**排查步骤**:
1. 使用 Profiler 查看 CPU 耗时分布
2. 检查是否 Debug 级别过高 (Verbose/Debug)
3. 减少冰层细分和碎块数量
4. 确认没有日志输出到 Console (影响巨大)
5. 考虑使用物体池复用碎块对象

---

## 8. API 参考手册

### 8.1 初始化与控制

#### IP_Initialize
```cpp
bool IP_Initialize(float fixedTimestep, DebugLevel debugLevel);
```
**描述**: 初始化物理引擎
**参数**:
- `fixedTimestep`: 固定时间步长 (秒)，建议 1/60
- `debugLevel`: 调试日志级别
**返回**: true 表示初始化成功

#### IP_Shutdown
```cpp
void IP_Shutdown();
```
**描述**: 关闭物理引擎，释放所有资源

#### IP_Simulate
```cpp
void IP_Simulate(float deltaTime);
```
**描述**: 推进物理模拟
**参数**:
- `deltaTime`: 距上次调用的时间差 (秒)

### 8.2 刚体管理

#### IP_CreateRigidBody
```cpp
uint32_t IP_CreateRigidBody(const RigidBodyDesc* desc);
```
**描述**: 创建刚体
**参数**:
- `desc`: 刚体描述结构体指针
**返回**: 刚体 ID，0 表示失败

#### IP_DestroyRigidBody
```cpp
void IP_DestroyRigidBody(uint32_t bodyId);
```
**描述**: 销毁刚体
**参数**:
- `bodyId`: 刚体 ID

### 8.3 冰层管理

#### IP_CreateIceSheet
```cpp
uint32_t IP_CreateIceSheet(const IceSheetDesc* desc);
```
**描述**: 创建冰层
**参数**:
- `desc`: 冰层描述结构体
**返回**: 冰层 ID，0 表示失败

#### IP_GetIceSheetMesh
```cpp
bool IP_GetIceSheetMesh(uint32_t iceSheetId, uint32_t* vertexCount, Vertex* vertices,
                        uint32_t* triangleCount, Triangle* triangles);
```
**描述**: 获取冰层网格数据
**参数**:
- `iceSheetId`: 冰层 ID
- `vertexCount`: [in/out] 顶点数量，vertices=null 时返回需要的大小
- `vertices`: [out] 顶点数组指针，null 时查询大小
- `triangleCount`: [in/out] 三角面数量
- `triangles`: [out] 三角面数组指针，null 时查询大小
**返回**: true 表示成功

#### IP_GetIceFragmentCount
```cpp
uint32_t IP_GetIceSheetFragmentCount(uint32_t iceSheetId);
```
**描述**: 获取冰层的碎块数量

### 8.4 水动力学

#### IP_CalculateHydrodynamicForces
```cpp
void IP_CalculateHydrodynamicForces(uint32_t bodyId, Vector3* totalForce, Vector3* totalTorque);
```
**描述**: 计算船舶受到的水动力和力矩
**参数**:
- `bodyId`: 船舶刚体 ID
- `totalForce`: [out] 总合力 (局部坐标系)
- `totalTorque`: [out] 总力矩 (局部坐标系)

#### IP_CalculateShipResistance
```cpp
float IP_CalculateShipResistance(float speedKnots, float waterTemperature, float iceConcentration);
```
**描述**: 计算船舶总阻力
**参数**:
- `speedKnots`: 船速 (节)
- `waterTemperature`: 水温 (°C)
- `iceConcentration`: 冰密集度 (0-1)
**返回**: 总阻力 (N)

### 8.5 调试与统计

#### IP_GetActiveRigidBodyCount
```cpp
uint32_t IP_GetActiveRigidBodyCount();
```
**描述**: 获取当前活跃刚体数量

#### IP_GetSimulationTime
```cpp
float IP_GetSimulationTime();
```
**描述**: 获取总模拟时间 (秒)

#### IP_GetLastError
```cpp
const char* IP_GetLastError();
```
**描述**: 获取最后一次错误信息
**返回**: 错误字符串指针，调用者不需要释放

---

## 附录 A: 物理常量

| 符号 | 数值 | 单位 | 描述 |
|------|-----|------|------|
| ρ_water | 1025 | kg/m³ | 海水密度 |
| ρ_ice | 917 | kg/m³ | 冰密度 |
| g | 9.81 | m/s² | 重力加速度 |
| ν_0°C | 1.789e-6 | m²/s | 0°C 水的运动粘度 |
| E_ice | 9.0e9 | Pa | 冰的杨氏模量 |
| ν_ice | 0.33 | - | 冰的泊松比 |
| σ_yield | 1.0-3.0e6 | Pa | 冰的屈服强度 |

## 附录 B: 单位转换

| 转换 | 公式 |
|------|------|
| 节 → m/s | `m/s = knots × 0.514444` |
| m/s → 节 | `knots = m/s × 1.94384` |
| Pa → MPa | `MPa = Pa × 1e-6` |
| N → kN | `kN = N × 1e-3` |
| W → kW | `kW = W × 1e-3` |
| 马力 → kW | `kW = hp × 0.7457` |

---

**文档结束**
