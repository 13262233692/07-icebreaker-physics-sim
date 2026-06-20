# 极地科考破冰船模拟仿真环境

## 项目简介

本项目是一款用于极地科考培训的破冰船模拟仿真环境，采用 Unity (C#) 作为核心引擎，并集成了自主开发的 C++ 原生物理引擎 DLL，用于实现高精度的船舶水动力学模拟和冰层破碎物理效果。

## 技术亮点

### 🔧 自定义 C++ 物理引擎
- **独立于 Unity 自带物理系统**，完全自主开发的刚体动力学引擎
- **GJK + EPA 碰撞检测算法**，实现精确的凸体碰撞检测和穿透深度计算
- **SAT 分离轴定理**，用于盒体碰撞快速检测
- **Voronoi 图算法 (Fortune 算法)**，实现冰层实时破碎效果
- **Lloyd 松弛优化**，提高碎块形状质量

### 🚢 船舶水动力学模拟
- **ITTC 1957 摩擦阻力公式**，精确计算船体摩擦阻力
- **弗劳德数与兴波阻力**，模拟船舶航行时的波浪阻力
- **Lindqvist 冰阻力公式**，计算船舶在冰区航行的阻力
- **螺旋桨推力模型**，考虑推进效率和船体效率
- **舵力与操纵性模型**，实现真实的船舶操纵响应
- **附加质量与惯性张量**，考虑水动力附加质量效应

### ❄️ 冰层破碎系统
- **实时压强分布计算**，船-冰接触时的法向压强分布
- **屈服强度判定**，基于冰的弯曲应力和剪切应力
- **Voronoi 切分**，撞击点附近冰层网格实时切分
- **碎块物理**，每个碎块作为独立刚体进行物理模拟
- **浮力模拟**，碎块的下沉和翻转行为

### 🔌 多层架构通信
- **P/Invoke 互操作层**，C# 与 C++ 无缝通信
- **回调事件系统**，C++ 向 C# 发送碰撞、破碎等事件
- **类型安全编组**，复杂数据结构的跨边界传递
- **调试日志系统**，多层级调试信息输出

## 项目结构

```
07-icebreaker-physics-sim/
├── IcePhysicsEngine/              # C++ 物理引擎 DLL
│   ├── CMakeLists.txt           # CMake 构建脚本
│   ├── include/                 # 头文件 (8个)
│   │   ├── IcePhysicsAPI.h      # DLL 导出接口
│   │   ├── MathUtils.h          # 数学工具库
│   │   ├── RigidBody.h          # 刚体系统
│   │   ├── Collider.h           # 碰撞器系统
│   │   ├── CollisionDetection.h # 碰撞检测
│   │   ├── Hydrodynamics.h      # 水动力学
│   │   ├── Voronoi.h            # Voronoi 图与冰层
│   │   └── PhysicsWorld.h       # 物理世界主类
│   └── src/                     # 源文件 (7个)
│       ├── RigidBody.cpp
│       ├── Collider.cpp
│       ├── CollisionDetection.cpp
│       ├── Hydrodynamics.cpp
│       ├── Voronoi.cpp
│       ├── PhysicsWorld.cpp
│       └── IcePhysicsAPI.cpp
│
├── UnityProject/                  # Unity 项目
│   ├── Assets/
│   │   ├── Plugins/           # 编译后的 DLL
│   │   └── Scripts/           # C# 脚本 (6个)
│   │       ├── IcePhysicsInterop.cs   # P/Invoke 绑定
│   │       ├── IcePhysicsManager.cs   # 物理引擎管理
│   │       ├── IcebreakerShip.cs      # 船舶控制
│   │       ├── IceSheet.cs            # 冰层模型
│   │       ├── IceFragment.cs         # 碎块脚本
│   │       └── DebugHUD.cs            # 调试界面
│   └── ProjectSettings/
│
├── Docs/                          # 文档
│   ├── MultiLayerArchitecture.md # 多层架构通信文档
│   └── BuildInstructions.md      # 编译构建说明
│
└── README.md
```

## 快速开始

### 1. 编译 C++ DLL

**前置要求**:
- Visual Studio 2019/2022 (含 C++ 桌面开发工作负载)
- CMake 3.15+

**编译步骤**:
```powershell
cd IcePhysicsEngine
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

编译成功后，DLL 会自动复制到 `UnityProject/Assets/Plugins/` 目录。

### 2. 配置 Unity 项目

1. 用 Unity Hub 打开 `UnityProject` 目录
2. 等待资源导入完成
3. 创建新场景，添加以下 GameObject:
   - `IcePhysicsManager` (物理引擎管理)
   - `IcebreakerShip` (破冰船)
   - `IceSheet` (冰层)
   - `DebugHUD` (调试界面)

详细步骤请参考 [Docs/BuildInstructions.md](Docs/BuildInstructions.md)

### 3. 运行仿真

1. 点击 Unity 的 Play 按钮
2. 使用 WASD 控制船舶:
   - **W/S**: 增加/减小油门
   - **A/D**: 左舵/右舵
3. 按 **F1** 切换调试 HUD 显示
4. 驾驶船舶撞击冰层，观察破碎效果

## 核心物理公式

### 牛顿-欧拉运动方程
```
线性: F = m · a
角运动: τ = I · α + ω × (I · ω)
```

### ITTC 摩擦阻力公式
```
Cf = 0.075 / (log10(Rn) - 2)²
Rf = 0.5 · ρ · V² · S · Cf
```

### 冰阻力 (Lindqvist 公式)
```
Rice = R_breaking + R_bending + R_crushing + R_submergence
```

### 弯曲应力判定
```
σ_bending = (6 · M) / (b · h²)
if σ_bending > σ_yield → 冰层破碎
```

## 调试与优化

### 调试级别
- `None`: 无输出
- `Error`: 仅错误
- `Warning`: 错误和警告
- `Info`: 重要运行信息 (默认)
- `Verbose`: 详细运行信息
- `Debug`: 调试细节 (影响性能)

### 性能目标
- 物理步进时间: < 16ms (60fps)
- 刚体数量: < 500
- 碎块数量: < 200

## 文档

- **[多层架构通信调试输出文档](Docs/MultiLayerArchitecture.md)**: 详细的系统架构、通信协议、数据流分析、API 参考
- **[编译与构建说明](Docs/BuildInstructions.md)**: 编译步骤、Unity 配置、常见问题排查

## 技术栈

| 层级 | 技术 |
|------|------|
| 游戏引擎 | Unity 2021.3 LTS+ |
| 游戏逻辑 | C# |
| 物理引擎 | C++17 |
| 构建系统 | CMake 3.15+ |
| 编译器 | MSVC v142/v143 |
| 互操作 | P/Invoke |

## 系统要求

### 最低配置
- **OS**: Windows 10 64-bit
- **CPU**: Intel Core i5-8400 / AMD Ryzen 5 2600
- **内存**: 8GB RAM
- **显卡**: NVIDIA GTX 1060 / AMD RX 580
- **磁盘**: 10GB 可用空间

### 推荐配置
- **OS**: Windows 11 64-bit
- **CPU**: Intel Core i7-10700K / AMD Ryzen 7 5800X
- **内存**: 16GB RAM
- **显卡**: NVIDIA RTX 3070 / AMD RX 6800
- **磁盘**: 10GB SSD

## 开发团队

Ice Physics Engineering Team

## 许可证

本项目用于极地科考培训教育目的。

---

**项目完成度**: ✅ 代码实现已完成  
**待完成**: ⏳ C++ DLL 编译 (依赖 Visual Studio Build Tools 安装)  
**文档**: ✅ 架构文档和编译说明已完成
