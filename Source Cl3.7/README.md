# 小型固定翼无人机飞行仿真系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows%20%7C%20macOS-green.svg)](https://github.com)

> 基于6-DOF动力学模型和级联PID控制的高保真固定翼无人机飞行仿真系统

## 项目简介

本项目实现了一个完整的小型固定翼无人机飞行仿真系统，提供高精度的6自由度飞行动力学建模和三级级联PID飞行控制。系统设计基于经典航空教材理论，适用于控制算法开发、飞行包线分析、任务规划验证等应用场景。

### 主要特性

- **高保真6-DOF动力学模型** - 完整的力学和气动建模
- **三级级联PID控制** - 外环/中环/内环分层控制架构
- **实时数据记录** - CSV格式飞行数据导出
- **航路点导航** - 自动任务规划和执行
-  **环境建模** - 风场和湍流干扰仿真
-  **完整测试套件** - 多种飞行场景验证
-  **模块化设计** - 易于扩展和集成

## 快速开始

### 环境要求

- **编译器**: 支持C++17的编译器 (GCC 7+, Clang 5+, MSVC 2017+)
- **操作系统**: Linux, Windows, macOS
- **依赖**: 标准C++库和数学库

### 编译安装

```bash
# 克隆仓库
git clone https://github.com/yourusername/uav-flight-simulation.git
cd uav-flight-simulation

# 编译
g++ -std=c++17 -O2 -o uav_sim src/uav_simulation.cpp -lm

# 运行测试
./uav_sim
```

### 基础使用示例

```cpp
#include "uav_simulation.h"

int main() {
    // 创建典型2kg固定翼无人机配置
    auto params = ConfigManager::createTypicalUAV();
    
    // 创建仿真系统
    AdvancedUAVSimulation sim(params, 0.01);  // 10ms步长
    sim.enableLogging(true);
    
    // 设置初始状态
    UAVState initial_state;
    initial_state.position = {0, 0, -100};    // 100m高度
    initial_state.velocity = {15, 0, 0};      // 15m/s巡航速度
    initial_state.attitude = {0, 0, 0};       // 水平姿态
    sim.setInitialState(initial_state);
    
    // 执行巡航飞行
    Commands commands;
    commands.desired_altitude = 120;   // 目标高度120m
    commands.desired_heading = 0.5;    // 目标航向30度
    commands.desired_airspeed = 18;    // 目标空速18m/s
    
    // 仿真100秒
    for (int i = 0; i < 10000; ++i) {
        sim.step(commands);
        
        if (i % 100 == 0) {  // 每秒输出状态
            const auto& state = sim.getCurrentState();
            printf("时间: %.1fs | 高度: %.1fm | 航向: %.1f° | 空速: %.1fm/s\n",
                   sim.getSimulationTime(), state.altitude, 
                   state.heading * 180 / M_PI, state.airspeed);
        }
    }
    
    // 导出飞行数据
    sim.exportData("flight_log.csv");
    return 0;
}
```

### 控制架构

- **外环控制** (20Hz): 航向/高度/空速 → 姿态指令
- **中环控制** (50Hz): 姿态角 → 角速度指令  
- **内环控制** (100Hz): 角速度 → 舵面偏角

## 测试套件

运行内置测试套件验证系统功能：

```bash
./uav_sim
```

### 测试项目

| 测试名称 | 目的 | 持续时间 | 输出文件 |
|---------|------|----------|----------|
| 直线巡航测试 | 验证基本控制精度 | 30秒 | `level_flight_test.csv` |
| 定点盘旋测试 | 验证转弯和定位能力 | 60秒 | `loiter_test.csv` |
| 航路点导航测试 | 验证任务执行能力 | 150秒 | `waypoint_navigation_test.csv` |
| 风扰动测试 | 验证抗干扰性能 | 60秒 | `wind_disturbance_test.csv` |
| 8字飞行测试 | 验证复杂机动能力 | 120秒 | `figure8_test.csv` |

## 技术规格

### 仿真性能
- **实时性**: 仿真步长 ≤ 0.01s，支持实时仿真
- **精度**: 航向/姿态/速度跟踪误差 ≤ 5%
- **稳定性**: 4阶Runge-Kutta积分保证数值稳定

### 无人机参数 (典型2kg固定翼)
- **质量**: 2.0 kg
- **翼展**: 1.5 m
- **机翼面积**: 0.3 m²
- **巡航速度**: 15-20 m/s
- **最大转弯率**: ~30°/s

### 控制性能
- **高度控制精度**: ±2 m
- **航向控制精度**: ±2°
- **空速控制精度**: ±1 m/s
- **最小转弯半径**: ~25 m

## 高级用法

### 自定义无人机配置

```cpp
// 创建自定义无人机参数
UAVParameters custom_params;
custom_params.mass = 3.5;                    // 3.5kg重型无人机
custom_params.wing_area = 0.5;               // 更大机翼面积
custom_params.aero.CL_alpha = 6.2;           // 不同的升力曲线斜率

// 使用自定义参数
AdvancedUAVSimulation sim(custom_params);
```

### 复杂任务规划

```cpp
// 创建航路点任务
std::vector<Waypoint> mission = {
    Waypoint(0, 0, 100, 15),           // 起点
    Waypoint(500, 0, 120, 20),         // 向东500m，爬升
    Waypoint(500, 500, 100, 18),       // 向北500m，下降
    Waypoint(0, 500, 110, 16),         // 向西返回
    Waypoint(0, 0, 100, 15)            // 回到起点
};

sim.setWaypoints(mission);

// 添加环境干扰
WindModel wind;
wind.setConstantWind(5.0, 2.0);       // 5m/s北风，2m/s东风
wind.setTurbulence(1.0);               // 1m/s湍流强度
sim.setWindModel(wind);

// 执行任务
while (!sim.isNavigationCompleted()) {
    sim.stepWithNavigation();
}
```

### 控制参数调优

```cpp
// 自定义PID控制器参数
CascadeController controller;

// 调整外环增益
controller.setHeadingGains(1.5, 0.02, 0.0);    // Kp, Ki, Kd
controller.setAltitudeGains(1.0, 0.008, 0.0);

// 调整内环增益  
controller.setRollRateGains(10.0, 0.0, 2.0);
controller.setPitchRateGains(12.0, 0.0, 2.5);
```

## 数据分析

### CSV数据格式

生成的CSV文件包含以下列：

```csv
时间,X位置,Y位置,Z位置,高度,U速度,V速度,W速度,空速,
滚转角,俯仰角,偏航角,航向,滚转率,俯仰率,偏航率,
油门,副翼,升降舵,方向舵,目标高度,目标航向,目标空速
```

### Python可视化脚本

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取仿真数据
data = pd.read_csv('flight_log.csv')

# 绘制飞行轨迹
plt.figure(figsize=(12, 8))
plt.plot(data['X位置'], data['Y位置'])
plt.xlabel('X位置 (m)')
plt.ylabel('Y位置 (m)')
plt.title('飞行轨迹')
plt.axis('equal')
plt.grid(True)
plt.show()
```

##  扩展开发

### 添加新的传感器模型

```cpp
class GPSModel {
public:
    UAVState addNoise(const UAVState& true_state) {
        UAVState noisy_state = true_state;
        // 添加GPS误差模型
        noisy_state.position[0] += gaussian_noise(0, 1.0);  // 1m标准差
        noisy_state.position[1] += gaussian_noise(0, 1.0);
        return noisy_state;
    }
};
```

### 集成外部接口

```cpp
// ROS节点接口示例
class ROSInterface {
public:
    void publishState(const UAVState& state);
    Commands subscribeCommands();
};

// MAVLink通信接口示例  
class MAVLinkInterface {
public:
    void sendHeartbeat();
    void sendAttitude(const UAVState& state);
    Commands receiveCommands();
};
```

## 理论背景

本系统基于以下经典教材和理论：

1. **Stevens, B.L. & Lewis, F.L.** - *Aircraft Control and Simulation* (2003)
2. **Beard, R.W. & McLain, T.W.** - *Small Unmanned Aircraft: Theory and Practice* (2012)
3. **Nelson, R.C.** - *Flight Stability and Automatic Control* (1998)

### 核心数学模型

**6-DOF动力学方程**:
```
位置: ṙ = R(φ,θ,ψ) · v
速度: v̇ = F/m + ω × v  
姿态: Ṙ = R · [ω]×
角速度: ω̇ = I⁻¹(M - ω × Iω)
```

**气动力模型**:
```
升力: L = ½ρV²S(CL₀ + CLₐα + CLδₑδₑ)
阻力: D = ½ρV²S(CD₀ + kCL²)
俯仰力矩: M = ½ρV²Sc̄(Cm₀ + Cmₐα + Cmδₑδₑ)
```

### 代码规范

- 使用C++17标准
- 遵循Google C++代码风格
- 添加详细的中文注释
- 包含单元测试

## 相关项目

- [JSBSim](https://github.com/JSBSim-Team/jsbsim) - 开源飞行动力学模型
- [ArduPilot](https://github.com/ArduPilot/ardupilot) - 开源自驾仪
- [PX4](https://github.com/PX4/PX4-Autopilot) - 专业级无人机固件
- [FlightGear](https://github.com/FlightGear/flightgear) - 开源飞行仿真器

