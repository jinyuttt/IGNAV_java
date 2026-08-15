# IGNAV-Java 项目介绍

## 1. 项目概述

IGNAV-Java 是一个 INS/GNSS 组合导航算法库，基于 C++ 开源项目 IGNAV 进行 Java 逐行转换实现。项目定位为**算法库**，提供 INS/GNSS 组合导航的核心算法接口，不包含 GUI 和文件 I/O 等软件层功能。

## 2. 核心功能

### 2.1 INS 机械编排
- 正向机械编排（Forward Mechanization）
- 逆向机械编排（Backward Mechanization）
- 支持 ECEF 和 N 系下的状态更新

### 2.2 INS 对准
- 粗对准（Coarse Alignment）：解析法
- 精对准（Fine Alignment）：EKF 滤波
- 双天线姿态辅助对准

### 2.3 INS/GNSS 组合
- 松组合（Loosely-Coupled）：位置/速度融合
- 紧组合（Tightly-Coupled）：观测值级融合
- 支持 EKF 滤波更新和闭环校正

### 2.4 辅助约束
- 零速更新（ZVU）：静态条件下的速度约束
- 零角速率更新（ZARU）：静态条件下的陀螺零偏约束
- 非完整约束（NHC）：车辆侧向和垂向速度为零
- 里程计辅助（ODO）：里程计速度观测融合

### 2.5 平滑算法
- RTS 平滑（Rauch-Tung-Striebel Smoother）
- 前后向平滑框架

### 2.6 静态检测
- GLRT（广义似然比检测）
- MV（均值方差检测）
- MAG（磁场检测）
- ARE（角速率检测）
- ODO（里程计辅助检测）

### 2.7 数据接口
- 标准 GNSS 定位结果输入（GnssPositionResult）
- POS 文件读取（rtklib 格式）
- rtklib_java 适配器
- 紧组合观测数据接口（GnssObservationProvider）

## 3. 技术选型

| 项目 | 选择 | 说明 |
|------|------|------|
| Java 版本 | 17 | LTS 版本 |
| 矩阵运算 | EJML 0.43.1 | 高性能 Java 矩阵库 |
| 日志 | SLF4J 2.0.9 + Logback 1.4.8 | 标准日志门面 |
| 测试 | JUnit 5.9.2 | 现代化测试框架 |
| 构建 | Maven | 标准构建工具 |

## 4. 项目结构

`
org.gnss.ignav
├── common/          通用数学工具（InsMath, Quaternion）
├── constants/       常量定义（IgnavConstants）
├── data/            数据结构（GTime, InsState, InsOpt 等 11 个类）
├── ins/             INS 核心算法（机械编排、对准、逆向编排）
├── insaux/          辅助约束（ZVU, ZARU, NHC, ODO, 静态检测）
├── insgnss/         INS/GNSS 组合（松组合、紧组合、RTS 平滑、门面类）
└── adapter/         外部接口适配（标准结果、POS 文件、rtklib 适配）
`

## 5. 与原项目 IGNAV 的关系

- 基于 IGNAV C++ 源码逐行转换，保持算法一致性
- 不包含原项目的流处理、文件输出、监控等软件层功能
- 新增标准化的 GNSS 结果输入接口和 POS 文件读取功能
- 新增门面类 InsGnss 简化调用流程

## 6. 与 rtklib_java 的关系

- 可复用 rtklib_java 的 SPP、RTK、坐标转换等模块
- 通过 RtklibAdapter 适配器桥接 rtklib_java 的定位结果
- 通过 GnssPositionResult 标准结构解耦，不硬依赖 rtklib_java