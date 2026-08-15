# IGNAV-Java 技术参考文档

## 1. 坐标系定义

| 符号 | 名称 | 说明 |
|------|------|------|
| e 系 | ECEF | 地心地固坐标系 |
| n 系 | 导航系 | 北东地（NED）局部水平坐标系 |
| b 系 | 载体系 | 右前上（RFU）载体坐标系 |
| i 系 | 惯性系 | 地心惯性坐标系 |

## 2. 状态向量定义

### 2.1 基本状态（15 维）

| 索引 | 变量 | 说明 | 单位 |
|------|------|------|------|
| 0-2 | dphi | 姿态误差角 | rad |
| 3-5 | dvel | 速度误差 | m/s |
| 6-8 | dpos | 位置误差 | m |
| 9-11 | bg | 陀螺零偏 | rad/s |
| 12-14 | ba | 加速度计零偏 | m/s^2 |

### 2.2 扩展状态

当启用加速度计零偏估计（baopt=1）时，状态包含 ba；
当启用陀螺零偏估计（bgopt=1）时，状态包含 bg；
总维数 nx = 15 + baopt*3 + bgopt*3

状态索引由 InsGnssState 类管理：

`java
int iPhi = 0;           // 姿态误差起始索引
int iVel = 3;           // 速度误差起始索引
int iPos = 6;           // 位置误差起始索引
int iBg  = 9;           // 陀螺零偏起始索引（bgopt=1时有效）
int iBa  = 12;          // 加速度计零偏起始索引（baopt=1时有效）
`

## 3. INS 机械编排算法

### 3.1 正向机械编排（InsMech.insmech）

输入：上一时刻状态 ins_prev，当前 IMU 数据 imu
输出：当前时刻状态 ins

核心步骤：
1. 姿态更新：四元数积分
   - q(k+1) = q(k) * dq(w_ib^b * dt)
   - Cbe(k+1) = Cbe(k) * R(w_ib^b * dt)
2. 速度更新：比力积分
   - v(k+1) = v(k) + (Cbe * f^b + ge - 2*wie * v) * dt
3. 位置更新：速度积分
   - r(k+1) = r(k) + v * dt

### 3.2 逆向机械编排（InsBackMech.updateinsbn）

用于前后向平滑，从后向前推算 INS 状态。

## 4. EKF 滤波模型

### 4.1 状态转移矩阵 F

F 为 nx*nx 矩阵，包含：
- 姿态误差传播：-Cbe*[f^b]* + wie_n 等项
- 速度误差传播：Cbe*[ba] + 位置相关项
- 位置误差传播：速度相关项
- 陀螺/加计零偏：一阶马尔可夫过程

### 4.2 过程噪声矩阵 Q

对角矩阵，对角元素为：
- 陀螺角度随机游走（ARW）
- 加速度计速度随机游走（VRW）
- 陀螺零偏驱动噪声
- 加速度计零偏驱动噪声

### 4.3 闭环校正（clp）

EKF 更新后，将误差状态反馈到导航状态：
- 姿态校正：Cbe = (I - [dphi]) * Cbe
- 速度校正：ve = ve - dvel
- 位置校正：re = re - dpos
- 零偏校正：bg = bg - dbg, ba = ba - dba

## 5. 松组合观测模型（InsGnssLc）

### 5.1 观测向量

Z = [pos_gnss - pos_ins; vel_gnss - vel_ins]

6 维观测（位置 + 速度），或 3 维（仅位置）

### 5.2 观测矩阵 H

位置部分：H_pos = [0, 0, I, 0, 0]（3*nx）
速度部分：H_vel = [0, I, 0, 0, 0]（3*nx）

### 5.3 观测噪声矩阵 R

R = diag(pos_std^2, vel_std^2)，来自 GNSS 解算精度

## 6. 紧组合观测模型（InsGnssTc）

### 6.1 观测向量

Z = 伪距/载波相位观测值 - INS 预测伪距/载波相位

### 6.2 天线杆臂补偿

r_gnss = r_ins + Cbe * lever_arm

## 7. 辅助约束模型

### 7.1 ZVU（InsZvu）

观测方程：v = 0
H = [0, I, 0, 0, 0]（3*nx）
R = diag(sig^2)

### 7.2 ZARU（InsZaru）

观测方程：w = 0（仅零偏部分）
H = [I, 0, 0, 0, 0]（3*nx）

### 7.3 NHC（InsNhc）

观测方程：v_lateral = 0, v_vertical = 0
H = 侧向和垂向速度对状态的偏导数

### 7.4 ODO（InsOdo）

观测方程：v_forward = odo_velocity
H = 前向速度对状态的偏导数

## 8. RTS 平滑算法（InsRts）

### 8.1 前向滤波

标准 EKF 前向处理，保存每历元的：
- 状态向量 x_k
- 协方差 P_k
- 状态转移矩阵 Phi_k

### 8.2 后向平滑

从最后一个历元向前递推：
- x_s(k) = x_k + K_k * (x_s(k+1) - x_pred(k+1))
- K_k = P_k * Phi_k^T * P_pred(k+1)^-1
- P_s(k) = P_k + K_k * (P_s(k+1) - P_pred(k+1)) * K_k^T

### 8.3 有效性验证（valsmth）

检查平滑后协方差是否合理，防止发散。

## 9. 静态检测算法（InsStaticDetect）

### 9.1 GLRT 检测

广义似然比检验，基于加速度幅值方差：
- 统计量：T = |a_mean|^2 / sigma_a^2
- 判决：T < threshold → 静态

### 9.2 MV 检测

加速度幅值方差检测：
- 统计量：var(|a|) 在滑动窗口内
- 判决：var < threshold → 静态

### 9.3 ARE 检测

角速率能量检测：
- 统计量：|w|^2 在滑动窗口内
- 判决：|w|^2 < threshold → 静态

## 10. 数学工具函数（InsMath）

### 10.1 坐标转换

| 函数 | 功能 |
|------|------|
| ecef2pos | ECEF → LLH |
| pos2ecef | LLH → ECEF |
| xyz2enu | ECEF → ENU 旋转矩阵 |
| enu2ecef | ENU → ECEF |
| ecef2enu | ECEF 向量 → ENU |
| ned2xyz | NED 旋转矩阵 |

### 10.2 矩阵运算

| 函数 | 功能 |
|------|------|
| matmul | 通用矩阵乘法 |
| matmul33 | 3x3 矩阵乘法（支持转置） |
| matinv | 矩阵求逆（Gauss-Jordan） |
| matcpy | 矩阵拷贝 |
| matadd | 矩阵加减 |

### 10.3 姿态转换

| 函数 | 功能 |
|------|------|
| rpy2dcm | 横滚俯仰偏航 → DCM |
| dcm2rpy | DCM → 横滚俯仰偏航 |
| rot2dcm | 旋转向量 → DCM |
| quat2dcm | 四元数 → DCM |
| dcm2quat | DCM → 四元数 |
| quatmul | 四元数乘法 |

### 10.4 协方差转换

| 函数 | 功能 |
|------|------|
| covenu | ECEF 协方差 → ENU 协方差 |
| covecef | ENU 协方差 → ECEF 协方差 |

## 11. 常量定义（IgnavConstants）

| 常量 | 值 | 说明 |
|------|-----|------|
| RE_WGS84 | 6378137.0 | WGS84 长半轴 (m) |
| FE_WGS84 | 1/298.257223563 | WGS84 扁率 |
| OMGE | 7.2921151467e-5 | 地球自转角速率 (rad/s) |
| GRAV | 9.794412 | 标准重力 (m/s^2) |
| INS_BAEST | 1 | 估计加速度计零偏 |
| INS_BGEST | 1 | 估计陀螺零偏 |
| INSS_MECH | 2 | 机械编排状态 |
| INSS_LCUD | 3 | 松组合更新状态 |