# IGNAV-Java 使用文档

## 1. 环境要求

- JDK 17+
- Maven 3.6+

## 2. 构建安装

`ash
# 编译
mvn compile

# 运行测试
mvn test

# 打包
mvn package -DskipTests

# 安装到本地仓库
mvn install -DskipTests
`

## 3. Maven 依赖

`xml
<dependency>
    <groupId>org.gnss</groupId>
    <artifactId>ignav-java</artifactId>
    <version>1.0.0</version>
</dependency>
`

## 4. 快速开始

### 4.1 创建 INS/GNSS 实例

`java
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.insgnss.InsGnss;
import org.gnss.ignav.constants.IgnavConstants;

InsOpt opt = new InsOpt();
opt.baopt = IgnavConstants.INS_BAEST;  // 估计加速度计零偏
opt.bgopt = IgnavConstants.INS_BGEST;  // 估计陀螺仪零偏
opt.hz = 100;                           // IMU 采样率
opt.lc = 1;                             // 启用松组合
opt.zvu = 1;                            // 启用 ZVU
opt.nhc = 1;                            // 启用 NHC

InsGnss ignav = new InsGnss(opt);
`

### 4.2 INS 初始化

#### 方式一：从 GNSS 定位结果初始化

`java
import org.gnss.ignav.adapter.GnssPositionResult;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Imud;

GnssPositionResult gnssResult = new GnssPositionResult();
gnssResult.time = new GTime(week, tow);
gnssResult.posEcef = new double[]{-2267749.234, 5009389.567, 3221290.123};
gnssResult.velEcef = new double[]{10.0, 20.0, 5.0};
gnssResult.status = GnssPositionResult.SolutionStatus.FIX;
gnssResult.numSat = 12;

Imud imu = new Imud();
imu.time = new GTime(gnssResult.time);
imu.gyro = new double[]{0.001, -0.002, 0.001};
imu.accl = new double[]{0.05, -0.03, 9.81};

int ret = ignav.initIns(gnssResult, imu);
`

#### 方式二：从 POS 文件读取并初始化

`java
import org.gnss.ignav.adapter.PosFileReader;
import java.util.List;

List<GnssPositionResult> results = PosFileReader.readPosFile("path/to/solution.pos");
if (!results.isEmpty()) {
    ignav.initIns(results.get(0), imu);
}
`

#### 方式三：双天线姿态初始化

`java
double[] rpy = {roll_deg * D2R, pitch_deg * D2R, yaw_deg * D2R};
ignav.initInsDualAnt(pos, vel, rpy, time, imu);
`

### 4.3 INS 机械编排

`java
ignav.updateIns(imuData);          // 正向
ignav.updateInsBackward(imuData);  // 逆向
`

### 4.4 松组合更新

`java
ignav.lcUpdate(imuData, gnssResult, upd);       // 单次
ignav.lcUpdate(imuData, gnssResults, upd);      // 多次
`

### 4.5 辅助约束

`java
ignav.zvu(imuData, staticFlag);   // 零速更新
ignav.zaru(imuData, staticFlag);  // 零角速率更新
ignav.nhc(imuData);               // 非完整约束
ignav.initOdo();                  // 初始化里程计
ignav.odo(odoData, imuData);      // 里程计辅助
`

### 4.6 获取结果

`java
InsState ins = ignav.getInsState();
GnssPositionResult result = ignav.getInsPositionResult();
boolean isMech = ignav.isInsMech();
`

## 5. 完整处理流程

`java
InsOpt opt = new InsOpt();
opt.baopt = IgnavConstants.INS_BAEST;
opt.bgopt = IgnavConstants.INS_BGEST;
opt.lc = 1; opt.zvu = 1; opt.nhc = 1;
InsGnss ignav = new InsGnss(opt);

List<GnssPositionResult> gnssResults = PosFileReader.readPosFile("gnss.pos");
ignav.initIns(gnssResults.get(0), imuList.get(0));

for (int i = 1; i < imuList.size(); i++) {
    Imud imu = imuList.get(i);
    ignav.updateIns(imu);

    GnssPositionResult gnss = findGnssByTime(gnssResults, imu.time);
    if (gnss != null) ignav.lcUpdate(imu, gnss, 1);

    int staticFlag = InsStaticDetect.detstaticGlrt(imuWindow, opt, pos);
    ignav.zvu(imu, staticFlag);
    ignav.nhc(imu);

    GnssPositionResult result = ignav.getInsPositionResult();
}
`

## 6. 与 rtklib_java 集成

`java
import org.gnss.ignav.adapter.RtklibAdapter;

GnssPositionResult result = RtklibAdapter.fromRtklibSol(rtklibSolObject);
GnssPositionResult result2 = RtklibAdapter.fromRtklibRtk(rtklibRtkObject);
ignav.initIns(result, imu);
`

## 7. 紧组合接口

`java
ignav.setRtkPosProvider(new MyRtkPosProvider());
ignav.tcUpdate(imuData, upd);
`

## 8. InsState 关键字段

| 字段 | 类型 | 说明 |
|------|------|------|
| re[3] | double[] | ECEF 位置 (m) |
| ve[3] | double[] | ECEF 速度 (m/s) |
| Cbe[9] | double[] | b系到e系方向余弦阵 |
| ba[3] | double[] | 加速度计零偏 |
| bg[3] | double[] | 陀螺仪零偏 |
| P[] | double[] | 状态协方差阵 |
| stat | int | INS 状态码 |
| time | GTime | 当前时刻 |

## 9. INS 状态码

| 常量 | 值 | 说明 |
|------|-----|------|
| INSS_NONE/INIT | 0 | 无状态/初始化 |
| INSS_ALIGN | 1 | 对准中 |
| INSS_MECH | 2 | 机械编排 |
| INSS_LCUD | 3 | 松组合更新 |
| INSS_TCUD | 4 | 紧组合更新 |
| INSS_ZVU | 8 | 零速更新 |
| INSS_ZARU | 9 | 零角速率更新 |
| INSS_NHC | 10 | 非完整约束 |
| INSS_ODO | 11 | 里程计辅助 |
| INSS_RTS | 12 | RTS 平滑 |