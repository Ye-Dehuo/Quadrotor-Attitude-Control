# 四旋翼飞机姿态控制
## 概述
+ `Attitude_Control_2nd_Linear_ADRC.m` 包含四旋翼飞机建模，控制系统建模与分析相关代码，Simulink 模型可见 `Attitude_Control_2nd_Linear_ADRC_Simulink.slx`<br><br>
+ `transfer_solve.m` 包含二阶线性ADRC系统的传递函数推导相关代码

## 1. 四旋翼飞机建模

建立三自由度( 姿态) 四旋翼盘旋系统的状态空间方程为<sup>[1]</sup> <br>
```math
\left[\begin{array}{c}\dot{y} \\ \dot{p} \\ \dot{r} \\ \ddot{y} \\ \ddot{p} \\ \ddot{r}\end{array}\right]=\left[\begin{array}{llllll}0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0\end{array}\right]\left[\begin{array}{c}y \\ p \\ r \\ \dot{y} \\ \dot{p} \\ \dot{r}\end{array}\right] + \left[\begin{array}{cccc}0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ \frac{k_{t, c}}{J_y} & \frac{k_{t, c}}{J_y} & \frac{k_{t, n}}{J_y} & \frac{k_{t, n}}{J_y} \\ \frac{l k_f}{J_p} & -\frac{l k_f}{J_p} & 0 & 0 \\ 0 & 0 & \frac{l k_f}{J_r} & -\frac{l k_f}{J_r}\end{array}\right]\left[\begin{array}{c}v_f \\ v_b \\ v_r \\ v_l\end{array}\right]
```
<br>

式中： 

$y​$为偏航角； $p​$为俯仰角； $r​$为滚转角

$v_f$, $v_b$, $v_r$, $v_l$ 分别为控制前，后，右，左 4 个旋翼转速的电压

$K_{t, n}​$为顺时针螺旋桨推力矩系数，其值为 $0.0036 \mathrm{~N} \cdot \mathrm{~m} / \mathrm{V}$

$K_{t, c}​$为逆时针螺旋桨推力矩系数，其值为 $-0.0036 \mathrm{~N} \cdot \mathrm{~m} / \mathrm{V}$

$K_f​$为螺旋桨推力系数，其值为 $0.1188 \mathrm{~N} / \mathrm{V}$

$J_y​$为偏航轴转动惯量，其值为 $0.1104 \mathrm{~kg} \cdot \mathrm{~m}^2$

$J_p$, $J_r​$分别为俯仰轴，滚转轴转动惯量，其值均为 $0.0552 \mathrm{~kg} \cdot \mathrm{~m}^2$

$l​$为旋转中心到螺旋桨中心的距离，其值为 $0.197 m$

## 2. ADRC控制律

基于二阶线性 ADRC 方法进行控制律设计，便于后续分析

### 线性扩张状态观测器 ( LESO )

采用连续形式的 ESO 算法<br>

```math
\left \{
\begin{align}
\dot{z_1} &= z_2 - \beta_{01} e, \\
\dot{z_2} &= z_3 - \beta_{02} e + b_0 u(t), \\
\dot{z_3} &= -\beta_{03} e.
\end{align}
\right.
\ \ \ (1)
```
<br>

$\beta_{01} \sim \beta_{03}$ 为可调参数； $e$ 为观测误差

将上式转换为状态空间表达形式<br>

```math
\left[\begin{array}{c} \dot{z_1} \\ \dot{z_2} \\ \dot{z_3} \end{array}\right] = \left[\begin{array}{c} -\beta_{01} & 1 & 0 \\  -\beta_{02} & 0 & 1 \\ -\beta_{03} & 0 & 0 \end{array}\right] \left[\begin{array}{c} z_1 \\ z_2 \\ z_3 \end{array}\right] + \left[\begin{array}{c} 0 & \beta_{01} \\ b_0 & \beta_{02} \\ 0 & \beta_{03} \end{array}\right] \left[\begin{array}{c} u \\ y \end{array}\right] \ \ \ (2)
```

### 线性状态误差反馈律 ( LSEF ) 

```math
\left\{\begin{array}{l}u_0=\beta_1 e1+\beta_2 e2 \\  u=u_0-z_3 / b_0 \end{array}\right. \ \ \ (3)
```

式中： $\beta_1, ~ \beta_2$ 为可调参数； $e_1=r_1-z_1, e_2=r_2-z_2$ 为系统状态误差，但鉴于可能存在的传感器局限问题，系统姿态误差改写为 $e_1=r-z_1, e_2=-z_2$；可调参数 $b_0$ 是决定补偿强弱的＂补偿因子＂

### 传递函数推导

将 $(3)$ 式代入 $(2)$ 式可得<br>

```math
\left[\begin{array}{c} \dot{z_1} \\ \dot{z_2} \\ \dot{z_3} \end{array}\right] = \left[\begin{array}{c} -\beta_{01} & 1 & 0 \\  -\beta_{02} - b_0 \beta_1 & -b_0 \beta_2 & 1 \\ -\beta_{03} & 0 & 0 \end{array}\right] \left[\begin{array}{c} z_1 \\ z_2 \\ z_3 \end{array}\right] + \left[\begin{array}{c} 0 & \beta_{01} \\ b_0 \beta_1 & \beta_{02} \\ 0 & \beta_{03} \end{array}\right] \left[\begin{array}{c} r \\ y \end{array}\right]
```

可得输入信号与参考信号之间的传递函数<br>

$\frac{U}{R} = \frac{\beta_1 s^3 + \beta_{01} \beta_1 s^2 + \beta_1 \beta_{02} s + \beta_1 \beta_{03}}{s^3 + (\beta_{01} + b_0 \beta_2) s^2 + (\beta_{02} + b_0 \beta_1 + b_0 \beta_{01} \beta_2) s}$

输入信号与输出信号之间的传递函数

$\frac{U}{Y} = \frac{- \beta_{03} - b_0 \beta_{01} \beta_1 - b_0 \beta_{02} \beta_2) s^2 + (- b_0 \beta_1 \beta_{02} - b_0 \beta_2 \beta_{03}) s - b_0 \beta_1 \beta_{03}}{b_0 s^3 + (\beta_2 b_0^2 + \beta_{01} b_0) s^2 + (b_0 \beta_{02} + b_0^2 \beta_1 + b_0^2 \beta_{01} \beta_2)s}$

可将二阶线性 ADRC 控制系统表示如下

![alt](/img/Control_System_Block_Diagram.png)

其中，

 $C_1(s) = \frac{b_0 \beta_1 s^3 + b_0 \beta_{01} \beta_1 s^2 + b_0 \beta_1 \beta_{02} s + b_0 \beta_1 \beta_{03}}{(\beta_{03} + b_0 \beta_{01} \beta_1 + b_0 \beta_{02} \beta_2) s^2 + (b_0 \beta_1 \beta_{02} + b_0 \beta_2 \beta_{03}) s + b_0 \beta_1 \beta_{03}}$

$C_2(s) = \frac{\beta_{03} + b_0 \beta_{01} \beta_1 + b_0 \beta_{02} \beta_2) s^2 + (b_0 \beta_1 \beta_{02} + b_0 \beta_2 \beta_{03}) s + b_0 \beta_1 \beta_{03}}{b_0 s^3 + (\beta_2 b_0^2 + \beta_{01} b_0) s^2 + (b_0 \beta_{02} + b_0^2 \beta_1 + b_0^2 \beta_{01} \beta_2) s}$

$G(s)$ 为被控对象

注意到 $C_1(s)$ 并不是真分式，因此给需要给其增加一个极点，便于仿真建模

$C_{11}(s) = C_1(s)C_{pole}(s) = C_1(s) \frac{1}{\frac{1}{f_p}s + 1}$ 

$f_p$ 的选值需确保新极点频率远远超出原极点频率，以保证添加的新极点不对原传递函数造成影响。此处选择 $f_p = 1000$

## 控制系统建立与分析

基于四旋翼飞机姿态动力学模型、状态观测器 LESO 与误差反馈律 LSEF 可建立如下控制系统

(控制框图搭建借鉴 [shirunqi/Attitude-Control-of-Quadrotor-based-on-ADRC](https://github.com/shirunqi/Attitude-Control-of-Quadrotor-based-on-ADRC))

![alt](/img/Control_System_Block_Diagram_Simulink.png)

图中 $K*u$ 模块可将输入的姿态控制指令转化为螺旋桨电机电压信号输出

参数选取：

+ YAW 通道

  $b_{0y}=0.04$

  LESO: $\beta_{01y}=30, \beta_{02y}=300, \beta_{03y}=1000$

  LSEF: $\beta_{1y}=200, \beta_{2y}=120$

+ PITCH 通道

  $b_{0p}=0.4$

  LESO: $\beta_{01p}=30, \beta_{02p}=300, \beta_{03p}=1000$

  LSEF: $\beta_{1p}=200, \beta_{2p}=120$

+ ROLL 通道

  $b_{0r}=0.4$

  LESO: $\beta_{01r}=30, \beta_{02r}=300, \beta_{03r}=1000$

  LSEF: $\beta_{1r}=200, \beta_{2r}=120$

系统初始值为 $x_0=\left[0, 0, 0, 0,0,0\right]$

每个通道的设定值均为：幅值为 3°，频率为 0. 1 Hz 的方波信号

控制结果如下

![alt](/img/Yaw.png)

![alt](/img/Pitch.png)

![alt](/img/Roll.png)

![alt](/img/Voltage.png)

### 根轨迹分析

对控制系统框图进行等效变换

![alt](/img/Equivalent_Transformation_Block_Diagram.png)

针对偏航通道进行分析。可得到开环传递函数

$G_{ol} (s)= K^* \frac{s^2 + 2.687 s + 2.985}{s^5 + 34.8 s^4 + 452 s^3}$

其中，开环根轨迹增益 $K^* = 2185$

绘制根轨迹图如下

![alt](/img/rlocus.png)

根轨迹与虚轴交点增益值为 640 和 12564，当开环根轨迹增益取值在 640~12564 区间内，系统皆能够稳定

### 稳定裕度分析

Nyquist 图如下

![alt](/img/nyquist.png)

开环传递函数零极点分布如下

![alt](/img/Open_Loop_System_Poles_Zeros.png)

开环系统极点为 $P_1 = P_2 =P_3 =(0, 0, 0)，P_4 = (-17.4000, 12.2164)，P_5 = (-17.4000, -12.2164)$，因此开环系统在右半 $s$ 平面内不存在极点。开环系统零点为 $Z_1 = (-1.3433, 1.0866)，Z_2 = (-1.3433, -1.0866)$

综合 Nyquist 图与开环系统零极点分布可看出，曲线将不会环绕 $(-1, 0)$ ，因此环绕次数为 0，与开环系统在右半 $s$ 平面含有的极点数一致，因此所对应的闭环系统稳定

Bode 图如下

![alt](/img/bode.png)

幅值裕度为 [0.29, 5.75]

以 $w \in [0 , +\infty]$ 为例，此时对应 Nyquist 图中从左上开始的曲线，该曲线与负实轴交于了 2 点。幅值裕度为 0.29 表示当开环幅频特性再增大 0.29 倍（即缩小约 3.4 倍）后，左侧交点会向右移动到 (-1,0) 处，此时  Nyquist 图即将包含 (-1,0)，系统会处于临界稳定状态；幅值裕度为 5.75 表示当开环幅频特性再增大 5.75 倍后，右侧交点会向左移动到 (-1,0) 处，此时  Nyquist 图即将包含 (-1,0)，系统会处于临界稳定状态

相角裕度为 36.22°

### 操纵品质分析

操纵品质标准借鉴文献<sup>[2]</sup>

（1）小幅/高频姿态变化

由 Bode 图可知，闭环系统带宽 $\omega_b = 6.8$

相位滞后 $\tau_{\mathrm{p}}=\frac{\Delta \phi_{2 \omega_{180}}}{2 \omega_{180}} = \frac{40}{38} = 1.1$

综合等级图，可得闭环系统的小幅/高频姿态变化等级为 I 级

![alt](/img/Small_Amplitude_Attitude_Level_Diagram.png)

（2）中幅/中低频姿态变化 

假设偏航姿态发生 30° 改变，偏航角响应与偏航角速度响应如下所示

![alt](/img/Yaw_Response.png)

![alt](/img/Yaw_Rate.png)

（图中纵轴单位基于弧度）

姿态变化量峰值为 $\Delta \theta_{p k} = 30.31°$

姿态变化量极小值为 $\Delta \theta_{\min } = 29.79°$

角速度峰值为 $q_{p k} = 35.2°/s$

可得

$\frac{q_{p k}}{\Delta \theta_{p k}} = 1.16$

综合等级图，可得闭环系统的中幅/中低频姿态变化等级为 II 级

![alt](/img/Medium_Amplitude_Attitude_Level_Diagram.png)

### 时域特性分析

闭环系统的极点分布如下

![alt](/img/CLosed_Loop_System_Poles.png)

5 个极点分别为

$P_1 = (-2.2927, 0)$

$P_2 = ( -1.9687, 2.7082) \ \ \ P_3 = (-1.9687, -2.7082)$

$P_4 = (-14.2850, 7.0506) \ \ \ P_5 = (-14.2850, -7.0506)$

闭环系统零点为

$Z_1 = Z_2 = Z_3 = (-10, 0)$

$P_4, \ P_5, \ Z_1, \ Z_2, \ Z_3$ 距离虚轴较远，不考虑其对于系统影响

系统响应主要由剩下的 1 个实数极点与 1 对共轭极点确定

实数极点为 $P_1 = (-2.2927, 0)$，对应一阶系统，时间常数 $T = \frac{1}{w} = 0.436$

共轭极点为 $P_2 = ( -1.9687, 2.7082) \ \ \ P_3 = (-1.9687, -2.7082)​$，对应欠阻尼系统，自然频率 $w_{n} = 3.348 ​$，阻尼比 $\zeta = 0.588​$

在输入为阶跃信号下

![alt](/img/Step_Response.png)

- 上升时间

  一阶系统上升时间 $t_{r1} = 2.2 T = 0.96 \ s$

  欠阻尼系统上升时间 $t_{r2}=\frac{\pi-arccos\zeta}{\omega_d} = 0.81 \ s$

  系统实际上升时间 $t_r = 0.84 \ s$

- 峰值时间

  一阶系统不具有峰值时间

  欠阻尼系统峰值时间 $t_{p2} = 1.16 \ s$

  系统实际峰值时间 $t_p = 1.64 \ s$

- 超调量

  一阶系统不具有超调量

  欠阻尼系统超调量 $\sigma_2 \\%=\mathrm{e}^{-\pi \zeta / \sqrt{1-\zeta^2}} \times 100 \\% = 10\\%$

  系统实际超调量 $\sigma \\%= 1\\%$

- 调节时间

  一阶系统调节时间 $t_{s1} = 3 T =1.31 \ s$

  欠阻尼系统调节时间 $t_{s2} = \frac{3.5}{\zeta \omega_n} = 1.78 \ s$

  系统实际调节时间 $t_s = 1.28 \ s$

（2）稳态性能指标

开环系统在坐标原点有 3 个极点， 所以系统属 III 型系统，因而理论上系统可以无误差跟踪加速度信号

![alt](/img/Acceleration_Signal_Input_Block_Diagram.png)

![alt](/img/Acceleration_Signal_Response_Error.png)

## 参考文献

[1] 李毅,陈增强,刘忠信.自抗扰技术在四旋翼飞行姿态控制中的应用[J].哈尔滨工业大学学报, 2014, 46(3):5.DOI:10.11918/j.issn.0367-6234.2014.03.020.

[2] 多旋翼飞行器飞行品质标准研究报告.深圳市标准技术研究院 , 2020.

