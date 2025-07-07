# Quadrotor-Attitude-Control
## Overview
+ Based on relevant literature, a quadrotor dynamic model is established. Considering the susceptibility of quadrotor attitude control to disturbances, an attitude control algorithm is designed using the Active Disturbance Rejection Control (ADRC) method to control the three-axis attitude changes. The performance of the designed control system is analyzed to achieve the goal of high-performance attitude control for the quadrotor<br>

  <br>

+ `Attitude_Control_2nd_Linear_ADRC.m` contains the modeling, control system design, and analysis code for the quadrotor aircraft. The corresponding Simulink model is available in `Attitude_Control_2nd_Linear_ADRC_Simulink.slx`<br><br>

+ `transfer_solve.m` includes the derivation of the transfer function for the second-order linear Active Disturbance Rejection Control (ADRC) system

## 1. Quadrotor Aircraft Modeling

A quadrotor hovering experimental platform manufactured by *Quanser*, with a weight of approximately a few kilogram, is used to analysis the attitude control method<br>

The state-space equation for a three-degree-of-freedom (attitude) quadrotor hovering system is formulated as<sup>[1]</sup> <br>

```math
\left[\begin{array}{c}\dot{y} \\ \dot{p} \\ \dot{r} \\ \ddot{y} \\ \ddot{p} \\ \ddot{r}\end{array}\right]=\left[\begin{array}{llllll}0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0\end{array}\right]\left[\begin{array}{c}y \\ p \\ r \\ \dot{y} \\ \dot{p} \\ \dot{r}\end{array}\right] + \left[\begin{array}{cccc}0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ \frac{k_{t, c}}{J_y} & \frac{k_{t, c}}{J_y} & \frac{k_{t, n}}{J_y} & \frac{k_{t, n}}{J_y} \\ \frac{l k_f}{J_p} & -\frac{l k_f}{J_p} & 0 & 0 \\ 0 & 0 & \frac{l k_f}{J_r} & -\frac{l k_f}{J_r}\end{array}\right]\left[\begin{array}{c}v_f \\ v_b \\ v_r \\ v_l\end{array}\right]
```
<br>

where:

+ $y​$ is the yaw angle, $p​$ is the pitch angle, and $r$​ is the roll angle

+ $v_f$, $v_b$, $v_r$, $v_l$ represent the control voltages applied to the front, rear, right, and left rotors, respectively

+ $K_{t, n}​$ is the thrust torque coefficient of the clockwise propeller, valued at $0.0036 \mathrm{~N} \cdot \mathrm{~m} / \mathrm{V}$

+ $K_{t, c}​$ is the thrust torque coefficient of the counterclockwise propeller, valued at $-0.0036 \mathrm{~N} \cdot \mathrm{~m} / \mathrm{V}$

+ $K_f​$ is the thrust coefficient of the propeller, valued at $0.1188 \mathrm{~N} / \mathrm{V}$

+ $J_y​$ is the moment of inertia about the yaw axis, valued at $0.1104 \mathrm{~kg} \cdot \mathrm{~m}^2$

+ $J_p$, $J_r​$ are the moments of inertia about the pitch and roll axes, both valued at $0.0552 \mathrm{~kg} \cdot \mathrm{~m}^2$

+ $l$ is the distance from the rotation center to the propeller center, valued at $0.197 m$

## 2. ADRC Control Law

The control law is designed based on the second-order linear ADRC method to facilitate subsequent analysis

### Linear Extended State Observer (LESO)

The ESO algorithm is implemented in continuous form:<br>

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

where $\beta_{01} \sim \beta_{03}$ are tunable parameters, and $e$ represents the observation error

Rewriting in state-space form:<br>

```math
\left[\begin{array}{c} \dot{z_1} \\ \dot{z_2} \\ \dot{z_3} \end{array}\right] = \left[\begin{array}{c} -\beta_{01} & 1 & 0 \\  -\beta_{02} & 0 & 1 \\ -\beta_{03} & 0 & 0 \end{array}\right] \left[\begin{array}{c} z_1 \\ z_2 \\ z_3 \end{array}\right] + \left[\begin{array}{c} 0 & \beta_{01} \\ b_0 & \beta_{02} \\ 0 & \beta_{03} \end{array}\right] \left[\begin{array}{c} u \\ y \end{array}\right] \ \ \ (2)
```

### Linear State Error Feedback Law (LSEF)

```math
\left\{\begin{array}{l}u_0=\beta_1 e1+\beta_2 e2 \\  u=u_0-z_3 / b_0 \end{array}\right. \ \ \ (3)
```

where:

+ $\beta_1, ~ \beta_2$ are tunable parameters; $e_1=r_1-z_1, e_2=r_2-z_2$ are system state errors. In this design method, the TD (Tracking Differentiator) is not used, therefore $r_2 = 0$ , the system attitude error is rewritten as $e_1=r-z_1, e_2=-z_2$ (As long as $\beta1 > \beta2$, making $e_1$ a more dominant control term than $e_2$, $r_2 = 0$ will not affect the control outcome)
+ The adjustable parameter $b_0$ is the "compensation factor" that determines the strength of the compensation

### System Transfer Function Derivation

Substituting Equation $(3)$ into Equation $(2)$ yields:<br>

```math
\left[\begin{array}{c} \dot{z_1} \\ \dot{z_2} \\ \dot{z_3} \end{array}\right] = \left[\begin{array}{c} -\beta_{01} & 1 & 0 \\  -\beta_{02} - b_0 \beta_1 & -b_0 \beta_2 & 1 \\ -\beta_{03} & 0 & 0 \end{array}\right] \left[\begin{array}{c} z_1 \\ z_2 \\ z_3 \end{array}\right] + \left[\begin{array}{c} 0 & \beta_{01} \\ b_0 \beta_1 & \beta_{02} \\ 0 & \beta_{03} \end{array}\right] \left[\begin{array}{c} r \\ y \end{array}\right]
```

The transfer function between the input signal and the reference signal is given by:<br>

$\frac{U}{R} = \frac{\beta_1 s^3 + \beta_{01} \beta_1 s^2 + \beta_1 \beta_{02} s + \beta_1 \beta_{03}}{s^3 + (\beta_{01} + b_0 \beta_2) s^2 + (\beta_{02} + b_0 \beta_1 + b_0 \beta_{01} \beta_2) s}$

The transfer function between the input signal and the output signal is given by:

$\frac{U}{Y} = \frac{(- \beta_{03} - b_0 \beta_{01} \beta_1 - b_0 \beta_{02} \beta_2) s^2 + (- b_0 \beta_1 \beta_{02} - b_0 \beta_2 \beta_{03}) s - b_0 \beta_1 \beta_{03}}{b_0 s^3 + (\beta_2 b_0^2 + \beta_{01} b_0) s^2 + (b_0 \beta_{02} + b_0^2 \beta_1 + b_0^2 \beta_{01} \beta_2)s}$

The second-order linear ADRC control system can be represented as follows:

![alt](/img/Control_System_Block_Diagram.png)

where

 $C_1(s) = \frac{b_0 \beta_1 s^3 + b_0 \beta_{01} \beta_1 s^2 + b_0 \beta_1 \beta_{02} s + b_0 \beta_1 \beta_{03}}{(\beta_{03} + b_0 \beta_{01} \beta_1 + b_0 \beta_{02} \beta_2) s^2 + (b_0 \beta_1 \beta_{02} + b_0 \beta_2 \beta_{03}) s + b_0 \beta_1 \beta_{03}}$

$C_2(s) = \frac{\beta_{03} + b_0 \beta_{01} \beta_1 + b_0 \beta_{02} \beta_2) s^2 + (b_0 \beta_1 \beta_{02} + b_0 \beta_2 \beta_{03}) s + b_0 \beta_1 \beta_{03}}{b_0 s^3 + (\beta_2 b_0^2 + \beta_{01} b_0) s^2 + (b_0 \beta_{02} + b_0^2 \beta_1 + b_0^2 \beta_{01} \beta_2) s}$

$G(s)$ represents the controlled plant

It is important to note that $C_1(s)$ is not a proper transfer function. Therefore, an additional pole needs to be introduced to facilitate simulation modeling:

$C_{11}(s) = C_1(s)C_{pole}(s) = C_1(s) \frac{1}{\frac{1}{f_p}s + 1}$ 

The selection of $f_p$ should ensure that the newly introduced pole frequency is significantly higher than the original pole frequency, so that the added pole does not affect the original transfer function. Here, $f_p = 1000$

## 3. Control System Establishment and Analysis

Based on the attitude dynamics model of the quadrotor, the state observer LESO, and the error feedback law LSEF, the following control system can be established:

(The control block diagram is constructed with reference to [shirunqi/Attitude-Control-of-Quadrotor-based-on-ADRC](https://github.com/shirunqi/Attitude-Control-of-Quadrotor-based-on-ADRC))

![alt](/img/Control_System_Block_Diagram_Simulink.png)

In the diagram, the $K*u​$ module converts the input attitude control commands into output voltage signals for the propeller motors

In this design, assuming the $3$ channels are independent, the quadrotor can perform attitude changes along $3$ axes simultaneously 

### Parameter Selection

+ YAW Channel

  $b_{0y}=0.04$

  LESO: $\beta_{01y}=30, \beta_{02y}=300, \beta_{03y}=1000$

  LSEF: $\beta_{1y}=200, \beta_{2y}=120$

+ PITCH Channel

  $b_{0p}=0.4$

  LESO: $\beta_{01p}=30, \beta_{02p}=300, \beta_{03p}=1000$

  LSEF: $\beta_{1p}=200, \beta_{2p}=120$

+ ROLL Channel

  $b_{0r}=0.4$

  LESO: $\beta_{01r}=30, \beta_{02r}=300, \beta_{03r}=1000$

  LSEF: $\beta_{1r}=200, \beta_{2r}=120$

The initial state of the system is set to $x_0=\left[0, 0, 0, 0,0,0\right]$

The input for each channel is a square wave signal with an amplitude of 3°, and a frequency of 0.1 Hz

### Control Results

![alt](/img/Yaw.png)

![alt](/img/Pitch.png)

![alt](/img/Roll.png)

![alt](/img/Voltage.png)

### Root Locus Analysis

The control system block diagram can be equivalently transformed as follows:

![alt](/img/Equivalent_Transformation_Block_Diagram.png)

The analysis is conducted for the **yaw channel**. The open-loop transfer function is obtained as:

$G_{ol} (s)= K^* \frac{s^2 + 2.687 s + 2.985}{s^5 + 34.8 s^4 + 452 s^3}$

where the open-loop root locus gain is $K^* = 2185$

The root locus plot is shown below:

![alt](/img/rlocus.png)

The gain values at the intersections of the root locus with the imaginary axis are 640 and 12564. When the open-loop root locus gain is within the range of 640 to 12564, the system remains stable

### Stability Margin Analysis

The Nyquist plot is shown below:

![alt](/img/nyquist.png)

The pole-zero distribution of the open-loop system is as follows:

![alt](/img/Open_Loop_System_Poles_Zeros.png)

The poles of the open-loop system are $P_1 = P_2 =P_3 =(0, 0, 0)，P_4 = (-17.4000, 12.2164)，P_5 = (-17.4000, -12.2164)$. Therefore, the open-loop system has no poles in the right half of the $s$ plane. The zeros of the open-loop system are $Z_1 = (-1.3433, 1.0866)$, $Z_2 = (-1.3433, -1.0866)$

Based on the Nyquist plot and the pole-zero distribution of the open-loop system, it can be observed that the curve does not encircle the point $(-1, 0)$. Thus, the number of encirclements is 0, which matches the number of poles of the open-loop system in the right half of the $s$ plane. Therefore, the corresponding closed-loop system is stable

The Bode plot is shown below:

![alt](/img/bode.png)

The gain margin is 0.29 and 5.75

Taking $\omega \in [0 , +\infty]​$ as an example, the corresponding curve in the Nyquist plot starts from the upper left and intersects the negative real axis at two points. The gain margin 0.29 indicates that if the open-loop magnitude frequency characteristic is increased by 0.29 times (i.e., reduced by approximately 3.4 times), the left intersection point will move to (-1,0). At this point, the Nyquist plot will just include (-1,0), and the system will be in a critically stable state. The gain margin 5.75 indicates that if the open-loop magnitude frequency characteristic is increased by 5.75 times, the right intersection point will move to (-1,0). At this point, the Nyquist plot will just include (-1,0), and the system will be in a critically stable state

Here, taking 5.75 as the gain margin (satisfying $10dB < h \left ( dB \right ) $)

The phase margin is 36.22° (satisfying $30^\circ < \gamma < 60^\circ$)

### Handling Quality Analysis

The handling quality standards are referenced from literature<sup>[2, 3]</sup> <br>

In this project, the quadrotor is in the low-speed state and the response-type is Attitude Command

#### 1. Small-Amplitude/High-Frequency Attitude Changes

The evaluation criteria for small-amplitude ($0 \sim 10°$)/high-frequency ($\ge 0.5Hz$ , not sure) attitude changes are bandwidth and phase delay. A quadrotor with a larger bandwidth and smaller phase delay can track high frequency command inputs more quickly, resulting in a better evaluation rating. The definitions of bandwidth and phase delay are as follows:

![alt](/img/Bandwidth_and_Phase_Delay.png)

The phase delay is $\tau_{\mathrm{p}}=\frac{\Delta \phi_{2 \omega_{180}}}{57.3 \left ( 2 \omega_{180} \right )}$

The Bode plot of the closed loop system is as follows:

![alt](/img/Bode_Plot_of_the_Closed_Loop_System.png)

From the Bode plot above, the bandwidth is $\omega_{BW} = \omega_{BW_{phase}} = 2.7 \ rad/s$ $(0.43Hz)$

The phase delay $\tau_{\mathrm{p}} =  0 \ sec$

Based on the grading figure, the small-amplitude/high-frequency attitude change grade of the closed-loop system is Level II

![alt](/img/Small_Amplitude_Attitude_Level_Diagram.png)

#### 2. Medium-Amplitude/Medium-Low-Frequency Attitude Changes

In *ADS-33*, medium-amplitude/medium-low-frequency attitude changes are often used to evaluate the 'attitude quickness' of a quadrotor (whether can achieve moderate attitude changes rapidly). What kind of attitude changes can be classified as medium-amplitude/medium-low-frequency? The required attitude changes shall be made as rapidly as possible from one steady heading to another and without significant reversals in the sign of the cockpit control input relative to the trim position

The evaluation criteria involves three variables: the peak attitude change $\Delta \psi_{p k}$ , the minimum attitude change $\Delta \psi_{\min }$ , and the peak angular velocity $r_{p k}$. The evaluation criteria is defined as the ratio of the peak angular velocity to the maximum attitude change after a medium-amplitude/medium-low-frequency attitude change, i.e., $\frac{r_{p k}}{\Delta \psi_{p k}}$. For a medium-amplitude/medium-low-frequency attitude change, a larger $r_{p k}$ , a smaller ${\Delta \psi_{p k}}$ and a smaller $\Delta \psi_{\min }$ will result in a better evaluation rating

Assuming a yaw attitude change of 30° , the yaw angle response and yaw rate response are shown below:

![alt](/img/Yaw_Response.png)

![alt](/img/Yaw_Rate.png)

(The unit of the vertical axis in the figure is based on radians)

The peak attitude change is $\Delta \psi_{p k} = 30.31°$

The minimum attitude change is $\Delta \psi_{\min } = 30°$

The peak angular velocity is $r_{p k} = 35.2°/s$

Thus,

$\frac{r_{p k}}{\Delta \psi_{p k}} = 1.16$

Based on the grading figure, the medium-amplitude/medium-low-frequency attitude change grade of the closed-loop system is Level I

![alt](/img/Medium_Amplitude_Attitude_Level_Diagram.png)

#### 3. Handing Quality Vertification

Although the input signal used in the main simulaztion is small-amplitude/medium-low-frequency, good experimental results can still be obtained by changing the input signal to small-amplitude/high-frequency or medium-amplitude/medium-low-frequency

![alt](/img/Small_Amplitude_High_Frequency_Attitude_Changes.png)

![alt](/img/Medium_Amplitude_Medium_Low_Frequency_Attitude_Changes.png)

### Time-Domain Characteristics Analysis

The pole distribution of the closed-loop system is shown below:

![alt](/img/CLosed_Loop_System_Poles.png)

The five poles are:

$P_1 = (-2.2927, 0)$

$P_2 = ( -1.9687, 2.7082) \ \ \ P_3 = (-1.9687, -2.7082)$

$P_4 = (-14.2850, 7.0506) \ \ \ P_5 = (-14.2850, -7.0506)$

The zeros of the closed-loop system are:

$Z_1 = Z_2 = Z_3 = (-10, 0)$

Since $P_4, \ P_5, \ Z_1, \ Z_2, \ Z_3$ are far from the imaginary axis, their impact on the system is negligible

The system response is primarily determined by the remaining real pole and the pair of complex conjugate poles

+ The real pole $P_1 = (-2.2927, 0)$, corresponding to a first-order system with a time constant $T = \frac{1}{w} = 0.436$

+ The complex conjugate poles are $P_2 = ( -1.9687, 2.7082) \ \ \ P_3 = (-1.9687, -2.7082)$, corresponding to an underdamped system with a natural frequency $w_{n} = 3.348$ and a damping ratio $\zeta = 0.588$ (According to *ADS-33*, $\zeta \ge 0.35$ satisfies good quality)

Under a step input:

![alt](/img/Step_Response.png)

+ Rise Time

  + First-order system rise time: $t_{r1} = 2.2 T = 0.96 \ s$

  + Underdamped system rise time: $t_{r2}=\frac{\pi-arccos\zeta}{\omega_d} = 0.81 \ s$

  + Actual system rise time: $t_r = 0.84 \ s$

+ Peak Time

  + The first-order system does not have a peak time

  + Underdamped system peak time: $t_{p2} = 1.16 \ s$

  + Actual system peak time: $t_p = 1.64 \ s$

+ Overshoot

  + The first-order system does not exhibit overshoot

  + Underdamped system overshoot: $\sigma_2 \\%=\mathrm{e}^{-\pi \zeta / \sqrt{1-\zeta^2}} \times 100 \\% = 10\\%$

  + Actual system overshoot: $\sigma \\%= 1\\%$ (generally required $1.5\\% < \sigma \\% < 25.4\\%$)

+ Settling Time

  + First-order system settling time: $t_{s1} = 3 T =1.31 \ s$

  + Underdamped system settling time: $t_{s2} = \frac{3.5}{\zeta \omega_n} = 1.78 \ s$

  + Actual system settling time: $t_s = 1.28 \ s$

（2）Steady-State Performance Metrics

The open-loop system has three poles at the origin, making it a Type III system. Therefore, theoretically, the system can track acceleration signals without error

![alt](/img/Acceleration_Signal_Input_Block_Diagram.png)

![alt](/img/Acceleration_Signal_Response_Error.png)

## References

[1] Li Yi, Chen Zengqiang, Liu Zhongxin. Attitude control of a quad-rotor robot based on ADRC [J]. Journal of Harbin Institute of Technology, 2014, 46(3):5.DOI:10.11918/j.issn.0367-6234.2014.03.020. (in Chinese)

[2] Research Report on Flight Quality Standards for Multi-Rotor Aircraft. Shenzhen Institute of Standards and Technology, 2020. (in Chinese)

[3] ADS-33E-PRF. United States Army Aviation and Missile Command (AMCOM) Aviation Engineering Directorate, Redstone Arsenal, Alabama, 2000.
