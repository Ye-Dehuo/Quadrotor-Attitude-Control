clc
clear all
close all

beta_01_y = 30;     % 可调参数
beta_02_y = 300;     % 可调参数
beta_03_y = 1000;   % 可调参数
b_0_y = 0.04;  % 决定补偿强弱的“补偿因子”
beta_1_y = 200;   % 可调参数
beta_2_y = 120;   % 可调参数

beta_01_p = 30;     % 可调参数
beta_02_p = 300;     % 可调参数
beta_03_p = 1000;   % 可调参数
b_0_p = 0.4;       % 决定补偿强弱的“补偿因子”
beta_1_p = 200;   % 可调参数
beta_2_p = 120;   % 可调参数

beta_01_r = 30;     % 可调参数
beta_02_r = 300;     % 可调参数
beta_03_r = 1000;   % 可调参数
b_0_r = 0.4;       % 决定补偿强弱的“补偿因子”
beta_1_r = 200;   % 可调参数
beta_2_r = 120;   % 可调参数

%% 四旋翼飞机动力学建模

% 四旋翼飞机参数
k_tn = 0.0036;
k_tc = -0.0036;
k_f = 0.1188;
J_y = 0.1104;
J_p = 0.0552;
J_r = 0.0552;
l = 0.197;

% 状态空间矩阵
A = [0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];

B = [0,0,0,0;0,0,0,0;0,0,0,0;k_tc/J_y,k_tc/J_y,k_tn/J_y,k_tn/J_y;l*k_f/J_p,-l*k_f/J_p,0,0;0,0,l*k_f/J_r,-l*k_f/J_r];

C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];

D = [0,0,0,0;0,0,0,0;0,0,0,0];

sys = ss(A, B, C, D); % 状态空间

gs = tf(sys); % 状态空间对应传递函数

% 针对偏航通道
% 参数
beta_01 = 30; 
beta_02 = 300;    
beta_03 = 1000; 
b_0 = 0.04;  
beta_1 = 200;  
beta_2 = 120;   
fp = 1e3;

% C1(s)
gs_c1 = tf([b_0*beta_1, b_0*beta_01*beta_1, b_0*beta_1*beta_02, b_0*beta_1*beta_03], [(beta_03 + b_0*beta_01*beta_1 + b_0*beta_02*beta_2), (b_0*beta_1*beta_02 + b_0*beta_2*beta_03), b_0*beta_1*beta_03]);

% C2(s)
gs_c2 = tf([(beta_03 + b_0*beta_01*beta_1 + b_0*beta_02*beta_2), (b_0*beta_1*beta_02 + b_0*beta_2*beta_03), b_0*beta_1*beta_03], [b_0, (beta_2*b_0^2 + beta_01*b_0), (b_0*beta_02 + b_0^2*beta_1 + b_0^2*beta_01*beta_2), 0]);

% C11(s)
gs_c11 = tf([b_0*beta_1*fp, b_0*beta_01*beta_1*fp, b_0*beta_1*beta_02*fp, b_0*beta_1*beta_03*fp], [beta_03 + b_0*beta_01*beta_1 + b_0*beta_02*beta_2, beta_03*fp + b_0*beta_1*beta_02 + b_0*beta_2*beta_03 + b_0*beta_01*beta_1*fp + b_0*beta_02*beta_2*fp, b_0*beta_1*beta_03 + b_0*beta_1*beta_02*fp + b_0*beta_2*beta_03*fp, b_0*beta_1*beta_03*fp]);

gs_g = tf(0.03261, [1 0 0]); % G(s)

% 开环传递函数 Gol(s)
gs_ol = tf([3261*beta_03 + 3261*b_0*beta_01*beta_1 + 3261*b_0*beta_02*beta_2, 3261*b_0*beta_1*beta_02 + 3261*b_0*beta_2*beta_03, 3261*b_0*beta_1*beta_03], [100000*b_0, 100000*beta_2*b_0^2 + 100000*beta_01*b_0, 100000*b_0*beta_02 + 100000*b_0^2*beta_1 + 100000*b_0^2*beta_01*beta_2, 0, 0, 0]);
K_star = 87.39 / 0.04; % 开环根轨迹增益
gs_ol_nor = tf([1, 2.687, 2.985], [1, 34.8, 452, 0, 0, 0]); % gs_ol = K_star * gs_ol_nor

[Wn, Zeta, Pole] = damp(gs_ol); % 开环传递函数自然频率，阻尼比，极点
Zero = zero(gs_ol); % 开环传递函数零点

% 闭环传递函数 Gcl(s)
gs_cl = tf([3261*b_0*beta_1, 3261*b_0*beta_01*beta_1, 3261*b_0*beta_1*beta_02, 3261*b_0*beta_1*beta_03], [100000*b_0, 100000*beta_2*b_0^2 + 100000*beta_01*b_0, 100000*b_0*beta_02 + 100000*b_0^2*beta_1 + 100000*b_0^2*beta_01*beta_2, 3261*beta_03 + 3261*b_0*beta_01*beta_1 + 3261*b_0*beta_02*beta_2, 3261*b_0*beta_1*beta_02 + 3261*b_0*beta_2*beta_03, 3261*b_0*beta_1*beta_03]);

%% 根轨迹
figure('Name','Root Locus')
rlocus(gs_ol_nor);
% [k,poles]=rlocfind(gs_ol_nor);

%% 稳定裕度

% Nyquist图
figure('Name','Nyguist Plot')
nyquist(gs_ol);

% Bode图
figure('Name','Bode Plot')
bode(gs_ol);

% 幅值裕度 相角裕度
S = allmargin(gs_ol);
Gm = S.GainMargin;
Pm = S.PhaseMargin;

[Gm1,Pm1,Wgm,Wpm] = margin(gs_ol);

%% 操纵品质

%% 时域特性

[Wn1, Zeta1, Pole1] = damp(gs_cl); % 闭环传递函数自然频率，阻尼比，极点
Zero1 = zero(gs_cl); % 闭环传递函数零点

T = 1 / Wn1(1); % 一阶系统时间常数
omega_n = Wn1(3); % 欠阻尼系统自然频率
zeta = Zeta1(3); % 欠阻尼系统阻尼比

% % 阶跃响应
% step(gs_cl, 4);

% 上升时间
tr_1 = 2.2 * T;
tr_2 = (pi - acos(zeta))/(omega_n * sqrt(1 - zeta ^2));

% 峰值时间
tp_2 = pi / (omega_n * sqrt(1 - zeta ^2));

% 超调量
sigma_2 = exp(-pi*zeta/sqrt(1-zeta^2)); 

% 调节时间
ts_1 = 3 * T;
ts_2 = 3.5/(zeta * omega_n);

%% 开始仿真
sim('Attitude_Control_2nd_Linear_ADRC_Simulink');

%% 画图
RTD = 180 / pi; % 弧度转化成角度

figure('name','偏航角随时间变化曲线')
plot(t,v0_yaw * RTD,'b:',t,yaw,'r-','LineWidth',2);
xlabel('时间(s)');
ylabel('偏航角(°)');
legend('期望值','实际值');
grid on;
title('偏航角随时间变化曲线');

figure('name','俯仰角随时间变化曲线')
plot(t,v0_pitch * RTD,'b:',t,pitch,'r-','LineWidth',2);
xlabel('时间(s)');
ylabel('俯仰角(°)');
legend('期望值','实际值');
grid on;
title('俯仰角随时间变化曲线');

figure('name','滚转角随时间变化曲线')
plot(t,v0_roll * RTD,'b:',t,roll,'r-','LineWidth',2);
xlabel('时间(s)');
ylabel('滚转角(°)');
legend('期望值','实际值');
grid on;
title('滚转角随时间变化曲线');

figure('name','电压随时间变化曲线')
plot(t,v(:,1),'b:',t,v(:,2),'r-',t,v(:,3),'g-.',t,v(:,4),'m--','LineWidth',1.2);
xlabel('时间(s)');
ylabel('电压(V)');
legend('v_f','v_b','v_r','v_l');
grid on;
title('电压随时间变化曲线');