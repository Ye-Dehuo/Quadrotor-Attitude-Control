clc
clear all
close all

%% 二阶线性ADRC传递函数推导

% 符号定义
syms  beta_01 beta_02 beta_03 b_0 beta_1 beta_2 s fp;

% 状态空间模型
A = [
    -beta_01 1 0;
    -beta_02-b_0*beta_1 -b_0*beta_2 0;
    -beta_03 0 0
    ];

B = [0 beta_01;
    b_0*beta_1 beta_02;
    0 beta_03];

% 传递函数阵
F = inv(s * eye(3) - A);
Ans = collect(F * B); 

% U/R推导
y = 0; syms r;
z_1 = Ans(1,1) * r + Ans(1,2) * y;
z_2 = Ans(2,1) * r + Ans(2,2) * y;
z_3 = Ans(3,1) * r + Ans(3,2) * y;
Ans_1 = collect(beta_1 * (r - z_1) - beta_2 * z_2 - z_3/b_0, s);
U_R = (beta_1*s^3 + beta_01*beta_1*s^2 + beta_1*beta_02*s + beta_1*beta_03)/(s^3 + (beta_01 + b_0*beta_2)*s^2 + (beta_02 + b_0*beta_1 + b_0*beta_01*beta_2)*s);

% U/Y推导
r = 0; syms y;
z_1 = Ans(1,1) * r + Ans(1,2) * y;
z_2 = Ans(2,1) * r + Ans(2,2) * y;
z_3 = Ans(3,1) * r + Ans(3,2) * y;
Ans_2 = collect(beta_1 * (r - z_1) - beta_2 * z_2 - z_3/b_0, s);
U_Y = ((- beta_03 - b_0*beta_01*beta_1 - b_0*beta_02*beta_2)*s^2 + (- b_0*beta_1*beta_02 - b_0*beta_2*beta_03)*s - b_0*beta_1*beta_03)/(b_0*s^3 + (beta_2*b_0^2 + beta_01*b_0)*s^2 + (b_0*beta_02 + b_0^2*beta_1 + b_0^2*beta_01*beta_2)*s);

C_2 = - U_Y; % C2(s)
C_1 = collect(U_R / C_2, s); % C1(s)

C_pole = 1 / ((1 / fp) * s +1); % Cpole(s)
C_11 = collect(C_1 * C_pole, s); % C11(s)

G = 0.03261 / s^2; % G(s)

gs_ol = collect((C_2 * G), s); % 开环传递函数Gol(s)

gs_cl = collect((C_1 * C_2 * G) / (1 + C_2 * G),s); % 闭环传递函数Gcl(s)