clc
clear all
close all

% Adjustable parameters
beta_01_y = 30;     
beta_02_y = 300;     
beta_03_y = 1000;   
b_0_y = 0.04;  % Compensation factor determining the compensation strength
beta_1_y = 200;   
beta_2_y = 120;   

beta_01_p = 30;     
beta_02_p = 300;     
beta_03_p = 1000;   
b_0_p = 0.4;  % Compensation factor determining the compensation strength
beta_1_p = 200;   
beta_2_p = 120;   

beta_01_r = 30;     
beta_02_r = 300;     
beta_03_r = 1000;   
b_0_r = 0.4;  % Compensation factor determining the compensation strength
beta_1_r = 200;   
beta_2_r = 120;   

%% Quadrotor Dynamics Modeling

% Quadrotor parameters
k_tn = 0.0036;
k_tc = -0.0036;
k_f = 0.1188;
J_y = 0.1104;
J_p = 0.0552;
J_r = 0.0552;
l = 0.197;

% State-space matrices
A = [0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];

B = [0,0,0,0;0,0,0,0;0,0,0,0;k_tc/J_y,k_tc/J_y,k_tn/J_y,k_tn/J_y;l*k_f/J_p,-l*k_f/J_p,0,0;0,0,l*k_f/J_r,-l*k_f/J_r];

C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];

D = [0,0,0,0;0,0,0,0;0,0,0,0];

sys = ss(A, B, C, D); % State-space model

gs = tf(sys); % Transfer function representation

% Yaw channel parameters
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

% Open-loop transfer function Gol(s)
gs_ol = tf([3261*beta_03 + 3261*b_0*beta_01*beta_1 + 3261*b_0*beta_02*beta_2, 3261*b_0*beta_1*beta_02 + 3261*b_0*beta_2*beta_03, 3261*b_0*beta_1*beta_03], [100000*b_0, 100000*beta_2*b_0^2 + 100000*beta_01*b_0, 100000*b_0*beta_02 + 100000*b_0^2*beta_1 + 100000*b_0^2*beta_01*beta_2, 0, 0, 0]);
K_star = 87.39 / 0.04; % Open-loop root locus gain
gs_ol_nor = tf([1, 2.687, 2.985], [1, 34.8, 452, 0, 0, 0]); % gs_ol = K_star * gs_ol_nor

[Wn, Zeta, Pole] = damp(gs_ol); % Natural frequency, damping ratio, and poles of the open-loop transfer function
Zero = zero(gs_ol); % Zeros of the open-loop transfer function

% Closed-loop transfer function Gcl(s)
gs_cl = tf([3261*b_0*beta_1, 3261*b_0*beta_01*beta_1, 3261*b_0*beta_1*beta_02, 3261*b_0*beta_1*beta_03], [100000*b_0, 100000*beta_2*b_0^2 + 100000*beta_01*b_0, 100000*b_0*beta_02 + 100000*b_0^2*beta_1 + 100000*b_0^2*beta_01*beta_2, 3261*beta_03 + 3261*b_0*beta_01*beta_1 + 3261*b_0*beta_02*beta_2, 3261*b_0*beta_1*beta_02 + 3261*b_0*beta_2*beta_03, 3261*b_0*beta_1*beta_03]);

%% Simulation
sim('Attitude_Control_2nd_Linear_ADRC_Simulink');

%% Simulation Results Plot
RTD = 180 / pi; % rad to degree

figure('name','Yaw Angle')
plot(t,v0_yaw * RTD,'b:',t,yaw,'r-','LineWidth',2);
xlabel('time(s)');
ylabel('yaw angle(бу)');
legend('Desired','Actual');
grid on;
title('Yaw Angle');

figure('name','Pitch Angle')
plot(t,v0_pitch * RTD,'b:',t,pitch,'r-','LineWidth',2);
xlabel('time(s)');
ylabel('pitch angle(бу)');
legend('Desired','Actual');
grid on;
title('Pitch Angle');

figure('name','Roll Angle')
plot(t,v0_roll * RTD,'b:',t,roll,'r-','LineWidth',2);
xlabel('time(s)');
ylabel('roll angle(бу)');
legend('Desired','Actual');
grid on;
title('Roll Angle');

figure('name','Voltage')
plot(t,v(:,1),'b:',t,v(:,2),'r-',t,v(:,3),'g-.',t,v(:,4),'m--','LineWidth',2);
xlabel('time(s)');
ylabel('voltage(V)');
legend('v_f','v_b','v_r','v_l');
grid on;
title('Voltage');

%% Root Locus Analysis
figure('Name','Root Locus')
rlocus(gs_ol_nor);

%% Stability Margins

% Nyquist plot
figure('Name','Nyquist Plot')
nyquist(gs_ol);

% Bode plot
figure('Name','Bode Plot')
bode(gs_ol);

% Gain and phase margins
S = allmargin(gs_ol);
Gm = S.GainMargin;
Pm = S.PhaseMargin;
Gm_1_dB = 20*log10(Gm(1));
Gm_2_dB = 20*log10(Gm(2));

[Gm1,Pm1,Wgm,Wpm] = margin(gs_ol);

%% Handling Quality Analysis

% Bode plot
figure('Name','Bode Plot of the Closed Loop System') % Bode plot of the closed loop system
bode(gs_cl);

% bandwidth
bw = bandwidth(gs_cl);

%% Time-Domain Characteristics

[Wn1, Zeta1, Pole1] = damp(gs_cl); % Natural frequency, damping ratio, and poles of the closed-loop transfer function
Zero1 = zero(gs_cl); % Zeros of the closed-loop transfer function

T = 1 / Wn1(1); % Time constant of the first-order system
omega_n = Wn1(3); % Natural frequency of the underdamped system
zeta = Zeta1(3); % Damping ratio of the underdamped system

% Rise time
tr_1 = 2.2 * T;
tr_2 = (pi - acos(zeta))/(omega_n * sqrt(1 - zeta ^2));

% Peak time
tp_2 = pi / (omega_n * sqrt(1 - zeta ^2));

% Overshoot
sigma_2 = exp(-pi*zeta/sqrt(1-zeta^2)); 

% Settling time
ts_1 = 3 * T;
ts_2 = 3.5/(zeta * omega_n);