clc
clear all
close all

beta_01_y = 30;     % �ɵ�����
beta_02_y = 300;     % �ɵ�����
beta_03_y = 1000;   % �ɵ�����
b_0_y = 0.04;  % ��������ǿ���ġ��������ӡ�
beta_1_y = 200;   % �ɵ�����
beta_2_y = 120;   % �ɵ�����

beta_01_p = 30;     % �ɵ�����
beta_02_p = 300;     % �ɵ�����
beta_03_p = 1000;   % �ɵ�����
b_0_p = 0.4;       % ��������ǿ���ġ��������ӡ�
beta_1_p = 200;   % �ɵ�����
beta_2_p = 120;   % �ɵ�����

beta_01_r = 30;     % �ɵ�����
beta_02_r = 300;     % �ɵ�����
beta_03_r = 1000;   % �ɵ�����
b_0_r = 0.4;       % ��������ǿ���ġ��������ӡ�
beta_1_r = 200;   % �ɵ�����
beta_2_r = 120;   % �ɵ�����

%% ������ɻ�����ѧ��ģ

% ������ɻ�����
k_tn = 0.0036;
k_tc = -0.0036;
k_f = 0.1188;
J_y = 0.1104;
J_p = 0.0552;
J_r = 0.0552;
l = 0.197;

% ״̬�ռ����
A = [0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];

B = [0,0,0,0;0,0,0,0;0,0,0,0;k_tc/J_y,k_tc/J_y,k_tn/J_y,k_tn/J_y;l*k_f/J_p,-l*k_f/J_p,0,0;0,0,l*k_f/J_r,-l*k_f/J_r];

C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];

D = [0,0,0,0;0,0,0,0;0,0,0,0];

sys = ss(A, B, C, D); % ״̬�ռ�

gs = tf(sys); % ״̬�ռ��Ӧ���ݺ���

% ���ƫ��ͨ��
% ����
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

% �������ݺ��� Gol(s)
gs_ol = tf([3261*beta_03 + 3261*b_0*beta_01*beta_1 + 3261*b_0*beta_02*beta_2, 3261*b_0*beta_1*beta_02 + 3261*b_0*beta_2*beta_03, 3261*b_0*beta_1*beta_03], [100000*b_0, 100000*beta_2*b_0^2 + 100000*beta_01*b_0, 100000*b_0*beta_02 + 100000*b_0^2*beta_1 + 100000*b_0^2*beta_01*beta_2, 0, 0, 0]);
K_star = 87.39 / 0.04; % �������켣����
gs_ol_nor = tf([1, 2.687, 2.985], [1, 34.8, 452, 0, 0, 0]); % gs_ol = K_star * gs_ol_nor

[Wn, Zeta, Pole] = damp(gs_ol); % �������ݺ�����ȻƵ�ʣ�����ȣ�����
Zero = zero(gs_ol); % �������ݺ������

% �ջ����ݺ��� Gcl(s)
gs_cl = tf([3261*b_0*beta_1, 3261*b_0*beta_01*beta_1, 3261*b_0*beta_1*beta_02, 3261*b_0*beta_1*beta_03], [100000*b_0, 100000*beta_2*b_0^2 + 100000*beta_01*b_0, 100000*b_0*beta_02 + 100000*b_0^2*beta_1 + 100000*b_0^2*beta_01*beta_2, 3261*beta_03 + 3261*b_0*beta_01*beta_1 + 3261*b_0*beta_02*beta_2, 3261*b_0*beta_1*beta_02 + 3261*b_0*beta_2*beta_03, 3261*b_0*beta_1*beta_03]);

%% ���켣
figure('Name','Root Locus')
rlocus(gs_ol_nor);
% [k,poles]=rlocfind(gs_ol_nor);

%% �ȶ�ԣ��

% Nyquistͼ
figure('Name','Nyguist Plot')
nyquist(gs_ol);

% Bodeͼ
figure('Name','Bode Plot')
bode(gs_ol);

% ��ֵԣ�� ���ԣ��
S = allmargin(gs_ol);
Gm = S.GainMargin;
Pm = S.PhaseMargin;

[Gm1,Pm1,Wgm,Wpm] = margin(gs_ol);

%% ����Ʒ��

%% ʱ������

[Wn1, Zeta1, Pole1] = damp(gs_cl); % �ջ����ݺ�����ȻƵ�ʣ�����ȣ�����
Zero1 = zero(gs_cl); % �ջ����ݺ������

T = 1 / Wn1(1); % һ��ϵͳʱ�䳣��
omega_n = Wn1(3); % Ƿ����ϵͳ��ȻƵ��
zeta = Zeta1(3); % Ƿ����ϵͳ�����

% % ��Ծ��Ӧ
% step(gs_cl, 4);

% ����ʱ��
tr_1 = 2.2 * T;
tr_2 = (pi - acos(zeta))/(omega_n * sqrt(1 - zeta ^2));

% ��ֵʱ��
tp_2 = pi / (omega_n * sqrt(1 - zeta ^2));

% ������
sigma_2 = exp(-pi*zeta/sqrt(1-zeta^2)); 

% ����ʱ��
ts_1 = 3 * T;
ts_2 = 3.5/(zeta * omega_n);

%% ��ʼ����
sim('Attitude_Control_2nd_Linear_ADRC_Simulink');

%% ��ͼ
RTD = 180 / pi; % ����ת���ɽǶ�

figure('name','ƫ������ʱ��仯����')
plot(t,v0_yaw * RTD,'b:',t,yaw,'r-','LineWidth',2);
xlabel('ʱ��(s)');
ylabel('ƫ����(��)');
legend('����ֵ','ʵ��ֵ');
grid on;
title('ƫ������ʱ��仯����');

figure('name','��������ʱ��仯����')
plot(t,v0_pitch * RTD,'b:',t,pitch,'r-','LineWidth',2);
xlabel('ʱ��(s)');
ylabel('������(��)');
legend('����ֵ','ʵ��ֵ');
grid on;
title('��������ʱ��仯����');

figure('name','��ת����ʱ��仯����')
plot(t,v0_roll * RTD,'b:',t,roll,'r-','LineWidth',2);
xlabel('ʱ��(s)');
ylabel('��ת��(��)');
legend('����ֵ','ʵ��ֵ');
grid on;
title('��ת����ʱ��仯����');

figure('name','��ѹ��ʱ��仯����')
plot(t,v(:,1),'b:',t,v(:,2),'r-',t,v(:,3),'g-.',t,v(:,4),'m--','LineWidth',1.2);
xlabel('ʱ��(s)');
ylabel('��ѹ(V)');
legend('v_f','v_b','v_r','v_l');
grid on;
title('��ѹ��ʱ��仯����');