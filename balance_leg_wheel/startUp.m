clear;

%% cart-pole properties
global M m l g K_LQR
%% 定义小车倒立摆物理性质
R = 0.09;%0.0925;   %车轮的半径
D = 0.42;%0.55;     %左轮、右轮两个轮子间的距离
l = 0.088;%0.15;     %摆杆质心到转轴距离
m = 1.54;%0.88;     %车轮的质量
M = 18.23;%13;       %摆杆质量
I = (1/2)*m*R^2;    %车轮的转动惯量
Jz = (1/3)*M*l^2;    %机器人机体对 z 轴的运动时产生的转动惯量(俯仰方向)
Jy = (1/12)*M*D^2;    %机器人机体对 y 轴的运动时产生的转动惯量(偏航方向)
g = 9.8;      %重力加速度b = 1e-4;

% wheel_damping=1e-4;
% joint_damping=1e-4;

%% cart-pole initial condition
x_0=0;
y_0=0.4;
q_0=100;%angle

%% Constrollers
LQR=1;

if LQR
    K_LQR=cartPoleLQR
end