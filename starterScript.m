clc; clear all; close all;

mdl_ur5

%% Robot params

N = 100;

HFUS25.J = 1.07e-4;
HFUS25.Tr = 157;       % Repeated peak torque
HFUS25.Ta = 108;       % Average torque
HFUS25.T1 = 14;
HFUS25.T2 = 48;
HFUS25.K3 = 57e3;
HFUS25.K2 = 50e3;
HFUS25.K1 = 31e3;

HFUS14.J = 0.091e-4;
HFUS14.Tr = 28;       % Repeated peak torque
HFUS14.Ta = 11;       % Average torque
HFUS14.T1 = 2;
HFUS14.T2 = 6.9;
HFUS14.K3 = 7.1e3;
HFUS14.K2 = 6.1e3;
HFUS14.K1 = 4.7e3;

%% Controllers

% PID controller
Kp = 40000;
Kd = 10;
Ki = 2000;

% LQR controller
B = 0.00064;
Jm = 0.091;
Jl = 0.01;
K = 4700;

A_c = [0 1 0 0; -K/Jl -B/Jl K/(Jl*N) 0; 0 0 0 1; K/(Jm*N) 0 -K/(Jm*N*N) -B/Jm];
B_c = [0 0 0 1/Jm]';
C_c = [1 0 0 0];

Q = [1 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
R = 1;
K_lqr = lqr(A_c, B_c, Q, R);

%% Trajectory

T = 5;
res = 100;

qn = [pi/2 -pi/2 pi/4 0 pi/6 0];
[q_traj,~,~] = jtraj(qz,qn,res);
q_desired = timeseries(q_traj,linspace(0, T, length(q_traj)));

x_traj = ur5.fkine(q_traj);

traj_visual = zeros(length(x_traj), 3);
for i=1:length(x_traj)
    traj_visual(i, :) = (x_traj(i).t)';
end