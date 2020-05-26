%% Setup
close all; clear all; clc
addpath('2R Robot Matrices')

%% Get desired trajectories
ti = 0; tf = 20; dt = 0.001;

% Sine Sweep Trajectories
[t1, Q1] = traj_sin_sweep(ti, tf, dt, 0, deg2rad(90), 0, 1/20);
[t2, Q2] = traj_sin_sweep(ti, tf, dt, deg2rad(45), deg2rad(90), 0, 1/10);

%{
%% Get sine trajectory


t1 = ti:dt:tf;
t2 = t1;

u = 2*pi*t1;
du = 2*pi;
ddu = 0;

A = deg2rad(90);
q1 = 0 + A * sin(u);
d1q1 = A * cos(u) .* du;
d2q1 = A*(-sin(u).*(du.^2));

phi = pi/4;
q2 = deg2rad(45) + A * sin(u + phi);
d1q2 = A * cos(u + phi) .* du;
d2q2 = A*(cos(u + phi)*ddu - sin(u + phi).*(du.^2));

Q1 = [q1; d1q1; d2q1];
Q2 = [q2; d1q2; d2q2];

% figure
% plot(t1, Q1, t1, Q2)
% title('X Actual')
%}

if (t1 ~= t2)
    error('Time Vectors t1, t2 not equal')
end

T = [Q1; Q2]; %create theta matrix of states

l1 = 0.25; l2 = 0.125; r = 0.05;
m1 = 5; m2 = 3; m3 = 1;
g = 9.81; B = 1;

param = [l1 l2 m1 m2 m3 g B r];

%% Get torque values corresponding to the trajectory
Tau = inverse_dynamics(T, param);

%% Use Torque Values to Calculate the Trajectory

q1 = Q1(1,:); q2 = Q2(1,:); d1q1 = Q1(2,:); d1q2 = Q2(2,:);
X = [q1; q2; d1q1; d1q2];

d2q1 = Q1(3,:); d2q2 = Q2(3,:);
dQ = [d1q1; d1q2; d2q1; d2q2];
dX = zeros(4,size(X,2));
for idx = 1:size(dX,2)
    t = t1(idx);
    dX(:,idx) = joint_acceleration(t, X(:,idx), t1, Tau, param);
end

time = t1;
X0 = X(:,1);



options = odeset('RelTol',1e-6, 'AbsTol', 1e-10);
[tn, Xn] = ode45(@(t,y) joint_acceleration(t, y, time, Tau, param),...
    [time(1), time(end)], X0, options);
tn = tn'; Xn = Xn';

% [tn, Xn] = ode45(@(t,y) joint_acceleration(t, y, time, Tau, param),...
%     [time(1), time(end)], X0);
% tn = tn'; Xn = Xn';

Y = zeros(4,length(time));
for idx = 1:size(Y,1)
    Y(idx,:) = interp1(tn, Xn(idx,:), time);
end

%% Plot

figure
subplot(2,2,1)
hold on
plot(time, rad2deg(Y(1:2,:)))
plot(time, rad2deg(X(1:2,:)))
title('Position')
legend('ODE45 \theta_{1}', 'ODE45 \theta_{2}', 'Analytical \theta_{1}',...
    'Analytical \theta_{2}', 'Location','SouthWest')

subplot(2,2,3)
plot(time, rad2deg(Y(1:2,:)-X(1:2,:)))
title('ODE 45 Position Error')
legend('\theta_{1}', '\theta_{2}')

subplot(2,2,2)
hold on
plot(time, rad2deg(Y(3:4,:)))
plot(time, rad2deg(X(3:4,:)))
title('Velocity')
legend({'ODE45 $\dot{\theta}_{1}$', 'ODE 45 $\dot{\theta}_{2}$',...
    'Analytical $\dot{\theta}_{1}$', 'Analytical $\dot{\theta}_{2}$'},...
    'Interpreter', 'latex')

subplot(2,2,4)
plot(time, rad2deg(Y(3:4,:)-X(3:4,:)))
title('ODE 45 Velocity Error')
legend({'$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$'},...
    'Interpreter', 'latex')


figure
% subplot(2,1,1)
plot(tn,rad2deg(Xn))
title('ODE 45')
legend({'$\theta_{1}$', '$\theta_{2}$', '$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$'},'Interpreter', 'latex')
% subplot(2,1,2)
% plot(time,rad2deg(Y))
% title('Interpolated ODE45')

figure
plot(time,rad2deg(X))
title('X Actual')
legend({'$\theta_{1}$', '$\theta_{2}$', '$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$'},'Interpreter', 'latex')


figure
plot(time,rad2deg(dX))
title('Calculated dX')
legend({'$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', '$\ddot{\theta}_{1}$', '$\ddot{\theta}_{2}$'},'Interpreter', 'latex')


figure
plot(time, rad2deg(dX - dQ))
title('Error betweeen calculated and analytical derivative')
legend({'$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', '$\ddot{\theta}_{1}$', '$\ddot{\theta}_{2}$'},'Interpreter', 'latex')


figure
plot(time, Tau)
title('Joint Torques')
legend('\theta_{1}', '\theta_{2}')
