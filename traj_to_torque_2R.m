function traj_to_torque_2R

close all; clear all; clc

% Get desired trajectories
ti = 0; tf = 10; dt = 0.001;
[t1, Q1] = traj_sin_sweep(ti, tf, dt, 0, deg2rad(90), 0, 1/10);
[t2, Q2] = traj_sin_sweep(ti, tf, dt, deg2rad(45), deg2rad(90), 0, 1/2);

figure
plot(t1, Q1, t1, Q2)

if (t1 ~= t2)
    error('Time Vectors t1, t2 not equal')
end

T = [Q1; Q2]; %create theta matrix of states

l1 = 0.5; l2 = 0.25; r = 0.05;
m1 = 5; m2 = 3; m3 = 1;
g = 9.81; B = 0.1;

param = [l1 l2 m1 m2 m3 g B r];

Tau = inverse_dynamics(T, param);

figure
plot(t1, Tau)
return


