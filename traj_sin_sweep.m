function [t, Q] = traj_sin_sweep(ti, tf, dt, q0, A, w0, wf)
% [t, q] = traj_sin_sweep(ti, tf, dt, q0, A, w0, wf)
% 
% ti is a scalar value correponding to the initial time for the sine sweep
% 
% tf is a scalar value corresponding to the final time for the sine sweep
% 
% dt is the time increment from ti to tf. If (tf - ti)/dt is not a whole
% number, then the resulting time vector will get as close to tf as
% possible. Specically the time vector will be: 
% t = ti:dt:(tf - mod((tf-ti),dt))
% 
% q0 is the mean value about which the sine sweep oscillates
% 
% A is the amplitude of the sine sweep
% 
% w0 and wf are the initital and final frequencies. Suggested values are 0
% and 1 respectively
% 
% t is the time vector
% 
% Q is a 3xn matrix of a values corresponding to the values of sine sweep
% at all n time steps. Q consists of the row vector of the position values
% at each time step, the row vector of the velocity values at each time
% step, and the acceleration values at each time step. 
% 
% q = q0 + A*sin(2*pi*w(t)*t) where w(t) is linearlly intepolated between
% w0 and wf.


% Create a time vector
t = ti:dt:tf;

%  %Starting and ending frequencies
% f0 = 0;
% f1 = 1;

% Get frequency as a linear function of time w(t0)

m = (w0 - wf)/(ti - tf); %slope of w(t)

u = 2*pi*m*(t.^2);
du = 4*pi*m*t;
ddu = 4*pi*m;

q = q0 + A*sin(u);
dq = A .* cos(u) .* du;
ddq = A*(cos(u).*ddu - sin(u).*(du.^2));
Q = [q; dq; ddq];
return

