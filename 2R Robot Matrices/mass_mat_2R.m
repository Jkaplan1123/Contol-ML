function M = mass_mat_2R(T, param)
% Calculates the Mass Matrix at a particular time step. Currently assumes a
% 2R robot with Lc = 1/2*L
%
% Calculates the Mass Matrix at a particular time step. Currently 
% assumes a 2R robot with Lc = 1/2*L
%
% T is an 6x1 vector consisting of 2 3xM matrices stacked on top of each 
% other. Each 3x1 vector consists of the joint position, velocity, and 
% accleration at this time step for the corresponding joint..
%
% param is a vector consisting of the model parameter values: for this
% function it is important that param be defined as follows:
% param = [l1 l2 m1 m2 m3 g B r]
%
% M is the mass matrix for the joint position, velocity, and
% acceleration values at this particular time step

l1 = param(1); l2 = param(2);
m1 = param(3); m2 = param(4); m3 = param(5);
g = param(6); B = param(7); r = param(8);

num_joints = size(T,1)/3;
t1 = T(1); dt1 = T(2);
t2 = T(4); dt2 = T(5);


M = zeros(num_joints);

M(1,1) = (m1*(7*l1^2 + 3*r^2))/12 ...
    + (m2*(24*l1^2 + 14*l2^2 + 6*r^2 + 24*l1*l2*cos(t2)))/24 ...
    + m3*(l1^2 + l2^2 + 2*l1*l2*cos(t2));

M(1,2) = m3*(l2^2 + l1*l2*cos(t2))...
    + (m2*(14*l2^2 + 6*r^2 + 12*l1*l2*cos(t2)))/24;

M(2,1) = (m2*(14*l2^2 + 6*r^2 + 12*l1*l2*cos(t2)))/24 ...
    + l2*m3*(l2 + l1*cos(t2));

M(2,2) = (m2*(14*l2^2 + 6*r^2))/24 + l2^2*m3;
return
