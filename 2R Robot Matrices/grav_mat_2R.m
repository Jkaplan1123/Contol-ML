function G = grav_mat_2R(T, param)
%  G = grav_mat_2R(T, param)
%
% Calculates the Gravity Matrix at a particular time step. Currently 
% assumes a 2R robot with Lc = 1/2*L
%
% T is an 6x1 vector consisting of 2 3xM matrices stacked on top of each 
% other. Each 3x1 vector consists of the joint position, velocity, and 
% accleration at this time step for the corresponding joint.
%
% param is a vector consisting of the model parameter values: for this
% function it is important that param be defined as follows:
% param = [l1 l2 m1 m2 m3 g B r]
%
% G is the gravity matrix for the joint position, velocity, and
% acceleration values at this particular time step

l1 = param(1); l2 = param(2);
m1 = param(3); m2 = param(4); m3 = param(5);
g = param(6); B = param(7);

num_joints = size(T,1)/3;
t1 = T(1); t2 = T(4);


G = zeros(num_joints, 1);

G(1) = g*(m3*(l2*cos(t1 + t2) + l1*cos(t1))...
    + m2*((l2*cos(t1 + t2))/2 + l1*cos(t1)) + (l1*m1*cos(t1))/2);
 
G(2) = (g*l2*cos(t1 + t2)*(m2 + 2*m3))/2;
return
