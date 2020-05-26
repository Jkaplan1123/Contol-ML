function V = velocity_mat_2R(T, param)
% Calculates the Velocity Matrix at a particular time step. Currently 
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
% V is the velocity matrix for the joint position, velocity, and
% acceleration values at this particular time step

l1 = param(1); l2 = param(2);
m1 = param(3); m2 = param(4); m3 = param(5);
g = param(6); B = param(7);

num_joints = size(T,1)/3;
t1 = T(1); dt1 = T(2);
t2 = T(4); dt2 = T(5);


V = zeros(num_joints, 1);

V(1) = -(dt2*l1*l2*sin(t2)*(2*dt1 + dt2)*(m2 + 2*m3))/2;
 
V(2) = (dt1^2*l1*l2*sin(t2)*(m2 + 2*m3))/2;
return
