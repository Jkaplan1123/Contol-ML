function D = damp_mat_2R(T, param)
% Calculates the Damping Matrix at a particular time step. Currently 
% assumes a 2R robot with Lc = 1/2*L
%
% T is an 6x1 vector consisting of 2 3xM matrices stacked on top of each 
% other. Each 3x1 vector consists of the joint position, velocity, and 
% accleration at this time step for the corresponding joint.
%
% param is a vector consisting of the model parameter values: for this
% function it is important that param be defined as follows:
% param = [l1 l2 m1 m2 m3 g B]
%
% B is the velocity matrix for the joint velocities at this particular
% time step


B = param(7);

num_joints = size(T,1)/3;

D = B*eye(num_joints);

return