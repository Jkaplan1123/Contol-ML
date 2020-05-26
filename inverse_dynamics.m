function Tau = inverse_dynamics(T, param)
% T is an (3*N)xM matrix where N is the number of joints and M is the
% number of time steps. T consists of N 3xM matrices stacked on top of each
% other. Each 3xM matrix consists of the joint position, velocity, and
% accleration at each time step for the corresponding joint.
%
% param is a vector consisting of the model parameter values: for this
% function it is important that param be defined as follows:
% param = [l1 l2 m1 m2 m3 g B r]
%
% Tau is the NxM matrix corresponding to the joint torque values for each
% joint at each time step

% q1 = T(1,:); d1q1 = T(2,:); d2q1 = T(3,:);
% q2 = T(4,:); d1q2 = T(5,:); d2q2 = T(6,:);
% 
% d2Q= [d2q1; d2q2];

addpath('2R Robot Matrices')


M = size(T,2); 
N = size(T,1)/3;

Tau = zeros(N,M);
for t = 1:M
    Theta = T(:,t);
    d2theta = [Theta(3); Theta(6)];
    d1theta = [Theta(2); Theta(5)];
    
    M = mass_mat_2R(Theta, param);
    V = velocity_mat_2R(Theta, param);
    G = grav_mat_2R(Theta, param);
    D = damp_mat_2R(T, param);
    
    Tau(:,t) = M*d2theta + V + G + D*d1theta;
    
    if (abs((d2theta - (inv(M)*(-1*(V+G+D*d1theta) + Tau(:,t))))) > 1e-10)
        error(['Something went wrong: ', 't = ' num2str(t)])
    end
    
end
return







