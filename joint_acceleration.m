function dX = joint_acceleration(t, X0, time, Torque, param)

% Extract the derivative terms from the state vector
dX0 = [X0(3);X0(4)];

%Interoplate to get the torque Tau at this particular moment

num_inputs = size(Torque,1);
Tau = zeros(num_inputs, 1);
for idx = 1:num_inputs
    Tau(idx) = interp1(time, Torque(idx,:), t);
end

T = [X0(1); X0(3); 0; X0(2); X0(4); 0];

[M,V,G,D] = get_2R_matrices(T,param);

Mi = M\eye(size(M)); %Get the inverse of M
dX = zeros(4,1);

%Get the dX vector
dX(1:2) = dX0;
dX(3:4) = -Mi*(V + G + D*dX0) + Mi*Tau;




%{
% debugging
q = zeros(2, 1);
for idx = 1:num_inputs
    q(idx) = interp1(time, Q(3*idx,:), t);
end




time_idx = find(t==time);

[M2,V2,G2,D2] = get_2R_matrices(Q(:,time_idx), param);
d2theta = inv(M)*(-1*(V+G+D*dX0) + Torque(:,time_idx))

right = d2theta;
left = dX(3:4);

dM = M2 - M;
dV = V2 - V;
dG = G2 - G;

if (abs((left - right)) > 1e-3)
    keyboard
end
%}

return

function [M,V,G,D] = get_2R_matrices(T, param)
M = mass_mat_2R(T, param);
V = velocity_mat_2R(T,param);
G = grav_mat_2R(T, param);
D = damp_mat_2R(T, param);
return