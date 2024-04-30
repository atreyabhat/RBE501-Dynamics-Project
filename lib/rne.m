function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links

g = params.g;
S = params.S;
M = params.M;
G = params.G;
jointPos = params.jointPos;
jointVel = params.jointVel;
jointAcc = params.jointAcc;

n = size(S, 2);  % Number of joints

M_h_j = zeros(4,4,n);
M_h_j(:,:,1) = M(:,:,1);
for i = 2:n+1
    M_h_j(:,:,i) = M_h_j(:,:,i-1) * M(:,:,i);
end

A = zeros(6,n);
for i = 1:n
    A(:,i) = adjoint(M_h_j(:,:,i)) \ S(:,i);
end

T_i = zeros(4,4,n+1);
for j = 1:n
    T_iI = fkine(A(:,j), M(:,:,j), jointPos(j), 'body');
    T_i(:,:,j) = T_iI \ eye(4);
end
T_i(:,:,n+1) = M(:,:,n+1) \ eye(4);

V = zeros(6,n+1);
Vdot = zeros(6,n+1);

Vdot(:,1) = [0 0 0 -g(1) -g(2) -g(3)]';


% Forward iterations
for k = 1:n
    AdT = adjoint(T_i(:,:,k));
    V(:,k+1) = A(:,k) * jointVel(k) +  AdT * V(:,k);
    Vdot(:,k+1) = A(:,k) * jointAcc(k) + AdT * Vdot(:,k) + ad(V(:,k+1)) * A(:,k) * jointVel(k);
end

% Backward iterations
tau = zeros(n,1);
F = params.Ftip;

for i = n:-1:1
    Wi = G(:,:,i) * Vdot(:,i+1) - ad(V(:,i+1))' * G(:,:,i) * V(:,i+1) + adjoint(T_i(:,:,i+1))'*F;
    tau(i) = Wi' * A(:,i);
    F = Wi;
end


end
