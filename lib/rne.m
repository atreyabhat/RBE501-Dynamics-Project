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
%
% Forward iterations
% YOUR CODE HERE 

n = size(params.S, 2);
g = params.g;
G = params.G;
M = params.M;
S = params.S;
q = params.jointPos;
qd = params.jointVel;
qdd = params.jointAcc;

A = zeros(6, n);
V = zeros(6,n+1);
Vdot = zeros(6,n+1);
Vdot(:,1) = [0 0 0 -params.g'];
tau = zeros(n,1);

% Compute A
Mx = zeros(4,4,n);
Mx(:,:,1) = M(:,:,1);
for ii = 2:n+1
    Mx(:,:,ii) = Mx(:,:,ii-1) * M(:,:,ii);
end

A = zeros(6,n);
for ii = 1:n
    A(:,ii) = adjoint(Mx(:,:,ii)) \ S(:,ii);
end

% compute TIi
T_Ii = zeros(4,4,n+1);
for ii = 1:n
    T_iI = fkine(A(:,ii), M(:,:,ii), q(ii), 'body');
    T_Ii(:,:,ii) = T_iI \ eye(4);
end
T_Ii(:,:,n+1) = M(:,:,n+1) \ eye(4);


for ii = 1:n
    AdT_Ii = adjoint(T_Ii(:,:,ii));
    V(:,ii+1) = A(:,ii) * params.jointVel(ii) + AdT_Ii * V(:,ii);
    Vdot(:,ii+1) = A(:,ii) * params.jointAcc(ii) + AdT_Ii * Vdot(:,ii) + ad(V(:,ii+1)) * A(:,ii) * params.jointVel(ii);
end
    



% Backward iterations

G = params.G;
F_next = params.Ftip;

for i = n:-1:1
    F = G(:,:,i) * Vdot(:,i+1) - ad(V(:,i+1))' * G(:,:,i) * V(:,i+1) + adjoint(T_Ii(:,:,i+1))' * F_next;
    tau(i) = F' * A(:,i);
    F_next = F;
end

end
