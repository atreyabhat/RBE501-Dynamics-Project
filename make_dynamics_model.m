function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Link poses when the robot is in the home configuration

[M01, M12, M23, M34, M45, M56, M67, M78] = calculatelinkframes(robot);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67, M78);



%% Spatial Inertia Matrices
% *** Define the link inertial properties ***

m1 = 4;   % Mass of Link 1 [kg]
m2 = 4;   % Mass of Link 2 [kg]
m3 = 3;   % Mass of Link 3 [kg]
m4 = 2.7;
m5 = 1.7;
m6 = 1.8;
m7 = 0.3;


% Spatial Inertia Matrices
G1 = zeros(6,6); 
Ixx1 = 0.1 ;
Iyy1 = 0.09;
Izz1 = 0.02;
G1(1:3,1:3) = diag([Ixx1 Iyy1 Izz1]);
G1(4:6,4:6) = m1 * eye(3);

G2 = zeros(6,6);
Ixx2 = 0.05;
Iyy2 = 0.018;
Izz2 = 0.044;
G2(1:3,1:3) = diag([Ixx2 Iyy2 Izz2]);
G2(4:6,4:6) = m2 * eye(3);

G3 = zeros(6,6);
Ixx3 = 0.08;
Iyy3 = 0.075;
Izz3 = 0.01;
G3(1:3,1:3) = diag([Ixx3 Iyy3 Izz3]);
G3(4:6,4:6) = m3 * eye(3);

G4 = zeros(6,6);
Ixx4 = 0.03;
Iyy4 = 0.01;
Izz4 = 0.029;
G4(1:3,1:3) = diag([Ixx4 Iyy4 Izz4]);
G4(4:6,4:6) = m4 * eye(3);

G5 = zeros(6,6);
Ixx5 = 0.02;
Iyy5 = 0.018;
Izz5 = 0.005;
G5(1:3,1:3) = diag([Ixx5 Iyy5 Izz5]);
G5(4:6,4:6) = m5 * eye(3);

G6 = zeros(6,6);
Ixx6 = 0.005;
Iyy6 = 0.0036;
Izz6 = 0.0047;
G6(1:3,1:3) = diag([Ixx6 Iyy6 Izz6]);
G6(4:6,4:6) = m6 * eye(3);

G7 = zeros(6,6);
Ixx7 = 0.001;
Iyy7 = 0.001;
Izz7 = 0.001;
G7(1:3,1:3) = diag([Ixx7 Iyy7 Izz7]);
G7(4:6,4:6) = m7 * eye(3);

Glist = cat(3, G1, G2, G3, G4, G5, G6, G7);

end