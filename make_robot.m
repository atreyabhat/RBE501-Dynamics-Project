function robot = make_robot()
%MAKE_ROBOT Creates the kinematic structure of the robot used in homework 5, problem 1.
%
%   This is a factory function that creates the robot used in the homework.
%
%   Inputs: none
%
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: L. Fichera <lfichera@wpi.edu>
%   Last modified: 4/05/2023

%% Create the manipulator
d1 = 0.360; % Lenght of Link 1 [m]
d3 = 0.429; % Lenght of Link 2 [m]
d5 = 0.400; % Lenght of Link 3 [m]
d7 = 0.126;

robot = SerialLink([Revolute('a', 0, 'd', d1, 'alpha', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', pi/2), ...
                    Revolute('a', 0, 'd', d3, 'alpha', pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2), ...
                    Revolute('a', 0, 'd', d5, 'alpha', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', pi/2), ...
                    Revolute('a', 0, 'd', d7, 'alpha', 0)], ...
                    'name', 'KUKA iiwa 14');
end