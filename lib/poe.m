function [S,M] = poe()
% POE Calculates the screw axes and the home configuration for the Motoman
% GP7 robot.
%
% Inputs: none
%
% Output: S - 6xn matrix representing the screw axes of the robot
%         M - Homoegeneous Transformation Matrix representing the home configuration
%
% RBE 501 - Robot Dynamics - Spring 2024
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/23/2024

%% Calculate the screw axes here:
S = [0 0 1 0 0 0;
     0 -1 0 0.33 0 -0.04;
     0 -1 0 0.775 0 -0.04;
     1 0 0 0 0.815 0;
     0 -1 0 0.815 0 -0.48
     1 0 0 0 0.815 0]';

%% Calculate the home configuration here:
M = [0 0 1 0.56; 0 -1 0 0; 1 0 0 0.815; 0 0 0 1];

end