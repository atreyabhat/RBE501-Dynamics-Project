% RBE 501 - Robot Dynamics - Spring 2024
% Midterm Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/26/2024
clear, clc, close all
addpath('utils');

%% First, execute poe() to load the S and M matrices
n = 6; % number of degrees of freedom
robot = make_robot();
[S,M] = poe();
close all

robot.plot(zeros(1,n)), hold on;

%% Load the Path that the robot has to trace
load ik_data.mat
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

%% Solution

lambda = 0.1;
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
currentQ = zeros(1,n); 
qList = zeros(10,n);


for ii = 1 : 10
    % Generate the robot's pose
    currentTargetPose = targetPose(:,ii);
    %currentTargetPose
   
    while norm(currentTargetPose - currentPose) > 1e-3
        J = jacob0(S,currentQ);
        % Compute the error between the current and desired end-effector poses
        error = currentTargetPose - currentPose;

        %damped least sq
        J_star = J'*pinv((J*J' + lambda^2*eye(n))); 
        del_theta = J_star * error;
        currentQ = currentQ + del_theta';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
            
    end

    disp(T(1:3, 4));

    %robot.teach(currentQ);
    % drawnow;

    qList(ii, :) = currentQ';
    
end
    

%figure;
%robot.plot(qList)


%plot 
close all;
robot.plot(repmat([qList; flip(qList)], 10, 1),...
    'trail',{'r', 'LineWidth',5});







