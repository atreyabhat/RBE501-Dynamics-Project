function [flag, currentQ, t_acc, jointPos_acc, jointVel_actual, jointAcc_actual, tau_acc] = point2point(path, currentangles, Force)
    addpath('lib');

    % Create the environment
    g = [0 0 -9.81]; % Gravity Vector [m/s^2]

    % Create the robot and display it in the home configuration
    robot = make_robot();

    % Create a kinematic model of the robot
    [S,M] = make_kinematics_model(robot);

    n = size(S,2); % read the number of joints

    % Create a dynamical model of the robot
    [Mlist,Glist] = make_dynamics_model(robot);

    %% Control the motion of the robot between 2 set points
    fprintf('---------------------- Dynamic Control of a Kuka Arm --------------------\n');

    nPts = size(path,2);

    fprintf('Calculating the Inverse Kinematics... \n');

    % Calculate the inverse kinematics
    waypoints = zeros(n,nPts);

    % Current Pose and Joint Angles of Arm
    M = xyzrpy_to_transformation(path(:,1));
    currentQ = zeros(1,7);
    currentPose = MatrixLog6(M);
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

    % Set the iteration limit
    max_iterations = 25000;

    % Initialize the counter
    iteration = 0;
    lambda = 0.1;

    waypoints = zeros(n,nPts);
    waypoints(:, 1) = currentangles;
    for i = 2:size(path,2)
        % Desired end-effector position and orientation
        desiredPose = xyzrpy_to_transformation(path(:,2));
        desiredPose = MatrixLog6(desiredPose);
        desiredPose = [desiredPose(3,2) desiredPose(1,3) desiredPose(2,1) desiredPose(1:3,4)']';
        
        while norm(currentPose - desiredPose) > 1e-3
            % Check if the iteration limit has been reached
            if iteration >= max_iterations
                disp('Unable to Solve Inverse Kinematics');
                flag = false;
                [t_acc, jointPos_acc, jointVel_actual, jointAcc_actual, tau_acc] = deal(0, 0, 0, 0, 0);
                return;
            end
        
            % Calculate the Jacobian at the current joint configuration
            J_a = jacob0(S,currentQ);
        
            % Damped Least Squares
            J_pseudo = (J_a') * pinv(J_a * J_a' + (lambda^2)*eye(6));
            deltaQ = J_pseudo*(desiredPose - currentPose);
        
            % Update joint angles
            currentQ = currentQ + deltaQ';
        
            currentPose = MatrixLog6(double(robot.fkine(currentQ)));
            currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
        
            % Increment the counter
            iteration = iteration + 1;
        end
        if checklimits(currentQ) == 0
                disp('Unable to Solve Inverse Kinematics');
                flag = false;
                [t_acc, jointPos_acc, jointVel_actual, jointAcc_actual, tau_acc] = deal(0, 0, 0, 0, 0);
                return;
        end
        waypoints(:,i) = currentQ';
    end

    fprintf('IK Done.\n');

    %% Torque-Based Motion Control
    % Now, for each pair of consecutive waypoints, we will first calculate a trajectory between these two points, and then calculate the torque
    % profile necessary to move from one point to the next.
    fprintf('Generating the Trajectory and Torque Profiles... \n');
    nbytes = fprintf('0%%');

    % Inititalize the variables where we will store the torque profiles, joint positions, and time, so that we can display them later
    tau_acc = [];
    jointPos_acc = [];
    t_acc = [];

    nPts = size(waypoints,2);

    for jj = 1 : nPts - 1
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
    
        % Initialize the time vector
        dt = 1e-3;       % time step [s]
        t  = 0:dt:1; % total time [s]

        % Initialize the arrays where we will accumulate the output of the robot dynamics
        jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
        jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
        jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
        tau_prescribed      = zeros(n,size(t,2)); % Joint Torques (Prescribed)

        jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
        jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)
        jointAcc_actual = zeros(n,size(t,2)); % Joint Accelerations (Actual)

        % For each joint
        for ii = 1 : n
            % Calculate a trajectory using a quintic polynomial
            params_traj.t = [0 t(end)]; % start and end time of each movement step
            params_traj.time_step = dt;
            params_traj.q = [waypoints(ii,jj) waypoints(ii,jj+1)];
            params_traj.v = [0 0];
            params_traj.a = [0 0];

            traj = make_trajectory('quintic', params_traj);

            % Generate the joint profiles (position, velocity, and acceleration)
            jointPos_prescribed(ii,:) = traj.q;
            jointVel_prescribed(ii,:) = traj.v;
            jointAcc_prescribed(ii,:) = traj.a;
        end

        % Initialize the parameters for both inverse and forward dynamics
        params_rne.g = g; % gravity
        params_rne.S = S; % screw axes
        params_rne.M = Mlist; % link frames
        params_rne.G = Glist; % inertial properties
        params_fdyn.g = g; % gravity
        params_fdyn.S = S; % screw axes
        params_fdyn.M = Mlist; % link frames
        params_fdyn.G = Glist; % inertial properties

        % Initialize the (actual) joint variables
        jointPos_actual(:,1) = jointPos_prescribed(:,1);
        jointVel_actual(:,1) = jointVel_actual(:,1);

        for ii = 1 : size(t,2) - 1
            % Calculate the joint torques using the RNE algorithm
            params_rne.jointPos = jointPos_prescribed(:,ii);
            params_rne.jointVel = jointVel_prescribed(:,ii);
            params_rne.jointAcc = jointAcc_prescribed(:,ii);
            
            % Total Force
            Total_Force = Force;
            T = fkine(S,M,params_rne.jointPos,'space');
            Ftip_space = [cross(T(1:3,4), Total_Force), Total_Force];
            params_rne.Ftip = adjoint(T)*Ftip_space';
            
            tau_prescribed(:,ii) = rne(params_rne);

            % Feed the torques to the forward dynamics model and perform one simulation step
            params_fdyn.jointPos = jointPos_actual(:,ii);
            params_fdyn.jointVel = jointVel_actual(:,ii);
            params_fdyn.tau = tau_prescribed(:,ii);
            params_fdyn.Ftip = params_rne.Ftip; % end effector wrench

            jointAcc = fdyn(params_fdyn);
            jointAcc_actual(:, ii+1) = jointAcc;
            
            % Integrate the joint accelerations to get velocity and position
            jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
            jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
        end

        tau_prescribed(:,end) = tau_prescribed(:,end-1);
        
        tau_acc = [tau_acc tau_prescribed];
        jointPos_acc = [jointPos_acc jointPos_actual];
        
        t_acc = [t_acc t+t(end)*(jj-1)];
    end
    
    flag = true;
end
