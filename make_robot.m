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

    %% URL for Robot URDF
    % https://github.com/miladehghani/KUKA_IIWA_URDF/blob/master/iiwa14.urdf

    %% Create the manipulator
    L1 = 0.360; 
    L3 = 0.429; 
    L5 = 0.400; 
    L7 = 0.126;

    robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', -pi/2), ...
                        Revolute('a', 0, 'd', 0, 'alpha', pi/2), ...
                        Revolute('a', 0, 'd', L3, 'alpha', pi/2), ...
                        Revolute('a', 0, 'd', 0, 'alpha', -pi/2), ...
                        Revolute('a', 0, 'd', L5, 'alpha', -pi/2), ...
                        Revolute('a', 0, 'd', 0, 'alpha', pi/2), ...
                        Revolute('a', 0, 'd', L7, 'alpha', 0)], ...
                        'name', 'KUKA IIWA 14');


end