function [S,M] = make_kinematics_model(robot)
     % MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
     % the UR5 robot.
     %
     % Inputs: robot - the robot object created by the robotics toolbox
     %
     % Output: S - 6xn matrix whose columns are the screw axes of the robot
     %         M - homogeneous transformation representing the home configuration

     % Screw Axes - Kukaiiwa7 
     S = [0 0 1 0 0 0;
          0 1 0 -0.36 0 0;
          0 0 1 0 0 0;
          0 -1 0 0.789 0 0;
          0 0 1 0 0 0;
          0 1 0 -1.189 0 0;
          0 0 1 0 0 0]';

     % Home configuration
     M = double(robot.fkine(zeros(1,7)));

end