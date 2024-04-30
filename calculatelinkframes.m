function [M01, M12, M23, M34, M45, M56, M67, M78] = calculatelinkframes(robot)
  
  theta = [0 0 0 0 0 0 0];   %predefining theta here, robot.theta isnt available

  Mj1 = tdh(theta(1), robot.d(1), robot.a(1), robot.alpha(1));
  Mj2 = Mj1 * tdh(theta(2), robot.d(2), robot.a(2), robot.alpha(2));
  Mj3 = Mj2 * tdh(theta(3), robot.d(3), robot.a(3), robot.alpha(3));
  Mj4 = Mj3 * tdh(theta(4), robot.d(4), robot.a(4), robot.alpha(4));
  Mj5 = Mj4 * tdh(theta(5), robot.d(5), robot.a(5), robot.alpha(5));
  Mj6 = Mj5 * tdh(theta(6), robot.d(6), robot.a(6), robot.alpha(6));
  Mj7 = Mj6 * tdh(theta(7), robot.d(7), robot.a(7), robot.alpha(7));
  
  % CoM is the origin of all the links
  M1 = Mj1 * [eye(3) [0, 0, 0.0075]'; 0 0 0 1];
  M2 = Mj2 * [eye(3) [0.0003, 0.059, 0.042]'; 0 0 0 1];
  M3 = Mj3 * [eye(3) [0 ,0.03, 0.13]'; 0 0 0 1];
  M4 = Mj4 * [eye(3) [0, 0.067, 0.034]'; 0 0 0 1];
  M5 = Mj5 * [eye(3) [0.0001, 0.021, 0.076]'; 0 0 0 1];
  M6 = Mj6 * [eye(3) [0, 0.0006, 0.0004]'; 0 0 0 1];
  M7 = Mj7 * [eye(3) [0, 0, 0.02]'; 0 0 0 1];
  
  M01 = M1;
  M12 = pinv(pinv(M2)*M1);
  M23 = pinv(pinv(M3)*M2);
  M34 = pinv(pinv(M4)*M3);
  M45 = pinv(pinv(M5)*M4);
  M56 = pinv(pinv(M6)*M5);
  M67 = pinv(pinv(M7)*M6);
  M78 = eye(4);
end

