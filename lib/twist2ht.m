function T = twist2ht(S,theta)
    % Unpack The Twist into Rotation Velocity and Translational Velocity 
    omega = S(1:3);
    V = S(4:6);
    
    % omega_skew = [0 -omega(3) omega(2);
    %               omega(3) 0 -omega(1);
    %               -omega(2) omega(1) 0];

    omega_skew = skew(omega);
   
    R = axisangle2rot(omega,theta);
    p = ( eye(3)*theta + (1-cos(theta))*omega_skew + (theta - sin(theta))*omega_skew^2 )*V;
    T = [R  p; 0 0 0 1];
end