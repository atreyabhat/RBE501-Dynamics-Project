function R = axisangle2rot(omega,theta)
    % omega_skew = [0 -omega(3) omega(2);
    %               omega(3) 0 -omega(1);
    %               -omega(2) omega(1) 0];
    
    % Skew func from Robotics Toolbox
    omega_skew = skew(omega);
    
    % Rodrigues Formula
    R = eye(3) + sin(theta)*omega_skew + (1 - cos(theta))*(omega_skew)^2;
end