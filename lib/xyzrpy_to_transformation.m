function T = xyzrpy_to_transformation(path)
    % Create the rotation matrix using the RPY angles
    R = eul2rotm([path(6), path(5), path(4)] * pi/180, 'ZYX');

    % Create the homogeneous transformation matrix
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = [path(1); path(2); path(3)];
end