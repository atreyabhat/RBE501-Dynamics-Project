function T = xyzrpy_to_transformation(x, y, z, roll, pitch, yaw)
    % Create the rotation matrix using the RPY angles
    R = eul2rotm([yaw, pitch, roll], 'ZYX');

    % Create the homogeneous transformation matrix
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = [x; y; z];
end