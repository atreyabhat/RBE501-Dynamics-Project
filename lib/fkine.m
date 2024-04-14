function T = fkine(S,M,q,frame)
    num_joints = size(S, 2);
    if strcmp(frame, 'space')
        T = eye(4);
        for i = 1:num_joints
            T_joint = twist2ht(S(:,i), q(i));
            T = T * T_joint;
        end
        T = T*M;
    end   
    if strcmp(frame, 'body')
        T = M;
        for i = 1:num_joints
            T_joint = twist2ht(S(:,i) , q(i));

            T = T * T_joint;
        end
    end
end

