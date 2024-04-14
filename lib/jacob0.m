function [J,T] = jacob0(S,q) 
    % Initialize
    J = zeros(6, length(q));
    
    % Iterate over each joint
    for i = 1:length(q)
        
        % Calculate the homogeneous transformation up to joint i
        T = eye(4);
        for j = 1:i
            T = T * twist2ht(S(:, j), q(j));
        end
        
        % Slice the twist for joint i
        V = S(:, i);
        V_trans = adjoint_twist(V, T);
        
        J(:, i) = V_trans;
        
    end
end
