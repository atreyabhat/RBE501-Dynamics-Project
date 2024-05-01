function J_b = jacobe(S,M,q)    
    
    % Initialize
    J_s = zeros(6, length(q));
    
    % Iterate over each joint
    for i = 1:length(q)
        
        % Calculate the homogeneous transformation up to joint i
        T = eye(4);
        for j = 1:i
            T = T * twist2ht(S(:, j), q(j));
        end
        
        % Slice the twist for joint i
        V = S(:, i);
        V_trans = adjoint(V, T);
        
        J_s(:, i) = V_trans;
        
    end
    
    T = T*M;
    
    AdjT = adjoint_only(T);
    
    J_b = AdjT\J_s;
    
    
end