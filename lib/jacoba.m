function J_a = jacoba(S,M,q)        
    % Space Jacobian
    J = jacob0(S,q);
    T = fkine(S,M,q,'space');
    
    p = T(1:3,4);
    [~,col] = size(J);
    
    for i = 1:col
        J_w = J(1:3,i);
        J_v = J(4:6,i);
        J_a(:,i) = J_v - skew(p) * J_w;
    end
    
end
