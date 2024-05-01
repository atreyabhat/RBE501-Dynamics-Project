function J_a = jacoba(S,M,q)    
    
    [J,T] = jacob0(S,q);
    J_a = zeros(3,length(q));
    
    T = T * M;
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    % Construct the skew-symmetric matrix of p
    p_skew = [0, -p(3), p(2);
           p(3), 0, -p(1);
           -p(2), p(1), 0];
    
    Jw = J(1:3,:);
    Jv = J(4:6,:);
    
    J_a = Jv- p_skew * Jw;

end