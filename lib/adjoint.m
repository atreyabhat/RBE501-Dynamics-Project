function AdT = adjoint_n(T)
    % Extract R and p
    R = T(1:3,1:3);
    p = T(1:3,4);
    
    skew_p = [0 -p(3) p(2);
              p(3) 0 -p(1);
              -p(2) p(1) 0];
    
    AdT = [R zeros(3); skew_p*R R];   
end
