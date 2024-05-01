function twist_inB = adjoint_twist(twist_inA,T_AB)
    R = T_AB(1:3, 1:3);
    p = T_AB(1:3, 4);
    
    px = [0, -p(3), p(2);
         p(3), 0, -p(1);
         -p(2), p(1), 0];
    
    AdT = [R, zeros(3); px*R, R];
    
    twist_inB = AdT * twist_inA;
end

