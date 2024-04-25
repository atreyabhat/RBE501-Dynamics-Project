function adV = ad(V)
    w = V(1:3);
    v = V(4:6);
    
    skew_w = [0 -w(3) w(2);
              w(3) 0 -w(1);
              -w(2) w(1) 0];
    
    skew_v = [0 -v(3) v(2);
              v(3) 0 -v(1);
              -v(2) v(1) 0];
    
    adV = [skew_w zeros(3);
           skew_v skew_w];
   
end