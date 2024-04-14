function R = axisangle2rot(omega,theta)
   
    K = skew(omega);
    
    %Rodrigues formula
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K^2);

end