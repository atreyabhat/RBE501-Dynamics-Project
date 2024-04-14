function adV = ad(V)
    v = V(4:6);
    omega = V(1:3);
    
    % skew-symmetric matrices
    omega_hat = skew(omega);
    v_hat = skew(v);
    
    adV = [omega_hat, zeros(3,3); v_hat , omega_hat];
end