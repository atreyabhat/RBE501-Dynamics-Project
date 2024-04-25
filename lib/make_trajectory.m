function traj = make_trajectory(type, params)
    
    time_step = params.time_step;
    
    traj.t = params.t(1):time_step:params.t(2);
    t0 = params.t(1);
    tf = params.t(2);
    
    if strcmp(type,'cubic')
        a = zeros(4,1);
        
        q_m = [params.q(1) params.v(1) params.q(2) params.v(2)]';
        t_m = [1 t0 t0^2 t0^3;
               0 1  2*t0 3*t0^2;
               1 tf tf^2 tf^3;
               0 1  2*tf 3*tf^2];
        
        a = t_m \ q_m;

        traj.q = a(1) + a(2) * traj.t + a(3) * traj.t.^2 + a(4) * traj.t.^3;
        traj.v = a(2) + 2 * a(3) * traj.t + 3 * a(4) * traj.t.^2;
        traj.a = 2 * a(3) + 6 * a(4) * traj.t;
        
    elseif strcmp(type,'quintic')
        a_m = zeros(6,1);
        
        q_m = [params.q(1) params.v(1) params.a(1) params.q(2) params.v(2) params.a(2)]';
        t_m = [1 t0 t0^2   t0^3    t0^4    t0^5;
               0 1  2*t0   3*t0^2  4*t0^3  5*t0^4;
               0 0  2      6*t0    12*t0^2 20*t0^3;
               1 tf tf^2   tf^3    tf^4    tf^5;
               0 1  2*tf   3*tf^2  4*tf^3  5*tf^4;
               0 0  2      6*tf    12*tf^2 20*tf^3;];
        
        a_m = t_m \ q_m;
        
        traj.q = a_m(1) + a_m(2) * traj.t + a_m(3) * traj.t.^2 + a_m(4) * traj.t.^3 + a_m(5) * traj.t.^4 + a_m(6) * traj.t.^5;
        traj.v = a_m(2) + 2 * a_m(3) * traj.t + 3 * a_m(4) * traj.t.^2 + 4 * a_m(5) * traj.t.^3 + 5 * a_m(6) * traj.t.^4;
        traj.a = 2 * a_m(3) + 6 * a_m(4) * traj.t + 12 * a_m(5) * traj.t.^2 + 20 * a_m(6) * traj.t.^3;
        
    else
        error('Invalid trajectory');
    end
end
