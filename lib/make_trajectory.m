function traj = make_trajectory(type, params)

    traj.t = params.t(1):params.time_step:params.t(2);
    n = length(traj.t);

    if strcmp(type, 'cubic')
        % Cubic polynomial coefficients
        x = zeros(4,1);
        q_m = [params.q(1) params.v(1) params.q(2) params.v(2)]';
        t_m = [1 params.t(1) params.t(1)^2 params.t(1)^3;
               0 1 2*params.t(1) 3*params.t(1)^2;
               1 params.t(2) params.t(2)^2 params.t(2)^3;
               0 1 2*params.t(2) 3*params.t(2)^2];
        x = t_m \ q_m;
        x0 = x(1);
        x1 = x(2);
        x2 = x(3);
        x3 = x(4);

        % Position, velocity, and acceleration
        traj.q = x0 + x1 * traj.t + x2 * traj.t.^2 + x3 * traj.t.^3;
        traj.v = x1 + 2 * x2 * traj.t + 3 * x3 * traj.t.^2;
        traj.a = 2 * x2 + 6 * x3 * traj.t;

    elseif strcmp(type, 'quintic')
        % Quintic polynomial coefficients
        x_m = zeros(6,1);
        q_m = [params.q(1) params.v(1) params.a(1) params.q(2) params.v(2) params.a(2)]';
        t_m = [1 params.t(1) params.t(1)^2 params.t(1)^3 params.t(1)^4 params.t(1)^5;
               0 1 2*params.t(1) 3*params.t(1)^2 4*params.t(1)^3 5*params.t(1)^4;
               0 0 2 6*params.t(1) 12*params.t(1)^2 20*params.t(1)^3;
               1 params.t(2) params.t(2)^2 params.t(2)^3 params.t(2)^4 params.t(2)^5;
               0 1 2*params.t(2) 3*params.t(2)^2 4*params.t(2)^3 5*params.t(2)^4;
               0 0 2 6*params.t(2) 12*params.t(2)^2 20*params.t(2)^3];
        x_m = t_m \ q_m;
        x0 = x_m(1);
        x1 = x_m(2);
        x2 = x_m(3);
        x3 = x_m(4);
        x4 = x_m(5);
        x5 = x_m(6);

        % Position, velocity, and acceleration
        traj.q = x0 + x1 * traj.t + x2 * traj.t.^2 + x3 * traj.t.^3 + x4 * traj.t.^4 + x5 * traj.t.^5;
        traj.v = x1 + 2 * x2 * traj.t + 3 * x3 * traj.t.^2 + 4 * x4 * traj.t.^3 + 5 * x5 * traj.t.^4;
        traj.a = 2 * x2 + 6 * x3 * traj.t + 12 * x4 * traj.t.^2 + 20 * x5 * traj.t.^3;

    else
        error('Invalid type');
    end
end

