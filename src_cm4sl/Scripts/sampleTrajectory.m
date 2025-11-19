function [s, d, s_dot, d_dot, s_ddot, d_ddot, s_jerk, d_jerk] = sampleTrajectory(coeff_s, coeff_d, T, N_pts)
    t_vec = linspace(0, T, N_pts)';

    % s = quartic
    [s, s_dot, s_ddot, s_jerk] = evalQuartic(coeff_s, t_vec);

    % d = quintic
    [d, d_dot, d_ddot, d_jerk] = evalQuintic(coeff_d, t_vec);
end

function [x, dx, ddx, jx] = evalQuintic(coeff, t)
    a0 = coeff(1); a1 = coeff(2); a2 = coeff(3);
    a3 = coeff(4); a4 = coeff(5); a5 = coeff(6);

    x   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    dx  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    ddx = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
    jx  = 6*a3 + 24*a4*t + 60*a5*t.^2;
end

function [x, dx, ddx, jx] = evalQuartic(coeff, t)
    a0 = coeff(1); a1 = coeff(2); a2 = coeff(3);
    a3 = coeff(4); a4 = coeff(5);

    x   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4;
    dx  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3;
    ddx = 2*a2 + 6*a3*t + 12*a4*t.^2;
    jx  = 6*a3 + 24*a4*t;
end