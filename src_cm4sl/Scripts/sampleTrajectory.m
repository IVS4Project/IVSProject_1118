function [s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, s_jerk, d_jerk] = sampleTrajectory(coeff_s, coeff_d, T, N_pts)
    t_vec = linspace(0, T, N_pts)';

    % s(t), s_dot(t), s_ddot(t), s_jerk(t)
    s_traj = polyval(flip(coeff_s), t_vec);
    s_dot = polyval(polyder(flip(coeff_s)), t_vec);
    s_ddot = polyval(polyder(polyder(flip(coeff_s))), t_vec);
    s_jerk = polyval(polyder(polyder(polyder(flip(coeff_s)))), t_vec);

    % d(t), d_dot(t), d_ddot(t), d_jerk(t)
    d_traj = polyval(flip(coeff_d), t_vec);
    d_dot = polyval(polyder(flip(coeff_d)), t_vec);
    d_ddot = polyval(polyder(polyder(flip(coeff_d))), t_vec);
    d_jerk = polyval(polyder(polyder(polyder(flip(coeff_d)))), t_vec);
end