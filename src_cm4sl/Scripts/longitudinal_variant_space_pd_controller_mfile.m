function desired_ax = longitudinal_variant_space_pd_controller_mfile(ego_velocity, clearance)
    
    persistent error_prev
    dt = 0.05;
    time_gap = 3.0;
    target_distance = ego_velocity*time_gap;

    error = clearance - target_distance;
    
    if isempty(error_prev)
        error_prev = error;
    end

    Kp = 3.0;
    Kd = 1.0;

    P_term = Kp*error;
    D_term = Kd*(error - error_prev) / dt;
    
    desired_ax = (P_term + D_term);
    error_prev = error;
end