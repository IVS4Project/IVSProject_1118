function tire_angle = lateral_pid_controller_mfile(cross_track_error)
    persistent error_prev error_sum
    % error = average_filter_mfile(cross_track_error);
    error = cross_track_error;
    
    dt = 0.05;

    if isempty(error_prev)
        error_prev = error;
    end

    if isempty(error_sum)
        error_sum = 0;
    else
        error_sum = error_sum + error * dt;
    end

    Kp = 0.415;
    Ki = 0.001;
    Kd = 0.25;
    
    P_term = Kp*error;
    I_term = Ki*error_sum;
    D_term = Kd*(error - error_prev) / dt;
    
    tire_angle = -(P_term + I_term + D_term);
    error_prev = error;
end