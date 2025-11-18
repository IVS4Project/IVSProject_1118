function desired_ax = longitudinal_space_pid_controller_mifle(target_distance, clearance)
    persistent error_prev error_sum
    dt = 0.05;
    
    error = clearance - target_distance;
    
    if isempty(error_prev)
        error_prev = error;
    end

    if isempty(error_sum)
        error_sum = 0;
    else
        if error < 10
            error_sum = error_sum + error * dt;
        end
    end

    Kp = 1.5;
    Ki = 0.01;
    Kd = 2.0;

    P_term = Kp*error;
    I_term = Ki*error_sum;
    D_term = Kd*(error - error_prev) / dt;
    
    desired_ax = (P_term + I_term + D_term);
    error_prev = error;
end