function desired_ax = longitudinal_speed_pi_controller_mfile(target_velocity, ego_velocity)
     
    persistent error_sum
    dt = 0.05;
    
    error = target_velocity - ego_velocity;

    if isempty(error_sum)
        error_sum = 0;
    else
        if error < 10
            error_sum = error_sum + error * dt;
        end
    end

    Kp = 1.5;
    Ki = 0.001;

    P_term = Kp*error;
    I_term = Ki*error_sum;
    
    desired_ax = (P_term + I_term);

end