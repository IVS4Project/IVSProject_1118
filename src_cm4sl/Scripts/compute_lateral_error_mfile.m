function [lateral_error_control_ref, x_lookahead, lateral_error] = compute_lateral_error_mfile(poly_coeff, velocity, lookahead_time)

    x_lookahead = velocity * lookahead_time;
    degree = length(poly_coeff) - 1;

    y_ref = 0;
    for i = 0:degree
        y_ref = y_ref + poly_coeff(i+1) * x_lookahead^i;
    end

    lateral_error_control_ref = y_ref;
    lateral_error = poly_coeff(1);
end
