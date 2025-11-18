function poly_coeff = fit_polynomial_to_waypoints_mfile(local_waypoints)
    
    num_degree = 3;
    N = size(local_waypoints, 1);

    x = local_waypoints(:,1);
    b = local_waypoints(:,2);

    A = zeros(N, num_degree+1);
    for i = 0:num_degree
        A(:, i+1) = x.^i;
    end
    
    poly_coeff = pinv(A) * b;
    
end