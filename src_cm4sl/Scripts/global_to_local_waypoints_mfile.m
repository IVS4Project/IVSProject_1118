function local_waypoints_ = global_to_local_waypoints_mfile(waypoints, vehicle_position, head)

    num_waypoint = 20;
    vehicle_position = vehicle_position(:)';

    dx = waypoints(:,1) - vehicle_position(1);
    dy = waypoints(:,2) - vehicle_position(2);
    dist = dx.^2 + dy.^2;

    isNotForward = (dx*cos(head) + dy*sin(head)) < 0;
    dist(isNotForward) = inf;

    [~, idx_start] = min(dist);

    idx_end = idx_start + num_waypoint - 1;
    idx_end = min(idx_end, size(waypoints,1));
    selected = waypoints(idx_start:idx_end, :);

    L = size(selected,1);
    if L < num_waypoint
        selected = [selected; repmat(selected(end,:), num_waypoint-L, 1)];
    end

    R_inv = [ cos(head),  sin(head);
             -sin(head),  cos(head) ];

    delta = bsxfun(@minus, selected, vehicle_position);
    local_waypoints_ = (R_inv * delta')';

end
