function adj = get_adj_obstacles_info(vehicle_position, obstacles_info, dist_threshhold)

    % vehicle_position : [x, y] (EGO global position)
    % obstacles_info   : Nx? matrix or struct (each obstacle row = one vehicle)
    %                    columns assumed: [X, Y, Vx, Vy, Ax, Ay, Yaw]
    % dist_thresh      : scalar (e.g., 50 meters)

    ego_x = vehicle_position(1);
    ego_y = vehicle_position(2);

    X = obstacles_info(:,1);
    Y = obstacles_info(:,2);

    % 거리 계산
    dx = X - ego_x;
    dy = Y - ego_y;
    dist = sqrt(dx.^2 + dy.^2);

    % threshold 안에 있는 인덱스
    idx = dist < dist_threshhold;

    % 필터링된 장애물 정보
    adj = obstacles_info(idx, :);

    disp(adj);
end
