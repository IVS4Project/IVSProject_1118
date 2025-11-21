function global_waypoints  = optimal_trajectory_planner(centerLine, egoState, otherVehicles, N_pts, mission_state)
    % centerLine: [X, Y]
    % egoState: [x, y, yaw, v, a]
    % otherVehicles: [X, Y, Vx, Vy, Ax, Ay, Yaw, Yawrate]
    % N_pts: 출력할 trajectory 점 개수
    % 출력: x_traj, y_traj - N_pts x 1 trajectory
    
    laneWidth = 6.0;
    dList = [-laneWidth, 0.0, laneWidth];
    targetSpeed = 20.0;
    TimeList = 1.0:1.0:4.0;

    [si, di, yaw_ref] = globalToFrenet(centerLine, egoState(1), egoState(2));
    v = egoState(4);
    a = egoState(5);
    si_dot = v * cos(egoState(3) - yaw_ref);
    si_ddot = a * cos(egoState(3) - yaw_ref);
    di_dot = v * sin(egoState(3) - yaw_ref);
    di_ddot = a * sin(egoState(3) - yaw_ref);

    sf_dot = targetSpeed;
    
    trajSet = {};
    costs = [];

    for i = 1:3
        df = dList(i);
        for j = 1:4
            T = TimeList(j);

            coeff_d = quinticPoly(di, di_dot, di_ddot, df, 0, 0, T);
            coeff_s = quarticPoly(si, si_dot, si_ddot, sf_dot, 0, T);

            [s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, s_jerk, d_jerk] = sampleTrajectory(coeff_s, coeff_d, T, N_pts);

            [x_traj_tmp, y_traj_tmp] = frenetToGlobal(s_traj, d_traj, centerLine);
            kappa = computeCurvature(x_traj_tmp, y_traj_tmp);

            if ~checkConstraints(s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, kappa, otherVehicles, centerLine, mission_state)
                continue;
            end

            cost = computeCost(s_jerk, d_jerk, T, di, s_dot(end), df, targetSpeed);

            trajSet{end+1} = [x_traj_tmp, y_traj_tmp];
            costs(end+1) = cost;

        end
    end

    if isempty(costs)
        x_traj = zeros(N_pts, 1);
        y_traj = zeros(N_pts, 1);
    else
        [~, index] = min(costs);
        x_traj = trajSet{index}(:,1);
        y_traj = trajSet{index}(:,2);
    end

    global_waypoints = [x_traj, y_traj];

end

function coeff = quinticPoly(xi, vi, ai, xf, vf, af, T)
    A = [T^3,    T^4,     T^5;
         3*T^2,  4*T^3,   5*T^4;
         6*T,    12*T^2,  20*T^3];

    b = [xf - (xi + vi*T + 0.5*ai*T^2);
         vf - (vi + ai*T);
         af - ai];

    x = A \ b;

    coeff = [xi, vi, 0.5*ai, x(1), x(2), x(3)];
end

function coeff = quarticPoly(xi, vi, ai, vf, af, T)
    A = [3*T^2,    4*T^3;
         6*T,     12*T^2];

    b = [vf - (vi + ai*T);
         af - ai];

    x = A \ b;

    coeff = [xi, vi, 0.5*ai, x(1), x(2)];
end

function [s, d, heading] = globalToFrenet(centerLine, x, y)
    min_dist = inf;
    closest_idx = 1;
    
    proj_point = [centerLine(1,1); centerLine(1,2)];
    segment_vec = [1; 0];
    
    % 가장 가까운 segment 찾기
    for i = 1:size(centerLine,1)-1
        p1 = centerLine(i,:)';
        p2 = centerLine(i+1,:)';
        proj = projectionOnSegment([x; y], p1, p2);
        dist = norm([x; y] - proj);
        if dist < min_dist
            min_dist = dist;
            closest_idx = i;
            proj_point = proj;
            segment_vec = p2 - p1;
        end
    end

    % cumulative s 계산
    s = 0;
    for i = 1:closest_idx-1
        s = s + norm(centerLine(i+1,:) - centerLine(i,:));
    end
    s = s + norm(proj_point' - centerLine(closest_idx,:));

    % d 계산
    heading = atan2(segment_vec(2), segment_vec(1));
    normal = [-sin(heading); cos(heading)]; % 왼쪽 수직 방향
    d = dot(([x; y] - proj_point), normal);
end


function proj = projectionOnSegment(p, p1, p2)
    % p: 2x1, p1, p2: 2x1
    v = p2 - p1;
    t = dot(p - p1, v) / dot(v, v);
    t = max(0, min(1, t));
    proj = p1 + t * v;
end

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

function [x_traj, y_traj] = frenetToGlobal(s_traj, d_traj, centerLine)
    % s_traj, d_traj : N x 1 (frenet trajectory)
    % centerLine : M x 2 (global waypoints)
    % x_traj, y_traj : N x 1 (global trajectory)
    
    
    N = length(s_traj);
    x_traj = zeros(N, 1);
    y_traj = zeros(N, 1);
    
    
    % 사전 cumulative s 계산
    s_map = zeros(size(centerLine,1),1);
    for i = 2:length(centerLine)
        ds = norm(centerLine(i,:) - centerLine(i-1,:));
        s_map(i) = s_map(i-1) + ds;
    end
    
    
    for i = 1:N
        s = s_traj(i);
        d = d_traj(i);
        
        
        % s 기준 위치 찾기
        idx = find(s_map >= s, 1);
        if isempty(idx)
            idx = length(s_map);
        end
        if idx == 1
            idx = 2;
        end
    
        s0 = s_map(idx-1);
        s1 = s_map(idx);
        p0 = centerLine(idx-1,:);
        p1 = centerLine(idx,:);
        
        
        heading = atan2(p1(2) - p0(2), p1(1) - p0(1));
        ratio = (s - s0) / (s1 - s0);
        px = p0(1) + ratio * (p1(1) - p0(1));
        py = p0(2) + ratio * (p1(2) - p0(2));
        
    
        normal = [-sin(heading), cos(heading)];
        x_traj(i) = px + d * normal(1);
        y_traj(i) = py + d * normal(2);
    end

end

function valid = checkConstraints(s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, kappa, otherVehicles, centerLine, mission_state)    
    % 차선 조건
    laneWidth = 6.0;
    dList = [laneWidth, 0.0, -laneWidth];
    epsilon = 0.5;

    tollgate_num = 1; % 이 값 변경
    
    % 제약조건
    V_MAX     = 30.0;
    A_MAX     = 20.0;
    KAPPA_MAX = 3.0;
    COLL_DIST = 4.0;
    
    if mission_state == 2 && (abs(d_traj(end) - dList(tollgate_num)) > epsilon)
        valid = false;
        return;
    end

    if mission_state == 3 && abs(d_traj(end) + laneWidth) > epsilon
        valid = false;
        return;
    end

    % 속도 제약
    v = s_dot.^2 + d_dot.^2;
    if any(v > V_MAX^2)
        valid = false;
        return;
    end
    
    % 가속도 제약
    acc = s_ddot.^2 + d_ddot.^2;
    if any(acc > A_MAX^2)
        valid = false;
        return;
    end

    % 곡률 제약
    if any(abs(kappa) > KAPPA_MAX)
        valid = false;
        return;
    end

    % 충돌 확인
    [x_traj, y_traj] = frenetToGlobal(s_traj, d_traj, centerLine);
    for i = 1:size(otherVehicles,1)
        veh_x = otherVehicles(i,1);
        veh_y = otherVehicles(i,2);

        dist2 = (x_traj - veh_x).^2 + (y_traj - veh_y).^2;
        if any(dist2 < COLL_DIST^2) % 충돌 반경 0.25m
            valid = false;
            return;
        end
    end

    valid = true;
end

function cost = computeCost(s_jerk, d_jerk, T, di, sf_dot, df, TARGET_SPEED)
    % 가중치
    K_J = 0.5;
    K_T = 0.5;
    K_D = 3.0;
    K_V = 0.1;
    K_LAT = 5.0;
    K_LON = 1.0;
    K_first = 1.0;
    laneWidth = 6.0;
    
    J_lat = sum(d_jerk.^2);
    J_lon = sum(s_jerk.^2);
    
    d_diff = (df - di)^2;
    v_diff = (TARGET_SPEED - sf_dot)^2;
    
    c_first_lane = (df - laneWidth)^2;
    
    c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
    c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
    
    cost = K_LAT * c_lat + K_LON * c_lon + K_first * c_first_lane;
end

function kappa = computeCurvature(x, y)
    dx = gradient(x);
    dy = gradient(y);
    ddx = gradient(dx);
    ddy = gradient(dy);
    
    kappa = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(3/2));
end