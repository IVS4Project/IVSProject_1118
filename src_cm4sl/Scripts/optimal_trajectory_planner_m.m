function [global_waypoints, driving_mission_info_out]  = optimal_trajectory_planner_m(MapData_Info, ego_status, otherVehicles, driving_mission_info_in)
    % centerLine: [X, Y]
    % egoState: [x, y, yaw, v, a]
    % otherVehicles: [X, Y, Vx, Vy, Ax, Ay, Yaw, Yawrate]
    % N_pts: 출력할 trajectory 점 개수
    % 출력: x_traj, y_traj - N_pts x 1 trajectory
    
    driving_mission_info_out = driving_mission_info_in;
    mission_state = driving_mission_info_in.missionState;
    tollgate_num = driving_mission_info_in.tollgateLaneNumber;
    targetSpeed = driving_mission_info_in.velReference;
    
    x = ego_status.ego_X;
    y = ego_status.ego_Y;
    yaw = ego_status.ego_YAW;
    centerLine = MapData_Info.Route2;

    laneWidth = 3.5;
    dList = [-laneWidth, 0.0, laneWidth];
    vCandidates = [targetSpeed, targetSpeed*0.5, targetSpeed*0.3];
    TimeList = 1.5:0.2:3.5;
    N_pts = 30;

    if mission_state == 3
        centerLine = MapData_Info.Route_Parking;
        dList = 0.0;
        TimeList = 1.0:0.5:2.0;
    end

    if mission_state == 4
        global_waypoints = [zeros(N_pts, 1), zeros(N_pts, 1)];
        return;
    end

    [si, di, yaw_ref] = globalToFrenet(centerLine, x, y);
    v = ego_status.ego_Speed;
    a = ego_status.ego_Accel;
    si_dot = v * cos(yaw - yaw_ref);
    si_ddot = a * cos(yaw - yaw_ref);
    di_dot = v * sin(yaw - yaw_ref);
    di_ddot = a * sin(yaw - yaw_ref);

    sf_dot = targetSpeed;
    
    trajSet = {};
    costs = [];
    VList = [];

    for i = 1:length(dList)
        df = dList(i);
        for j = 1:length(TimeList)
            T = TimeList(j);
            for k = 1:length(vCandidates)
                sf_dot_candidate = vCandidates(k);
    
                coeff_d = quinticPoly(di, di_dot, di_ddot, df, 0, 0, T);
                coeff_s = quarticPoly(si, si_dot, si_ddot, sf_dot_candidate, 0, T);
    
                [s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, s_jerk, d_jerk] = sampleTrajectory(coeff_s, coeff_d, T, N_pts);
    
                [x_traj_tmp, y_traj_tmp] = frenetToGlobal(s_traj, d_traj, centerLine);
                kappa = computeCurvature(x_traj_tmp, y_traj_tmp);
    
                if ~checkConstraints(s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, kappa, otherVehicles, centerLine, mission_state, tollgate_num, T, N_pts)
                    continue;
                end
    
                cost = computeCost(s_jerk, d_jerk, T, di, s_dot(end), df, targetSpeed);
    
                trajSet{end+1} = [x_traj_tmp, y_traj_tmp];
                costs(end+1) = cost;
                VList(end+1) = s_dot(end);
            end
        end
    end

    if isempty(costs)
        x_traj = zeros(N_pts, 1);
        y_traj = zeros(N_pts, 1);
        V_ref = single(5/3.6);
        driving_mission_info_out.is_Optimal_Path = uint8(0);
        % disp("no optimal path")
    else
        [~, index] = min(costs);
        x_traj = trajSet{index}(:,1);
        y_traj = trajSet{index}(:,2);
        V_ref = single(VList(index));
        driving_mission_info_out.is_Optimal_Path = uint8(1);
        % disp("optimal path find")
    end

    global_waypoints = [x_traj, y_traj];
    driving_mission_info_out.velReference = V_ref;

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
    min_dist = single(inf);
    closest_idx = 1;
    
    proj_point = single([centerLine(1,1); centerLine(1,2)]);
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
            proj_point = single(proj);
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

function valid = checkConstraints(s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, kappa, otherVehicles, centerLine, mission_state, tollgate_num, T, N_pts) 
    % 차선 조건
    laneWidth = 3.5;
    dList = [laneWidth, 0.0, -laneWidth];
    epsilon = 0.5;
    
    % 제약조건
    V_MAX     = 80.0/3.6;
    A_MAX     = 20.0;
    KAPPA_MAX = 2.0;
    COLL_DIST = 3.3;
    
    if mission_state == 2 && (abs(d_traj(end) - dList(tollgate_num)) > epsilon)
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

    % 2) ego yaw (heading) 추정 : 궤적 기울기 기반
    dx = gradient(x_traj);
    dy = gradient(y_traj);
    ego_yaw_traj = atan2(dy, dx);   % N_pts x 1
    
    % 차량 직사각형 크기 (공통)
    car_length = 4.5;    % [m]
    car_width  = 2.0;    % [m]

    for i = 1:size(otherVehicles,1)
        obs_X_init = otherVehicles(i,1);
        obs_Y_init = otherVehicles(i,2);

        obs_Vx_init = otherVehicles(i,3);
        obs_Vy_init = otherVehicles(i,4);

        obs_Ax_init = otherVehicles(i,5);
        obs_Ay_init = otherVehicles(i,6);

        obs_yaw_init = otherVehicles(i,7);
        obs_yawrate_init = otherVehicles(i,8);

        obs_V_init = hypot(obs_Vx_init, obs_Vy_init);
        obs_A_init = hypot(obs_Ax_init, obs_Ay_init);

        obs_inist_state0 = [obs_X_init; obs_Y_init; obs_V_init; obs_A_init; obs_yaw_init; obs_yawrate_init];

        T_pred  = T; % 예측할 길이
        dt_pred = T / N_pts; % 예측 dt
        N_pred  = round(T_pred/dt_pred); % Number of dt


        % prediction rollout
        % x_next = ctra_predict(x, y, v, a, psi, w, dt)

        pred = zeros(6, N_pred + 1);
        pred(:,1) = obs_inist_state0;

        for j = 1:N_pred
            pred(:,j+1) = ctra_predict(pred(:,j), dt_pred);
        end

        Xpred = pred(1,:);
        Ypred = pred(2,:);
        obs_yaw   = pred(5,:);
        Nt = min(N_pts, size(pred,2));

        for j = 1:Nt
            Ce   = [x_traj(j);   y_traj(j)];   % ego center
            Co   = [Xpred(j);    Ypred(j)];    % obs center
            psi_e = ego_yaw_traj(j);
            psi_o = obs_yaw(j);

            % SAT 기반 OBB vs OBB 충돌 체크
            isCollide = obbCollisionSAT(Ce, psi_e, Co, psi_o, car_length, car_width);

            if isCollide
                valid = false;
                % 디버깅용 출력 (원하면 주석 처리 가능)
                disp('Collision detected (SAT) at T = ');
                disp(T);
                return;
            end
        end

        dist2 = (x_traj - Xpred).^2 + (y_traj - Ypred).^2;

        if any(dist2 < COLL_DIST^2)
            valid = false;
            return;
        end
    end

    valid = true;
end

function cost = computeCost(s_jerk, d_jerk, T, di, sf_dot, df, TARGET_SPEED)
    laneWidth = 3.5;

    % 가중치
    K_J = 0.1;
    K_T = 1.0;
    K_D = 0.1;
    K_V = 3.0;
    K_LAT = 1.0;
    K_LON = 1.5;
    
    K_avoid_center = 10000.0;
    K_firstLane = 0.0;
    K_thridLane = 0.0;
    
    
    J_lat = sum(d_jerk.^2);
    J_lon = sum(s_jerk.^2);
    
    d_diff = (df - di)^2;
    v_diff = (TARGET_SPEED - sf_dot)^2;
    
    first_lane_idff = (df - laneWidth)^2;
    center_avoid_cost = exp(-(abs(df) / laneWidth)^2);
    third_lane_diff = (df + laneWidth)^2;

    
    c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
    c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
    
    cost = K_LAT * c_lat + K_LON * c_lon + ...
        + K_thridLane * third_lane_diff + K_firstLane * first_lane_idff + K_avoid_center * center_avoid_cost;
end

function kappa = computeCurvature(x, y)
    dx = gradient(x);
    dy = gradient(y);
    ddx = gradient(dx);
    ddy = gradient(dy);
    
    kappa = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(3/2));
end


% ----------- CTRA function -----------
function x_next = ctra_predict(x_curr, dt)
    x = x_curr(1);
    y = x_curr(2);
    v = x_curr(3);
    a = x_curr(4);
    psi = x_curr(5);
    w = x_curr(6);

    eps = 1e-5;

    if abs(w) < eps
        v_next = v + a*dt;
        s      = v*dt + 0.5*a*dt^2;
        x_p    = x + s*cos(psi);
        y_p    = y + s*sin(psi);
        psi_n  = psi;
        w_n    = w;
        % disp('CTRV is done at yawrate of:')
        % disp(w)
    else
        psi_n = psi + w*dt;

        x_p = x ...
          + (v/w)*(sin(psi_n) - sin(psi)) ...
          + (a/w^2)*(cos(psi_n) - cos(psi) + w*dt*sin(psi_n));

        y_p = y ...
          - (v/w)*(cos(psi_n) - cos(psi)) ...
          + (a/w^2)*(sin(psi_n) - sin(psi) - w*dt*cos(psi_n));

        v_next = v + a*dt;
        w_n    = w;
    end

    a_n = a;
    x_next = [x_p; y_p; v_next; a_n; psi_n; w_n];
end

function isCollide = obbCollisionSAT(Ce, psi_e, Co, psi_o, car_length, car_width)
    % Ce   : ego center [2x1]
    % psi_e: ego yaw [rad]
    % Co   : obs center [2x1]
    % psi_o: obs yaw [rad]
    % car_length, car_width : 공통 크기 (둘 다 동일하다고 가정)

    % half-extent
    hx_e = car_length/2;
    hy_e = car_width /2;
    hx_o = car_length/2;
    hy_o = car_width /2;

    % ego local axes (a0: forward, a1: left)
    a0 = [cos(psi_e); sin(psi_e)];
    a1 = [-sin(psi_e); cos(psi_e)];

    % obs local axes (b0: forward, b1: left)
    b0 = [cos(psi_o); sin(psi_o)];
    b1 = [-sin(psi_o); cos(psi_o)];

    % center difference
    d = Co - Ce;

    % test axes: A의 두 축, B의 두 축
    axes_to_test = [a0, a1, b0, b1]; % 2 x 4

    % SAT: 어느 한 축에서라도 분리되면 충돌 X
    for k = 1:4
        L = axes_to_test(:,k);   % 현재 검사 축
        
        % 센터 간 투영 거리
        dist = abs(dot(d, L));
        
        % ego 직사각형 투영 half-range
        r_e = hx_e * abs(dot(a0, L)) + hy_e * abs(dot(a1, L));
        % obs 직사각형 투영 half-range
        r_o = hx_o * abs(dot(b0, L)) + hy_o * abs(dot(b1, L));
        
        if dist > (r_e + r_o)
            % 이 축에서는 분리됨 → 두 직사각형은 겹치지 않음
            isCollide = false;
            return;
        end
    end

    % 어느 축에서도 분리되지 않았으면 → overlap → 충돌
    isCollide = true;
end