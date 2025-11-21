egoState = [-49.57, -73.00, deg2rad(0), 15, 0];
otherVehicles = [
    -49.57 + 50, -73, 0,0,0,0,0,0;   % 정면 멀리
    -49.57 + 20, -73, 0,0,0,0,0,0;  % 정면 가까이
    -49.57, -73 - 12, 0,0,0,0,0,0;  % 오른쪽 차선 차량
];
N_pts = 100;

%% 5) planOptimalTrajectory 실행
disp("=== Running planOptimalTrajectory ===")
global_waypoints = optimal_trajectory_planner(route2_waypoint, egoState, otherVehicles, N_pts);


%% 7) s-map boundary test
disp("=== s-map boundary test ===")

%s_map 계산
s_map = zeros(size(route2_waypoint,1), 1);
for i = 2:length(route2_waypoint)
    s_map(i) = s_map(i-1) + norm(route2_waypoint(i,:) - route2_waypoint(i-1,:));
end

test_s = [ -1, 0, 0.5, s_map(end)-0.1, s_map(end), s_map(end)+1 ];

for k = 1:length(test_s)
    s_val = test_s(k);
    try
        [x_t, y_t] = frenetToGlobal(s_val, 0.0, route2_waypoint);
        fprintf("s = %.2f OK (x=%.2f, y=%.2f)\n", s_val, x_t, y_t);
    catch ME
        fprintf("s = %.2f FAILED (%s)\n", s_val, ME.message);
    end
end

%% 8) 9개 후보 trajectory 직접 플롯
disp("=== Plot all candidates ===")

figure; hold on; axis equal; grid on;
plot(route1_waypoint(:,1), route1_waypoint(:,2), 'k-', 'LineWidth', 1.5);
plot(route3_waypoint(:,1), route3_waypoint(:,2), 'k-', 'LineWidth', 1.5);
plot(route2_waypoint(:,1), route2_waypoint(:,2), 'k-', "LineWidth", 1.5);
plot(otherVehicles(:,1), otherVehicles(:,2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);

laneWidth = 6.0;
dList = [-laneWidth, 0.0, laneWidth];
TimeList = 1.0:1.0:4.0;

[si, di, yaw_ref] = globalToFrenet(route2_waypoint, egoState(1), egoState(2));
v = egoState(4);
a = egoState(5);
si_dot = v * cos(egoState(3) - yaw_ref);
si_ddot = a * cos(egoState(3) - yaw_ref);
di_dot = v * sin(egoState(3) - yaw_ref);
di_ddot = a * sin(egoState(3) - yaw_ref);

sf_dot = 20.0;

trajCount = 1;

for i = 1:3
    df = dList(i);
    for j = 1:4
        T = TimeList(j);

        coeff_d = quinticPoly(di, di_dot, di_ddot, df, 0, 0, T);
        coeff_s = quarticPoly(si, si_dot, si_ddot, sf_dot, 0, T);

        [s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, s_jerk, d_jerk] = ...
            sampleTrajectory(coeff_s, coeff_d, T, N_pts);

        [x_tmp, y_tmp] = frenetToGlobal(s_traj, d_traj, route2_waypoint);

        plot(x_tmp, y_tmp, '-', "LineWidth", 1);
        text(x_tmp(end), y_tmp(end), sprintf("T%d", trajCount));
        trajCount = trajCount + 1;
    end
end

plot(egoState(1), egoState(2), 'ro', "MarkerSize", 10, "LineWidth", 2);
title("All 9 candidate trajectories");
xlabel("X"); ylabel("Y");

%% 9) 최종 선택된 최적 경로 플롯
disp("=== Plot Selected Optimal Trajectory ===")

figure; hold on; axis equal; grid on;
plot(route1_waypoint(:,1), route1_waypoint(:,2), 'k--', "LineWidth", 1.5);
plot(route2_waypoint(:,1), route2_waypoint(:,2), 'k--', "LineWidth", 1.5);
plot(route3_waypoint(:,1), route3_waypoint(:,2), 'k--', "LineWidth", 1.5);
plot(global_waypoints(:,1), global_waypoints(:,2), 'b-', "LineWidth", 2);
plot(otherVehicles(:,1), otherVehicles(:,2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
plot(egoState(1), egoState(2), 'ro', "MarkerSize", 10);
title("Selected Optimal Trajectory");
xlabel("X"); ylabel("Y");



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

function global_waypoints  = optimal_trajectory_planner(centerLine, egoState, otherVehicles, N_pts)
    % centerLine: [X, Y]
    % egoState: [x, y, yaw, v, a]
    % otherVehicles: [X, Y, Vx, Vy, Ax, Ay, Yaw, Yawrate]
    % N_pts: 출력할 trajectory 점 개수
    % 출력: x_traj, y_traj - N_pts x 1 trajectory
    laneWidth = 6.0;
    dList = [-laneWidth, 0.0, laneWidth];
    targetSpeed = 20.0;
    TimeList = [1.0, 1.5, 2.0];

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
        for j = 1:3
            T = TimeList(j);
            coeff_d = quinticPoly(di, di_dot, di_ddot, df, 0, 0, T);
            coeff_s = quarticPoly(si, si_dot, si_ddot, sf_dot, 0, T);

            [s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, s_jerk, d_jerk] = sampleTrajectory(coeff_s, coeff_d, T, N_pts);

            [x_traj_tmp, y_traj_tmp] = frenetToGlobal(s_traj, d_traj, centerLine);
            kappa = computeCurvature(x_traj_tmp, y_traj_tmp);

            if ~checkConstraints(s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, kappa, otherVehicles, centerLine)
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

function valid = checkConstraints(s_traj, d_traj, s_dot, d_dot, s_ddot, d_ddot, kappa, otherVehicles, centerLine)
    V_MAX     = 30.0;
    A_MAX     = 20.0;
    KAPPA_MAX = 2.0;
    COLL_DIST = 3.0;
    % 속도 제약
    v = s_dot.^2 + d_dot.^2;
    if any(v > V_MAX^2)
        valid = false;
        disp("to much V")
        return;
    end
    
    % 가속도 제약
    acc = s_ddot.^2 + d_ddot.^2;
    if any(acc > A_MAX^2)
        valid = false;
        disp("to much Acc")
        return;
    end

    % 곡률 제약
    if any(abs(kappa) > KAPPA_MAX)
        valid = false;
        disp("to much kappa")
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
            disp("collision!")
            return;
        end
    end

    valid = true;
    disp(valid);
end

function cost = computeCost(s_jerk, d_jerk, T, di, sf_dot, df, TARGET_SPEED)
    % 가중치
    K_J = 0.1;
    K_T = 0.1;
    K_D = 2.0;
    K_V = 0.1;
    K_LAT = 2.0;
    K_LON = 1.0;
    
    J_lat = sum(d_jerk.^2);
    J_lon = sum(s_jerk.^2);
    
    d_diff = (df - di)^2;
    v_diff = (TARGET_SPEED - sf_dot)^2;
    
    c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
    c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
    
    cost = K_LAT * c_lat + K_LON * c_lon;
end

function kappa = computeCurvature(x, y)
    dx = gradient(x);
    dy = gradient(y);
    ddx = gradient(dx);
    ddy = gradient(dy);
    
    kappa = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(3/2));
end
