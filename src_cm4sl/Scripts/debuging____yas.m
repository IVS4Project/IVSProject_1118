egoState = [-49.57, -73.00, deg2rad(0), 15, 0];
otherVehicles = zeros(0, 8);
N_pts = 50;

%% 5) planOptimalTrajectory 실행
disp("=== Running planOptimalTrajectory ===")
global_waypoints = optimal_trajectory_planner(route2_waypoint, egoState, otherVehicles, N_pts);

%% 6) global → frenet → global 역변환 정확도 테스트
disp("=== global <-> frenet conversion test ===")

[s_test, d_test, yaw_ref] = globalToFrenet(route2_waypoint, egoState(1), egoState(2));
[x_back, y_back] = frenetToGlobal(s_test, d_test, route2_waypoint);

fprintf("Original global: (%.3f, %.3f)\n", egoState(1), egoState(2));
fprintf("Back-converted:  (%.3f, %.3f)\n", x_back, y_back);
fprintf("Error = %.6f m\n", hypot(x_back - egoState(1), y_back - egoState(2)));

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

laneWidth = 6.0;
dList = [-laneWidth, 0.0, laneWidth];
TimeList = [1.0, 1.5, 2.0];

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
    for j = 1:3
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
plot(route2_waypoint(:,1), route2_waypoint(:,2), 'k--', "LineWidth", 1.5);
plot(global_waypoints(:,1), global_waypoints(:,2), 'b-', "LineWidth", 2);
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