%% Load the existing filtered waypoint data
load('filtered_waypoints.mat'); 
% route1_waypoint, route2_waypoint, route3_waypoint, route_parking_waypoint

%% Modify route_parking_waypoint
route_parking_waypoint(24:end,1) = route_parking_waypoint(24:end,1) + 1;

route_parking_waypoint(42, :) = [];
% additional_points = [
%     2.2, -35.00;
%     2.5, -35.15;
%     3.0, -35.20;
%     3.2, -35.15;
%     3.4, -35.10;
%     3.6, -35.10;
%     3.7, -35.05;
%     3.8, -35.00;
%     3.9, -35.00;
% ];
% 
% route_parking_waypoint = [route_parking_waypoint; additional_points];

%% Save to new file "waypoint_set.mat"
save('waypoint_set.mat', ...
    'route1_waypoint', ...
    'route2_waypoint', ...
    'route3_waypoint', ...
    'route_parking_waypoint');

figure; hold on; grid on; axis equal;

plot(route1_waypoint(:,1), route1_waypoint(:,2), 'r.-', 'DisplayName','Route 1');
plot(route2_waypoint(:,1), route2_waypoint(:,2), 'g.-', 'DisplayName','Route 2');
plot(route3_waypoint(:,1), route3_waypoint(:,2), 'b.-', 'DisplayName','Route 3');
plot(route_parking_waypoint(:,1), route_parking_waypoint(:,2), 'm.-', 'DisplayName','Parking');

legend show;
title('All Routes Combined');
xlabel('X'); ylabel('Y');

