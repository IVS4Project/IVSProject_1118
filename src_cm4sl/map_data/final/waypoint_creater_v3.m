clear
clc

load('final_map_raw_data.mat')
load('final_map_waypoint_raw_data.mat')

% route 별 waypoint raw 데이터
route_waypoint_raw{1} = [route1_waypoint_x.Data, route1_waypoint_y.Data];
route_waypoint_raw{2} = [route2_waypoint_x.Data, route2_waypoint_y.Data];
route_waypoint_raw{3} = [route3_waypoint_x.Data, route3_waypoint_y.Data];
route_waypoint_raw{4} = [route4_waypoint_x.Data, route4_waypoint_y.Data];
route_waypoint_raw{5} = [route5_waypoint_x.Data, route5_waypoint_y.Data];

dist = 3;
dist_ = 1.5;
plot_ = false;
% distance_circular = @(p1, p2, route_length) min(abs(p1 - p2), route_length - abs(p1 - p2));
distance_circular = @(p1, p2, route_length) abs(p1 - p2);


% Define a function to calculate Euclidean distance between two points considering circular route

% Create a cell array to store filtered route data
filtered_route_waypoint_raw = cell(size(route_waypoint_raw));

% Iterate through each route
for i = 1:length(route_waypoint_raw)
    % Get the current route's raw waypoint data
    current_route_waypoints = route_waypoint_raw{i};
    
    % Get the length of the route (assuming circular)
    route_length = size(current_route_waypoints, 1);
    
    % Initialize filtered waypoint data for the current route
    filtered_waypoints = current_route_waypoints(1, :); % Start with the first point
    
    % Iterate through each waypoint in the current route
    for j = 2:size(current_route_waypoints, 1)
        % Flag to determine if the current point is too close to any previously added point
        too_close = false;
        
        % Compare the current point with all previously added points
        for k = 1:size(filtered_waypoints, 1)
            % Calculate distance between the current waypoint and a waypoint in the filtered list
            dist_to_last_point = distance_circular(current_route_waypoints(j, 1), filtered_waypoints(k, 1), route_length) + ...
                distance_circular(current_route_waypoints(j, 2), filtered_waypoints(k, 2), route_length);
            
            % If the distance is less than or equal to 1 meter, set the flag and break
            if dist_to_last_point <= dist
                too_close = true;
                break;
            end
        end
        
        % If the current point is not too close to any previously added point, add it to the filtered list
        if ~too_close
            filtered_waypoints = [filtered_waypoints; current_route_waypoints(j, :)];
        end
    end
    
    % Store the filtered waypoint data for the current route
    filtered_route_waypoint_raw{i} = filtered_waypoints;
end

if plot_ == true
for i = 1:length(route_waypoint_raw)
    a = filtered_route_waypoint_raw{i};
    plot(a(:,1),a(:,2),'.','MarkerSize',15)
    hold on
end
end


% Initialize cell array to store filtered route data
input = filtered_route_waypoint_raw;
output = cell(size(route_waypoint_raw));
% Iterate through each route
for i = 1:length(input)
    % Get the current route's raw waypoint data
    current_route_waypoints = input{i};
    
    % Get the length of the route (assuming circular)
    route_length = size(current_route_waypoints, 1)*3;
    
    % Initialize filtered waypoint data for the current route
%     filtered_waypoints = current_route_waypoints(1, :); % Start with the first point
    filtered_waypoints = []; % Start with the first point
    
    % Iterate through each waypoint in the current route
    for j = 2:size(current_route_waypoints, 1)
        % Flag to determine if the current point is too close to any previously added point
        too_close = false;
        
        % Compare the current point with all previously added points from all routes
        for k = 1:length(output)
            if i == k
                continue; % Skip comparing with the same route
            end
            
            % Get the waypoints from other routes
            other_route_waypoints = output{k};
            
            % Iterate through waypoints from other routes
            for l = 1:size(other_route_waypoints, 1)
                % Calculate distance between the current waypoint and a waypoint from another route
                dist_to_other_point = sqrt(distance_circular(current_route_waypoints(j, 1), other_route_waypoints(l, 1), route_length)^2 + ...
                    distance_circular(current_route_waypoints(j, 2), other_route_waypoints(l, 2), route_length)^2);
                
                % If the distance is less than or equal to the threshold, set the flag and break
                if dist_to_other_point <= dist_
                    too_close = true;
                    break;
                end
            end
            
            % If the current point is too close to any previously added point from other routes, break
            if too_close
                break;
            end
        end
        
        % If the current point is not too close to any previously added point, add it to the filtered list
        if ~too_close
            filtered_waypoints = [filtered_waypoints; current_route_waypoints(j, :)];
        end
    end
    
    % Store the filtered waypoint data for the current route
    output{i} = filtered_waypoints;
end

if plot_ == true
for i = 1:length(output)
    a = output{i};
    plot(a(:,1),a(:,2),'.')
    hold on
end
end
%%
% Initialize variables
all_waypoints = {}; % Cell array to store all individual waypoints
road_profiles = {}; % Cell array to store road profiles

% Define the minimum number of waypoints in a road profile
min_num_waypoints = 10;

% Initialize global ID counter for waypoints
global_waypoint_id_counter = 1;

% Iterate through each route
for route_idx = 1:numel(output)
    route_data = output{route_idx}; % Extract route data
    
    % Initialize variables for this route
    current_profile = []; % Array to store waypoints for the current road profile
    
    % Iterate through each waypoint in the route
    for point_idx = 1:size(route_data, 1)
        waypoint_data = route_data(point_idx, :); % Extract waypoint data
        
        % Assign a global ID to the waypoint
        waypoint_data_id = global_waypoint_id_counter;
        
        % Increment the global waypoint ID counter
        global_waypoint_id_counter = global_waypoint_id_counter + 1;
        
        % Add the current waypoint to the all_waypoints structure with ID
        all_waypoints{end+1} = struct('id', waypoint_data_id, 'waypoint', waypoint_data);
        
        % Add the current waypoint to the current profile with ID
        current_profile = [current_profile; [waypoint_data_id, waypoint_data]];
        
        % Check if the current profile has at least 10 waypoints
        if size(current_profile, 1) >= min_num_waypoints
            % Add the current road profile to the road_profiles cell array
            road_profiles{end+1} = struct('id', numel(road_profiles) + 1, 'waypoint', current_profile, 'numWaypoint', size(current_profile, 1));
            % Reset the current profile for the next road profile
            current_profile = [];
        end
    end
    
    % Check if there are remaining waypoints after iteration
    if ~isempty(current_profile)
        % Add the remaining waypoints to the last road profile
        road_profiles{end} = struct('id', numel(road_profiles), 'waypoint', [road_profiles{end}.waypoint;current_profile], 'numWaypoint', size(current_profile, 1)+road_profiles{end}.numWaypoint);
    end
end

%%
% Plot each road profile and waypoints
figure;
hold on;
for profile_idx = 1:numel(road_profiles)
    profile = road_profiles{profile_idx};
    waypoints = profile.waypoint;
    
    % Plot road profile
    plot(waypoints(:, 2), waypoints(:, 3), 'b.-');
    
    % Plot waypoints
    plot(waypoints(:, 2), waypoints(:, 3), 'ro');
    
    % Add labels to waypoints
    for wp = 1:size(waypoints, 1)
        text(waypoints(wp, 2), waypoints(wp, 3), num2str(waypoints(wp)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
end

% Set plot title and labels
title('Extracted Road Profiles and Waypoints');
xlabel('X-coordinate');
ylabel('Y-coordinate');
grid on;
hold off;