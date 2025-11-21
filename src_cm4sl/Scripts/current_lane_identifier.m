function [current_route, lane_idx] = current_lane_identifier(route1, route2, route3, x_ego, y_ego, yaw_ego)

    w_theta = 0.5;
    
    routes = {route1, route2, route3};
    scores = zeros(3,1);
    
    for k = 1:3
        route = routes{k};
        
        % projection distance + lane direction
        [d_proj, theta_lane] = proj_distance(route, x_ego, y_ego);
        
        % heading error
        diff = yaw_ego - theta_lane;
        heading_err = abs(atan2(sin(diff), cos(diff)));
        
        scores(k) = d_proj + w_theta * heading_err;
    end
    
    [~, lane_idx] = min(scores);
    
    current_route = routes{lane_idx};
    disp(lane_idx)
end


function [d_min, theta_lane] = proj_distance(route, x, y)
    N = size(route,1);
    p = [x; y];
    
    d_min = 1e9;
    theta_lane = 0;
    
    for i = 1:N-1
        p1 = route(i,:)';
        p2 = route(i+1,:)';
        
        v = p2 - p1;
        u = p - p1;
        
        denom = v' * v;
        if denom < 1e-9
            continue;
        end
        
        % projection factor
        t = (u' * v) / denom;
        t = max(0, min(1, t));   % clamp to segment
        
        % projected point
        p_proj = p1 + t * v;
        
        % distance
        d = norm(p - p_proj);
        
        if d < d_min
            d_min = d;
            theta_lane = atan2(v(2), v(1));  % segment 방향
        end
    end
end
