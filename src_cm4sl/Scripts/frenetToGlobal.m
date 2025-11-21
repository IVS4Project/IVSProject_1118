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