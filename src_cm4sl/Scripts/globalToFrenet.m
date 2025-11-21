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