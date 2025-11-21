clc; clear; close all;

%% ====== WORLD SETTING (네 조건 반영) ======
x_world_min = 5.5;
x_world_max = 47.5;
y_world_min = -45;
y_world_max = -4;

% state = [rear_x, rear_y, yaw]  (뒷범퍼 중심 기준)
start_world = [5.5,  -36.5, deg2rad(0)];
goal_world  = [38.8,  -6.3,  deg2rad(90)];

% obstacles: [rear_x, rear_y, yaw(deg)]  (얘들도 차량 뒷범퍼 중심 기준이라고 가정)
obstacles = [ ...
      7.3000,  -28.7000,   -90;
     12.8000,   -6.8000,   -90;
     21.3000,   -6.6000,   -90;
     30.0000,   -6.5000,   -90;
     41.7000,   -6.3000,   -90;
      6.9000,   -6.9000,   -90;
      7.0000,  -21.8000,    90;
     18.6000,  -21.8000,    90;
     24.3000,  -21.9000,    90;
     36.0000,  -21.9000,    90;
     38.8000,  -21.8000,    90;
     41.8000,  -21.8000,    90;
     12.6000,  -28.8000,   -90;
     24.3000,  -28.9000,   -90;
     41.6000,  -28.9000,   -90;
      9.9000,  -44.4000,    90;
     21.4000,  -44.3000,    90;
     24.5000,  -44.4000,    90;
     36.0000,  -44.4000,    90;
     41.8000,  -44.5000,    90];

%% ====== GLOBAL CONSTANTS ======
x_min = x_world_min;  x_max = x_world_max;
y_min = y_world_min;  y_max = y_world_max;

x_start = start_world(1);
y_start = start_world(2);
theta_start = start_world(3);

x_goal = goal_world(1);
y_goal = goal_world(2);
theta_goal = goal_world(3);

% 차량/장애물 크기 (필요하면 조정)
l_car = 4.5;   % 차량 길이 (front~rear)
w_car = 2.0;   % 차량 폭

obs_len = 4.5; % 장애물(차량) 길이
obs_wid = 2.0; % 장애물(차량) 폭

parking_spot_to_car_ratio = 2.4;
max_iter = 2000;
angle_weighting = 5;

numofpathfound = 0;

% ----- goal 주변 주차 공간 설정 (axis-aligned box, rear 기준) -----
parking_width  = w_car * parking_spot_to_car_ratio;
parking_length = l_car * parking_spot_to_car_ratio;
thres_radius   = parking_width * 1.5;

X0 = x_goal - parking_width/2;
X1 = x_goal + parking_width/2;
Y0 = y_goal - parking_length/2;
Y1 = y_goal + parking_length/2;

th_max = 2*pi;
flag_success = 0;
flag_near_goal = 0;
goal = 0;

% non-holonomic values (rear 기준 bicycle)
dt = 0.1;
v_range_far  = 1:1:10;
v_range_near = thres_radius*(-1):0.1:thres_radius;
steer_range_far  = -0.8:0.2:0.8;  % -45 ~ 45 deg
steer_range_near = -1:0.1:1;      % -60 ~ 60 deg
max_steps = 10;

%% ====== Plotting ======
figure(1); hold on;
xlim([x_min x_max]); ylim([y_min y_max]);
axis equal;

% Start / Goal (rear 위치)
plot(x_start, y_start, 'kx', 'MarkerSize', 15, 'LineWidth', 3);
goal_circle = [x_goal - thres_radius, y_goal - thres_radius, ...
               2*thres_radius, 2*thres_radius];
rectangle('Position', goal_circle, 'Curvature', [1, 1], ...
          'LineStyle', '--', 'EdgeColor', 'c');

% parking space (axis-aligned)
rectangle('Position', [X0 Y0 parking_width parking_length], ...
          'EdgeColor', 'k', 'LineWidth', 2);

% 장애물 시각화 (yaw 반영, OBB)
for i = 1:size(obstacles,1)
    ox   = obstacles(i,1);
    oy   = obstacles(i,2);
    oyaw = deg2rad(obstacles(i,3));
    obs_poly = makeOBB(ox, oy, oyaw, obs_len, obs_wid);
    patch(obs_poly(:,1), obs_poly(:,2), [0.7 0.7 0.7], ...
          'EdgeColor','k','LineWidth',1);
end

drawnow;

%% ====== Node 구조체 (rear 기준) ======
z = zeros(1,1);
node = struct('x',z,'y',z,'theta',z,'x_par',z,'y_par',z,'th_par',z,...
              'pv',z,'pa',z,'pt',z,'dist',z,'index',z,'p_index',z,'path',z);
nodes_created = 1;

node(1).x = x_start;
node(1).y = y_start;
node(1).theta = theta_start;
node(1).x_par = x_start;
node(1).y_par = y_start;
node(1).th_par = theta_start;
node(1).pv = 0; node(1).pa = 0; node(1).pt = 0;
node(1).dist = 0; node(1).index = 1; node(1).p_index = 0;

%% ====== Tree Generation ======
iter = 2;
while (iter <= max_iter)
    if goal
        % goal 도달 후에는 ellipse 안에서 샘플링 (informed RRT* 느낌)
        cmin = sqrt((x_goal - x_start)^2 + (y_goal - y_start)^2);
        if cbest > sqrt(cbest^2 - cmin^2)
            a = cbest/2;
            b = sqrt(cbest^2 - cmin^2)/2;
        else
            b = cbest/2;
            a = sqrt(cbest^2 - cmin^2)/2;
        end

        e_theta = -pi/2 + pi*rand;
        kk = (a*b) / sqrt((b*cos(e_theta))^2 + (a*sin(e_theta))^2);
        maxR = max(1, round(kk));
        r = randi(maxR);
        inclination = atan2(y_goal - y_start, x_goal - x_start);

        if mod(r,2) == 0
            xpoint = x_start + r*cos(e_theta + inclination);
            ypoint = y_start + r*sin(e_theta + inclination);
        else
            xpoint = x_goal - r*cos(e_theta + inclination);
            ypoint = y_goal - r*sin(e_theta + inclination);
        end
        x_rand = xpoint;
        y_rand = ypoint;
        th_rand = th_max*rand;
    else
        % 월드 범위에 맞게 랜덤 샘플 (rear 기준, 음수 y 포함)
        x_rand = x_min + (x_max - x_min)*rand;
        y_rand = y_min + (y_max - y_min)*rand;
        th_rand = th_max*rand;

        % goal 근처 bias
        if flag_success == 0 && flag_near_goal && rand < 0.5
            x_rand = x_goal - 2*thres_radius + 4*thres_radius*rand;
            y_rand = y_goal - 2*thres_radius + 4*thres_radius*rand;
            th_rand = mod(theta_goal - pi/180*15 + pi/180*30*rand, 2*pi);
            if rand < 0.5
                th_rand = mod(-th_rand, 2*pi);
            end
        end
    end

    % ====== 가장 가까운 노드 (rear 기준) ======
    distance = zeros(nodes_created,1);
    for i = 1:nodes_created
        distance(i) = sqrt( (x_rand - node(i).x)^2 + ...
                            (y_rand - node(i).y)^2 + ...
                            (parking_width/pi * angdiff(th_rand, node(i).theta))^2 );
    end
    [~, parent_index] = min(distance);

    x_close     = node(parent_index).x;
    y_close     = node(parent_index).y;
    theta_close = node(parent_index).theta;

    % ====== Motion Primitives (rear frame state) ======
    path = zeros(max_steps, 3);
    path(1,:) = [x_close, y_close, theta_close];

    % [rear_x, rear_y, theta, v, steer, t]
    path_points = zeros(max_steps * length(v_range_far) * length(steer_range_far), 6);
    path_points(1,:) = [x_close, y_close, mod(theta_close, 2*pi), 0, 0, 0];
    pp_iter = 2;

    % goal 주변이면 후진 허용 & steer 조밀하게
    if sqrt((x_close - x_goal)^2 + (y_close - y_goal)^2) <= thres_radius
        v_range = v_range_near;
        steer_range = steer_range_near;
    else
        v_range = v_range_far;
        steer_range = steer_range_far;
    end

    % parent에서 여러 primitive 펼쳐보기
    for v_iter = v_range
        for st_iter = steer_range
            path(1,:) = [x_close, y_close, theta_close];
            for i = 2:max_steps
                % rear 기준 차량 모델
                x_prev  = path(i-1,1);
                y_prev  = path(i-1,2);
                th_prev = path(i-1,3);

                x_next  = x_prev + v_iter*cos(th_prev)*dt;
                y_next  = y_prev + v_iter*sin(th_prev)*dt;
                th_next = th_prev + v_iter*tan(st_iter)*dt/l_car;

                % 충돌체크 (rear pose → center OBB, 장애물 yaw까지 반영)
                if IsObstacle(x_next, y_next, th_next, ...
                              obstacles, l_car, w_car, obs_len, obs_wid, ...
                              x_min, x_max, y_min, y_max)
                    break;
                end

                path(i,:) = [x_next, y_next, th_next];
                path_points(pp_iter,:) = [x_next, y_next, mod(th_next,2*pi), ...
                                          v_iter, st_iter, (i-1)*dt];
                pp_iter = pp_iter + 1;
            end
        end
    end

    % 유효한 path_points만 사용
    valid_idx = 1:(pp_iter-1);
    if numel(valid_idx) <= 1
        iter = iter + 1;
        continue;
    end

    dist_pp = zeros(length(valid_idx),1);
    for k = 1:length(valid_idx)
        idx = valid_idx(k);
        dist_pp(k) = sqrt( (x_rand - path_points(idx,1))^2 + ...
                           (y_rand - path_points(idx,2))^2 + ...
                           (parking_width/pi * angdiff(th_rand, path_points(idx,3)))^2 );
    end

    % 첫 점(부모) 제외하고 최소 거리 선택
    [dist_pp_value, rel_idx] = min(dist_pp(2:end));
    pp_parent_index = valid_idx(rel_idx + 1);  % offset 1

    % 노드가 반복되면 skip (rear 기준)
    if path_points(pp_parent_index,1) == path_points(1,1) && ...
       path_points(pp_parent_index,2) == path_points(1,2) && ...
       mod(path_points(pp_parent_index,3),2*pi) == mod(path_points(1,3),2*pi)
        iter = iter + 1;
        continue;
    end

    % ====== 새 노드 생성 (rear 기준 좌표 저장) ======
    nodes_created = nodes_created + 1;
    node(nodes_created).x      = path_points(pp_parent_index,1);
    node(nodes_created).y      = path_points(pp_parent_index,2);
    node(nodes_created).theta  = mod(path_points(pp_parent_index,3),2*pi);
    node(nodes_created).x_par  = path_points(1,1);
    node(nodes_created).y_par  = path_points(1,2);
    node(nodes_created).th_par = mod(path_points(1,3),2*pi);
    node(nodes_created).pv     = path_points(pp_parent_index,4);
    node(nodes_created).pa     = path_points(pp_parent_index,5);
    node(nodes_created).pt     = path_points(pp_parent_index,6);
    node(nodes_created).index  = nodes_created;
    node(nodes_created).p_index = parent_index;
    node(nodes_created).dist   = node(parent_index).dist + dist_pp_value;

    % 해당 primitive의 궤적(rear_x, rear_y) 저장
    mask = (path_points(:,4) == node(nodes_created).pv) & ...
           (path_points(:,5) == node(nodes_created).pa) & ...
           (path_points(:,6) <= node(nodes_created).pt);
    node(nodes_created).path = flipud(path_points(mask, 1:2));

    % ---- 시각화 (rear 위치 + OBB 차량) ----
    figure(1);
    ss = scatter(node(nodes_created).x, node(nodes_created).y, 'filled');
    ss.SizeData = 20;

    car_angle = node(nodes_created).theta;
    % rear → center 변환
    [car_center_x, car_center_y] = rear2center(node(nodes_created).x, ...
                                               node(nodes_created).y, ...
                                               car_angle, l_car);

    % 차량 사각형
    car_poly = makeOBB(car_center_x, car_center_y, car_angle, l_car, w_car);
    patch(car_poly(:,1), car_poly(:,2), 1, 'FaceColor','none', 'EdgeColor','r', 'LineWidth',1);

    % parking box 안에 차 네 모서리가 전부 들어가면 성공
    car_x1 = car_poly(1,1); car_y1 = car_poly(1,2);
    car_x2 = car_poly(2,1); car_y2 = car_poly(2,2);
    car_x3 = car_poly(3,1); car_y3 = car_poly(3,2);
    car_x4 = car_poly(4,1); car_y4 = car_poly(4,2);

    if IsInParking(car_x1,car_y1,car_x2,car_y2,car_x3,car_y3,car_x4,car_y4,X0,X1,Y0,Y1)
        p_index = node(nodes_created).p_index;
        numofpathfound = numofpathfound + 1;
        allpath(numofpathfound) = node(nodes_created).index;
        line_array = [node(nodes_created).x, node(nodes_created).y];

        while p_index >= 2
            c = [node(p_index).path(:,:)];
            line_array = [line_array; c];
            p_index = node(p_index).p_index;
        end

        iter
        goal = 1;
        cbest = 0;
        for g = 1:size(line_array,1)-1
            d = line_array(g,:) - line_array(g+1,:);
            cbest = cbest + sqrt(d(1)^2 + d(2)^2);
        end
        clear line_array;
        fprintf("Route Found\n");
        flag_success = 1;
    else
        % rear 위치 기준으로 goal 근처 플래그
        if sqrt((node(nodes_created).x - x_goal)^2 + ...
                (node(nodes_created).y - y_goal)^2) <= thres_radius
            flag_near_goal = 1;
        end
    end

    iter = iter + 1;
end

%% ====== 여러 경로 중 best path 선택 및 플롯 (rear 궤적) ======
if numofpathfound > 0
    totaldist = -inf;
    currentdist = 0;
    bestpath = 0;

    for z = 1:numofpathfound
        currentdist = node(allpath(z)).dist;
        if currentdist > totaldist
            totaldist = currentdist;
            bestpath = z;
        end
    end

    p_index = node(allpath(bestpath)).p_index;
    line_array = [node(allpath(bestpath)).x, node(allpath(bestpath)).y];

    while p_index >= 2
        c = [node(p_index).path(:,:)];
        line_array = [line_array; c];
        p_index = node(p_index).p_index;
    end

    plot(line_array(:,1), line_array(:,2), 'k', 'LineWidth', 2);
else
    disp('No path found.');
end

%% ====== Functions ======
% rear → center
function [cx, cy] = rear2center(rx, ry, theta, L)
    cx = rx + (L/2)*cos(theta);
    cy = ry + (L/2)*sin(theta);
end

% center + yaw(rad) + length/width → OBB 4점 (CCW)
function poly = makeOBB(cx, cy, yaw, length, width)
    dx = length/2;
    dy = width/2;

    R = [cos(yaw) -sin(yaw);
         sin(yaw)  cos(yaw)];

    corners_local = [
         dx,  dy;
         dx, -dy;
        -dx, -dy;
        -dx,  dy;
    ];

    poly = (R * corners_local')';
    poly(:,1) = poly(:,1) + cx;
    poly(:,2) = poly(:,2) + cy;
end

% 두 OBB(poly1, poly2)의 충돌 여부 (SAT)
function hit = OBB_Collision(poly1, poly2)
    hit = true;
    polys = {poly1, poly2};

    for p = 1:2
        poly = polys{p};
        for i = 1:4
            j = mod(i,4) + 1;
            edge = poly(j,:) - poly(i,:);
            normal = [-edge(2), edge(1)];  % 수직 벡터

            proj1 = poly1 * normal';
            proj2 = poly2 * normal';

            min1 = min(proj1); max1 = max(proj1);
            min2 = min(proj2); max2 = max(proj2);

            if (max1 < min2) || (max2 < min1)
                hit = false;
                return;
            end
        end
    end
end

% rear pose + 장애물(얘들도 rear 기준) + 맵 범위 → 충돌 여부
function yn = IsObstacle(rx, ry, theta, ...
                          obstacles, l_car, w_car, obs_len, obs_wid, ...
                          x_min, x_max, y_min, y_max)
    yn = 0;

    % 맵 범위 체크 (rear 기준)
    if rx < x_min || rx > x_max || ry < y_min || ry > y_max
        yn = 1;
        return;
    end

    % 차량 OBB (center 기준으로 만든 뒤)
    [cx, cy] = rear2center(rx, ry, theta, l_car);
    car_poly = makeOBB(cx, cy, theta, l_car, w_car);

    % 장애물들과 OBB 충돌 체크
    for i = 1:size(obstacles,1)
        ox   = obstacles(i,1);
        oy   = obstacles(i,2);
        oyaw = deg2rad(obstacles(i,3));

        obs_poly = makeOBB(ox, oy, oyaw, obs_len, obs_wid);

        if OBB_Collision(car_poly, obs_poly)
            yn = 1;
            return;
        end
    end
end

% parking box 안에 네 모서리 다 들어가면 1
function wn = IsInParking(x1,y1,x2,y2,x3,y3,x4,y4,X0,X1,Y0,Y1)
    wn = 0;
    if (x1>X0 && x1<X1 && y1>Y0 && y1<Y1) && ...
       (x2>X0 && x2<X1 && y2>Y0 && y2<Y1) && ...
       (x3>X0 && x3<X1 && y3>Y0 && y3<Y1) && ...
       (x4>X0 && x4<X1 && y4>Y0 && y4<Y1)
        wn = 1;
    end
end
