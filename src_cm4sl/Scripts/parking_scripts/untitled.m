function opt_path = hybridAstar_main()
% HYBRIDASTAR_MAIN
%  - No toolbox
%  - Simple Hybrid A* with kinematic car model (forward / reverse)
%  - Obstacles are parked cars whose coordinates are REAR-BUMPER based.
%  - Each obstacle car is modeled as TWO circles along its heading.

    %% 1. World 설정
    x_world_min = 5.5;
    x_world_max = 47.5;
    y_world_min = -45;
    y_world_max = -4;

    space = [x_world_min, x_world_max, y_world_min, y_world_max];

    % Start / Goal (world frame, [x, y, yaw(rad)])
    start_world = [5.5, -36.5, deg2rad(0)];
    goal_world  = [38.8, -6.3,  deg2rad(90)];

    % 장애물: [x_rear, y_rear, heading_deg]  (rear bumper 기준!)
    obstacles = [7.3000,  -28.7000,   -90;
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

    %% 2. Hybrid A* 파라미터
    p = struct();
    p.xyResolution   = 0.5;         % [m] grid resolution
    p.yawResolution  = deg2rad(15); % [rad]
    p.stepSize       = 1.0;         % [m] arc length per motion primitive
    p.R_min          = 5.8;         % [m] 최소 회전반경
    % 차량/장애물 충돌 반경 (대략 값, 튜닝 가능)
    p.vehicleRadius  = 1.0;         % [m] ego 차량 반경 (원 모델)
    p.obsRadius      = 1.0;         % [m] 각 장애물 서클 반경
    % 장애물 차량 길이 (rear 기준 좌표를 위해 필요)
    p.carLengthObs   = 4.47;        % [m] 장애물 차량 길이
    % 비용 관련
    p.reversePenalty = 2.0;         % 후진시 추가 비용 계수
    p.steerSet       = [-1, 0, 1];  % κ / κ_max 값 (좌, 직, 우)
    p.maxIter        = 50000;
    p.goalPosTol     = 1.0;         % [m]
    p.goalYawTol     = deg2rad(20); % [rad]
    p.yawWeight      = 1.0;         % heuristic에서 yaw 가중치

    %% 3. 플래너 실행
    fprintf('Hybrid A* planning...\n');
    [path, success] = hybridAstar_planner(start_world, goal_world, obstacles, space, p);

    if ~success
        warning('Path not found.');
        opt_path = [];
        return;
    end

    opt_path = path; % [N x 3] : [x, y, yaw]

    %% 4. 결과 플로팅
    figure; hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]');
    title('Hybrid A* Result');

    % World boundary
    rectangle('Position', [x_world_min, y_world_min, ...
                           x_world_max - x_world_min, ...
                           y_world_max - y_world_min], ...
              'EdgeColor', [0.5 0.5 0.5]);

    % Obstacles (rear 기준 좌표 → 두 개 서클로 시각화)
    for i = 1:size(obstacles,1)
        drawObstacleCarDual(obstacles(i,1), obstacles(i,2), obstacles(i,3), p);
    end

    % Path
    plot(opt_path(:,1), opt_path(:,2), 'b-', 'LineWidth', 2);

    % Start / Goal
    plot(start_world(1), start_world(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal_world(1),  goal_world(2),  'rx', 'MarkerSize', 10, 'LineWidth', 2);

    legend('World', 'Obstacle car (dual circles)', 'Path', 'Start', 'Goal');
    hold off;
end

%% ================== Hybrid A* Planner ==================
function [path, success] = hybridAstar_planner(start, goal, obstacles, space, p)

    x_min = space(1); x_max = space(2);
    y_min = space(3); y_max = space(4);

    % Grid size
    nx = floor((x_max - x_min) / p.xyResolution) + 1;
    ny = floor((y_max - y_min) / p.xyResolution) + 1;
    nyaw = floor(2*pi / p.yawResolution) + 1;

    % Pre-alloc
    cost3d    = inf(nx, ny, nyaw);   % g-value
    nodeId3d  = zeros(nx, ny, nyaw); % index of node in "nodes" array
    closed3d  = false(nx, ny, nyaw); % closed set flag

    % Node struct template
    nodeTemplate = struct('x',0,'y',0,'yaw',0,...
                          'g',0,'h',0,'f',0,...
                          'dir',1,...    % +1 forward, -1 reverse
                          'parent',0,...
                          'ix',0,'iy',0,'iyaw',0);

    nodes(1) = nodeTemplate; %#ok<AGROW>
    [ix,iy,iyaw] = world2grid(start(1), start(2), start(3), x_min, y_min, p.xyResolution, p.yawResolution);

    nodes(1).x    = start(1);
    nodes(1).y    = start(2);
    nodes(1).yaw  = wrapToPi_local(start(3));
    nodes(1).g    = 0;
    nodes(1).h    = heuristic_cost(start(1), start(2), start(3), goal, p);
    nodes(1).f    = nodes(1).g + nodes(1).h;
    nodes(1).dir  = 1;
    nodes(1).parent = 0;
    nodes(1).ix   = ix;
    nodes(1).iy   = iy;
    nodes(1).iyaw = iyaw;

    cost3d(ix,iy,iyaw)   = 0;
    nodeId3d(ix,iy,iyaw) = 1;

    openSet = 1; % store node indices
    success = false;
    goalId  = -1;

    iter = 0;
    while ~isempty(openSet) && iter < p.maxIter
        iter = iter + 1;

        % --- 현재 노드 선택 (최소 f)
        [~, idxMin] = min([nodes(openSet).f]);
        curId = openSet(idxMin);
        openSet(idxMin) = [];

        curNode = nodes(curId);
        if closed3d(curNode.ix, curNode.iy, curNode.iyaw)
            continue;
        end
        closed3d(curNode.ix, curNode.iy, curNode.iyaw) = true;

        % --- Goal 체크
        if isGoalReached(curNode, goal, p)
            success = true;
            goalId  = curId;
            fprintf('Goal reached. iter=%d, cost=%.2f\n', iter, curNode.g);
            break;
        end

        % --- 이 노드에서 자식 노드 확장
        children = expandNode(curNode, p);

        for k = 1:numel(children)
            child = children(k);

            % 범위 체크
            if child.x < x_min || child.x > x_max || ...
               child.y < y_min || child.y > y_max
                continue;
            end

            % 충돌 체크 (motion 전체에 대해)
            if checkCollisionMotion(curNode, child, obstacles, space, p)
                continue;
            end

            [ix_c,iy_c,iyaw_c] = world2grid(child.x, child.y, child.yaw, ...
                                            x_min, y_min, ...
                                            p.xyResolution, p.yawResolution);

            if closed3d(ix_c,iy_c,iyaw_c)
                continue;
            end

            g_new = curNode.g + child.cost; % arc length + penalty 포함

            if g_new < cost3d(ix_c,iy_c,iyaw_c)
                childNode = nodeTemplate;
                childNode.x    = child.x;
                childNode.y    = child.y;
                childNode.yaw  = wrapToPi_local(child.yaw);
                childNode.g    = g_new;
                childNode.h    = heuristic_cost(childNode.x, childNode.y, childNode.yaw, goal, p);
                childNode.f    = childNode.g + childNode.h;
                childNode.dir  = child.dir;
                childNode.parent = curId;
                childNode.ix   = ix_c;
                childNode.iy   = iy_c;
                childNode.iyaw = iyaw_c;

                if nodeId3d(ix_c,iy_c,iyaw_c) == 0
                    % 새로 추가
                    nodes(end+1) = childNode; %#ok<AGROW>
                    newId = numel(nodes);
                else
                    % 기존 노드 업데이트
                    newId = nodeId3d(ix_c,iy_c,iyaw_c);
                    nodes(newId) = childNode;
                end

                cost3d(ix_c,iy_c,iyaw_c)   = g_new;
                nodeId3d(ix_c,iy_c,iyaw_c) = newId;

                openSet(end+1) = newId; %#ok<AGROW>
            end
        end
    end

    if ~success
        path = [];
        return;
    end

    % --- 경로 backtracking
    path = backtrackPath(nodes, goalId);
end

%% ================== Node expansion ==================
function children = expandNode(node, p)
    % motion primitive: κ ∈ {-1/R, 0, +1/R}, dir ∈ {+1, -1}
    R = p.R_min;
    kappa_max = 1.0 / R;
    ds = p.stepSize;

    steeringSet = p.steerSet * kappa_max; % 실제 κ 값
    dirSet = [1, -1]; % forward, reverse

    children = struct('x',{},'y',{},'yaw',{},'dir',{},'cost',{});

    for i = 1:length(steeringSet)
        kappa = steeringSet(i);
        for d = 1:length(dirSet)
            dir = dirSet(d);
            s   = dir * ds;

            [x_new, y_new, yaw_new] = integrateKinematic(node.x, node.y, node.yaw, kappa, s);

            % cost: arc length + reverse penalty
            baseCost = abs(ds);
            if dir < 0
                baseCost = baseCost * p.reversePenalty;
            end

            c.x    = x_new;
            c.y    = y_new;
            c.yaw  = yaw_new;
            c.dir  = dir;
            c.cost = baseCost;

            children(end+1) = c; %#ok<AGROW>
        end
    end
end

%% ================== Kinematic integration ==================
function [x_new, y_new, yaw_new] = integrateKinematic(x, y, yaw, kappa, s)
    % Bicycle-like kinematics with constant curvature
    if abs(kappa) < 1e-6
        % Straight motion
        x_new   = x + s * cos(yaw);
        y_new   = y + s * sin(yaw);
        yaw_new = yaw;
    else
        % Circular arc
        R = 1.0 / kappa;
        yaw_new = yaw + kappa * s;
        x_new = x + R * (sin(yaw_new) - sin(yaw));
        y_new = y - R * (cos(yaw_new) - cos(yaw));
    end
    yaw_new = wrapToPi_local(yaw_new);
end

%% ================== Heuristic ==================
function h = heuristic_cost(x, y, yaw, goal, p)
    dx = goal(1) - x;
    dy = goal(2) - y;
    dist = sqrt(dx^2 + dy^2);

    dyaw = abs(wrapToPi_local(goal(3) - yaw));
    h = dist + p.yawWeight * dyaw;
end

%% ================== Goal check ==================
function flag = isGoalReached(node, goal, p)
    dx = goal(1) - node.x;
    dy = goal(2) - node.y;
    dist = sqrt(dx^2 + dy^2);

    dyaw = abs(wrapToPi_local(goal(3) - node.yaw));

    flag = (dist < p.goalPosTol) && (dyaw < p.goalYawTol);
end

%% ================== Collision check for motion ==================
function flag = checkCollisionMotion(n1, n2, obstacles, space, p)
    % 중간 샘플 몇 개 찍어서 충돌 체크
    nSamples = 5;
    flag = false;

    % κ 추정 (n1→n2)
    kappa_est = curvatureFromNodes(n1, n2, p);

    for i = 0:nSamples
        t  = i / nSamples;
        s  = t * p.stepSize * sign(n2.dir); % 방향 고려
        [x_s, y_s, ~] = integrateKinematic(n1.x, n1.y, n1.yaw, ...
                                           kappa_est, s);

        if checkCollisionPoint(x_s, y_s, obstacles, space, p)
            flag = true;
            return;
        end
    end
end

function kappa = curvatureFromNodes(n1, n2, p)
    % n1→n2 에 해당하는 κ를 다시 추정 (대충 yaw 변화로 계산)
    ds = p.stepSize;
    dyaw = wrapToPi_local(n2.yaw - n1.yaw);
    if abs(ds) < 1e-6
        kappa = 0;
    else
        kappa = dyaw / (ds * sign(n2.dir));
    end
end

%% ================== Collision check at a point ==================
function flag = checkCollisionPoint(x, y, obstacles, space, p)
    % World boundary
    if x < space(1) || x > space(2) || ...
       y < space(3) || y > space(4)
        flag = true;
        return;
    end

    % 장애물: rear 기준 + heading → 차량 길이 따라 원 2개 배치
    L = p.carLengthObs;
    % 두 서클의 rear 기준 거리 (비율은 적당히 조절 가능)
    d1 = L * 0.3;    % 뒤쪽 쪽
    d2 = L * 0.8;    % 앞쪽 쪽

    effR2 = (p.obsRadius + p.vehicleRadius)^2;

    for i = 1:size(obstacles,1)
        xr = obstacles(i,1);
        yr = obstacles(i,2);
        hdg = deg2rad(obstacles(i,3));

        % 두 개 서클의 중심
        x1 = xr + d1*cos(hdg);
        y1 = yr + d1*sin(hdg);
        x2 = xr + d2*cos(hdg);
        y2 = yr + d2*sin(hdg);

        dx1 = x - x1; dy1 = y - y1;
        dx2 = x - x2; dy2 = y - y2;

        if (dx1*dx1 + dy1*dy1 < effR2) || (dx2*dx2 + dy2*dy2 < effR2)
            flag = true;
            return;
        end
    end

    flag = false;
end

%% ================== Backtracking ==================
function path = backtrackPath(nodes, goalId)
    idx = goalId;
    tmp = [];
    while idx ~= 0
        n = nodes(idx);
        tmp = [n.x, n.y, n.yaw; tmp]; %#ok<AGROW>
        idx = n.parent;
    end
    path = tmp;
end

%% ================== World → Grid index ==================
function [ix, iy, iyaw] = world2grid(x, y, yaw, x_min, y_min, res_xy, res_yaw)
    ix = floor((x - x_min) / res_xy) + 1;
    iy = floor((y - y_min) / res_xy) + 1;
    yaw = wrapToPi_local(yaw);
    iyaw = floor((yaw + pi) / res_yaw) + 1;
end

%% ================== wrapToPi (툴박스 없이 구현) ==================
function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

%% ================== Draw basic circle ==================
function h = drawCircle(x, y, r, color)
    th = linspace(0, 2*pi, 40);
    xc = x + r*cos(th);
    yc = y + r*sin(th);
    h = fill(xc, yc, color, 'FaceAlpha', 0.3, 'EdgeColor', color);
end

%% ================== Draw obstacle car as dual circles ==================
function drawObstacleCarDual(xr, yr, hdg_deg, p)
    L   = p.carLengthObs;
    hdg = deg2rad(hdg_deg);

    d1 = L * 0.3;
    d2 = L * 0.8;

    x1 = xr + d1*cos(hdg);
    y1 = yr + d1*sin(hdg);
    x2 = xr + d2*cos(hdg);
    y2 = yr + d2*sin(hdg);

    % 두 서클을 그려서 차량 형상 근사
    drawCircle(x1, y1, p.obsRadius, [0.7 0.2 0.2]);
    drawCircle(x2, y2, p.obsRadius, [0.7 0.2 0.2]);
end
