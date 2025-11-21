function global_waypoints = generateGlobalWaypoints(map_boundary, obstacle_info, start_point, goal_point)

    %=== 맵 바운더리 8x1 → 4x2 변환 ===%
    P = reshape(map_boundary, 2, [])';
    xmin = min(P(:,1));
    xmax = max(P(:,1));
    ymin = min(P(:,2));
    ymax = max(P(:,2));

    %=== 파라미터 ===%
    carLength = 4.47;
    carRadius = 2.0;

    MAX_ITER  = 5000;
    STEP      = 1.0;
    MAX_STEER = deg2rad(30);
    GOAL_THR  = 3.0;

    %=== 장애물 원 근사 ===%
    obstCircles = [];
    for i = 1:size(obstacle_info,1)
        rear_xy = obstacle_info(i,1:2);
        yaw     = deg2rad(obstacle_info(i,3));

        C = [carLength/4, 0;
             3*carLength/4, 0];

        R = [cos(yaw) -sin(yaw);
             sin(yaw)  cos(yaw)];

        centers = (R * C')' + rear_xy;
        obstCircles = [obstCircles; centers, repmat(carRadius,2,1)];
    end

    %=== start / goal ===%
    start = newNodeTemplate();
    start.x   = start_point(1);
    start.y   = start_point(2);
    start.yaw = deg2rad(start_point(3));
    start.parent = -1;

    goal.x    = goal_point(1);
    goal.y    = goal_point(2);
    goal.yaw  = deg2rad(goal_point(3));

    %=== struct 배열 preallocate ===%
    nodes = repmat(newNodeTemplate(), MAX_ITER+1, 1);
    nodes(1) = start;
    node_count = 1;

    goalIdx = -1;

    %=== RRT ===%
    for iter = 1:MAX_ITER

        if rand < 0.2
            sample = [goal.x, goal.y, goal.yaw];
        else
            sample = [randBetween(xmin,xmax), randBetween(ymin,ymax), randBetween(-pi,pi)];
        end

        idx = nearestNode(nodes(1:node_count), sample);
        nodeNear = nodes(idx);

        newNode = steer(nodeNear, sample, STEP, MAX_STEER);

        if checkCollision(newNode, obstCircles, carRadius)
            continue;
        end

        newNode.parent = idx;
        node_count = node_count + 1;
        nodes(node_count) = newNode;

        if norm([newNode.x - goal.x, newNode.y - goal.y]) < GOAL_THR
            goalIdx = node_count;
            break;
        end
    end

    if goalIdx == -1
        error("No valid path found.");
    end

    %=== 경로 복원 ===%
    path = [];
    cur = goalIdx;
    while cur ~= -1
        path = [nodes(cur); path];
        cur = nodes(cur).parent;
    end

    global_waypoints = [[path.x]' [path.y]'];

end


%% === Node 템플릿 ===
function n = newNodeTemplate()
    n = struct( ...
        'x', 0, ...
        'y', 0, ...
        'yaw', 0, ...
        'parent',(-1) ...
    );
end


%% === Helper functions ===
function v = randBetween(a,b)
    v = a + (b-a)*rand;
end

function idx = nearestNode(nodes, sample)
    d = arrayfun(@(n) norm([n.x-sample(1) n.y-sample(2)]), nodes);
    [~, idx] = min(d);
end


function newNode = steer(node, sample, STEP, MAX_STEER)

    newNode = newNodeTemplate();

    dir = atan2(sample(2) - node.y, sample(1) - node.x);
    yaw_err = wrapToPi(dir - node.yaw);
    yaw_err = max(min(yaw_err, MAX_STEER), -MAX_STEER);

    newNode.yaw = node.yaw + yaw_err;
    newNode.x   = node.x + STEP * cos(newNode.yaw);
    newNode.y   = node.y + STEP * sin(newNode.yaw);
end


function isColl = checkCollision(node, obstCircles, carRadius)

    C = [1.5 0; 3.2 0];
    centers = (rotz(node.yaw) * C')' + [node.x node.y];

    isColl = false;
    for k = 1:2
        for j = 1:size(obstCircles,1)
            if norm(centers(k,:) - obstCircles(j,1:2)) < (carRadius + obstCircles(j,3))
                isColl = true;
                return;
            end
        end
    end
end


function R = rotz(a)
    R = [cos(a) -sin(a); sin(a) cos(a)];
end



%% 테스트 입력 (Simulink랑 동일 구조)
map_boundary = [5.5; -4; 47.5; -4; 47.5; -45; 5.5; -45];

start_point = [5.5 -36.5 0];
goal_point  = [38.8 -6.3 90];

obstacle_info = [7.3000,  -28.7000,   -90;
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

%% 경로 생성
wp = generateGlobalWaypoints(map_boundary, obstacle_info, start_point, goal_point);


%% 시각화
figure; hold on; grid on; axis equal;

% 맵
P = reshape(map_boundary, 2, [])';

xmin = min(P(:,1));
xmax = max(P(:,1));
ymin = min(P(:,2));
ymax = max(P(:,2));

rectangle('Position',[xmin ymin ...
                      (xmax - xmin) (ymax - ymin)], ...
          'EdgeColor','k');

% 장애물
carLength = 4.47;
carRadius = 2.0;

for i=1:size(obstacle_info,1)
    rear = obstacle_info(i,1:2);
    yaw  = deg2rad(obstacle_info(i,3));

    C = [carLength/4 0; 3*carLength/4 0];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    W = (R*C')' + rear;

    for k=1:2
        th = linspace(0,2*pi,50);
        plot(W(k,1) + carRadius*cos(th), ...
             W(k,2) + carRadius*sin(th), 'r');
    end
end

% 경로
plot(wp(:,1), wp(:,2), 'b-', 'LineWidth',2);
plot(wp(1,1), wp(1,2), 'go', 'MarkerSize',10);
plot(wp(end,1), wp(end,2), 'rx', 'MarkerSize',10);
title('RRT Visualization Test');
