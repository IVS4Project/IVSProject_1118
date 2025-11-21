%% 차량 경로 계획: 차량 크기 반영한 후처리 충돌 검사 포함

%% 1. 차량 및 지도 설정
carLength = 4.47;
carWidth  = 1.97;
radius = 1.1;  % 원 근사 반지름

x_world_min = 5.5;
x_world_max = 47.5;
y_world_min = -45;
y_world_max = -4;

x_range = x_world_max - x_world_min;
y_range = y_world_max - y_world_min;
res = 15;

map = occupancyMap(x_range, y_range, res);
setOccupancy(map, zeros(map.GridSize));

x_offset = -x_world_min;
y_offset = -y_world_min;
world2map = @(pt) [pt(1) + x_offset, pt(2) + y_offset];
map2world = @(pt) [pt(1) - x_offset, pt(2) - y_offset];


%% 2. 상태 공간 및 validator
bounds = [0, x_range; 0, y_range; -pi, pi];
ss = stateSpaceReedsShepp(bounds);
ss.MinTurningRadius = 5.8;

sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;


%% 3. 장애물 추가 (차량형 장애물, 회전 포함)
obsts_world = [7.3000,  -28.7000,   -90;
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


for i = 1:size(obsts_world, 1)
    pos = obsts_world(i, 1:2);
    yaw_deg = obsts_world(i, 3);
    yaw_rad = deg2rad(yaw_deg);
    markRotatedCar(map, pos, yaw_rad, carLength, carWidth, world2map);
    
end


%% 4. 플래너 설정
planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance = 1.0;
planner.MaxIterations = 3000;
planner.GoalReachedFcn = @(~, goalState, newState) ...
    (ss.distance(goalState, newState) < 0.5);


%% 5. 시작/목표 상태 (world → map)
start_world = [5.5, -36.5, deg2rad(0)];
%goal_world  = [38.8, -6.3, deg2rad(90)];
goal_world  = [38.8, -44.5, deg2rad(90)];

start = [world2map(start_world(1:2)), start_world(3)];
goal  = [world2map(goal_world(1:2)),  goal_world(3)];

if ~isStateValid(sv, start)
    error('시작 위치가 맵에서 유효하지 않습니다.');
end
if ~isStateValid(sv, goal)
    error('목표 위치가 맵에서 유효하지 않습니다.');
end


%% 6. 경로 탐색
rng(0);
[pathObj, solnInfo] = plan(planner, start, goal);


%% 7. 결과 시각화 및 후처리 충돌 검사
if solnInfo.IsPathFound
    figure;
    show(map); hold on;
    pathStates = pathObj.States;
    plot(pathStates(:,1), pathStates(:,2), 'b.-', 'DisplayName','Path');

    start_map = world2map(start_world(1:2));
    goal_map  = world2map(goal_world(1:2));
    plot(start_map(1), start_map(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
    plot(goal_map(1),  goal_map(2),  'rx', 'MarkerSize', 10, 'DisplayName', 'Goal');
    legend;

    % world 좌표로 변환
    worldStates = zeros(size(pathStates));
    for i = 1:size(pathStates, 1)
        worldStates(i,1:2) = map2world(pathStates(i,1:2));
        worldStates(i,3) = pathStates(i,3);
    end

    % 충돌 검사
    hasCollision = false;
    for i = 1:size(worldStates,1)
        if checkCircleCollisionAtPose(map, worldStates(i,:), world2map, radius)
            fprintf('충돌 발생: %d번째 점 (%.2f, %.2f)\n', i, worldStates(i,1), worldStates(i,2));
            hasCollision = true;
            break;
        end
    end

    if ~hasCollision
        disp("차량 바디 반영한 충돌 없음");
    else
        warning("차량 크기 기준 충돌 발생. 경로 재탐색 필요");
    end

else
    warning('경로를 찾지 못했습니다.');
end


%% ---- Function: 차량 사각형 장애물 추가 ----
function markRotatedCar(map, rear_xy, yaw, carLength, carWidth, world2mapFcn)
    x_extent = [0, carLength];
    y_extent = [-carWidth/2, carWidth/2];
    [xg, yg] = meshgrid(x_extent(1):0.1:x_extent(2), y_extent(1):0.1:y_extent(2));
    local_pts = [xg(:), yg(:)];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    world_pts = (R * local_pts')' + rear_xy;

    map_pts = zeros(size(world_pts));
    for j = 1:size(world_pts, 1)
        map_pts(j, :) = world2mapFcn(world_pts(j, :));
    end

    setOccupancy(map, map_pts, 1);
end


%% ---- Function: 차량 바디 원 충돌 검사 ----
function tf = checkCircleCollisionAtPose(map, pose, world2map, radius)
    localCenters = [1.1, 0; 3.3, 0];
    R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
    worldCenters = (R * localCenters')' + pose(1:2);
    hold on; plot(worldCenters(:,1), worldCenters(:,2), 'mo');

    tf = false;
    for i = 1:size(worldCenters, 1)
        center = worldCenters(i,:);
        mapXY = round(world2map(center));

        if any(mapXY < 1) || any(mapXY > map.GridSize)
            tf = true; return;
        end

        xIdx = (mapXY(1) - ceil(radius)):(mapXY(1) + ceil(radius));
        yIdx = (mapXY(2) - ceil(radius)):(mapXY(2) + ceil(radius));

        for xi = xIdx
            for yi = yIdx
                if xi < 1 || yi < 1 || xi > map.GridSize(1) || yi > map.GridSize(2)
                    continue;
                end
                cellCenter = map.grid2world([xi, yi]);
                if norm(cellCenter - center) <= radius
                    if getOccupancy(map, [xi, yi]) > 0.5
                        tf = true; return;
                    end
                end
            end
        end
    end
end
