clear;
clc;
clf;

load("final_map_waypoint_raw_data.mat");

% route 별 waypoint raw 데이터
route1_waypoint_raw = [route1_waypoint_x.Data, route1_waypoint_y.Data];
route2_waypoint_raw = [route2_waypoint_x.Data, route2_waypoint_y.Data];
route3_waypoint_raw = [route3_waypoint_x.Data, route3_waypoint_y.Data];
route4_waypoint_raw = [route4_waypoint_x.Data, route4_waypoint_y.Data];
route5_waypoint_raw = [route5_waypoint_x.Data, route5_waypoint_y.Data];

% route 별 waypoint 필터링 
route1_waypoint_temp = [];
route2_waypoint_temp = [];
route3_waypoint_temp = [];
route4_waypoint_temp = [];
route5_waypoint_temp = [];

for i=1:length(route1_waypoint_raw)
    add_point = true;
    for j=1:size(route1_waypoint_temp,1)
        if sqrt((route1_waypoint_raw(i,1)-route1_waypoint_temp(j,1))^2 + (route1_waypoint_raw(i,2)-route1_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route1_waypoint_temp = [route1_waypoint_temp; route1_waypoint_raw(i,:)];
    end
end

for i=1:length(route2_waypoint_raw)
    add_point = true;
    for j=1:size(route2_waypoint_temp,1)
        if sqrt((route2_waypoint_raw(i,1)-route2_waypoint_temp(j,1))^2 + (route2_waypoint_raw(i,2)-route2_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route2_waypoint_temp = [route2_waypoint_temp; route2_waypoint_raw(i,:)];
    end
end

for i=1:length(route3_waypoint_raw)
    add_point = true;
    for j=1:size(route3_waypoint_temp,1)
        if sqrt((route3_waypoint_raw(i,1)-route3_waypoint_temp(j,1))^2 + (route3_waypoint_raw(i,2)-route3_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route3_waypoint_temp = [route3_waypoint_temp; route3_waypoint_raw(i,:)];
    end
end

for i=1:length(route4_waypoint_raw)
    add_point = true;
    for j=1:size(route4_waypoint_temp,1)
        if sqrt((route4_waypoint_raw(i,1)-route4_waypoint_temp(j,1))^2 + (route4_waypoint_raw(i,2)-route4_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route4_waypoint_temp = [route4_waypoint_temp; route4_waypoint_raw(i,:)];
    end
end

for i=1:length(route5_waypoint_raw)
    add_point = true;
    for j=1:size(route5_waypoint_temp,1)
        if sqrt((route5_waypoint_raw(i,1)-route5_waypoint_temp(j,1))^2 + (route5_waypoint_raw(i,2)-route5_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route5_waypoint_temp = [route5_waypoint_temp; route5_waypoint_raw(i,:)];
    end
end

% 전체 waypoint 필터링
waypoint_temp = [route1_waypoint_temp; route2_waypoint_temp; route3_waypoint_temp; route4_waypoint_temp; route5_waypoint_temp];
waypoint = [];

for i=1:length(waypoint_temp(:,1))
    add_point = true;
    for j=1:size(waypoint,1)
        if sqrt((waypoint_temp(i,1)-waypoint(j,1))^2 + (waypoint_temp(i,2)-waypoint(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        waypoint = [waypoint; waypoint_temp(i,:)];
    end
end

% route 별 필터링 (전체 waypoint에 존재하는지, 다른 route에 동일한 데이터가 존재하는지)
route1_waypoint = [];
route2_waypoint = [];
route3_waypoint = [];
route4_waypoint = [];
route5_waypoint = [];
redundant = [0,0];

for i=1:length(route1_waypoint_temp)
    if ismember(route1_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route1_waypoint_temp(i,:),redundant,'rows')~=1
        route1_waypoint = [route1_waypoint; route1_waypoint_temp(i,:)];
        redundant = [redundant; route1_waypoint_temp(i,:)];
    end
end
for i=1:length(route2_waypoint_temp)
    if ismember(route2_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route2_waypoint_temp(i,:),redundant,'rows')~=1
        route2_waypoint = [route2_waypoint; route2_waypoint_temp(i,:)];
        redundant = [redundant; route2_waypoint_temp(i,:)];
    end
end
for i=1:length(route3_waypoint_temp)
    if ismember(route3_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route3_waypoint_temp(i,:),redundant,'rows')~=1
        route3_waypoint = [route3_waypoint; route3_waypoint_temp(i,:)];
        redundant = [redundant; route3_waypoint_temp(i,:)];
    end
end
for i=1:length(route4_waypoint_temp)
    if ismember(route4_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route4_waypoint_temp(i,:),redundant,'rows')~=1
        route4_waypoint = [route4_waypoint; route4_waypoint_temp(i,:)];
        redundant = [redundant; route4_waypoint_temp(i,:)];
    end
end
for i=1:length(route5_waypoint_temp)
    if ismember(route5_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route5_waypoint_temp(i,:),redundant,'rows')~=1
        route5_waypoint = [route5_waypoint; route5_waypoint_temp(i,:)];
        redundant = [redundant; route5_waypoint_temp(i,:)];
    end
end

% waypoint 구조체 생성
for i=1:length(waypoint(:,1))
    tWaypoint(i).id = i;
    tWaypoint(i).x = waypoint(i,1);
    tWaypoint(i).y = waypoint(i,2);
end

% road segment 구조체 생성
num = 8;
idx_offset = 0;
waypoint_offset = 0;
for i=1:length(route1_waypoint)
    if(rem(i,num)==0)
        tRoadseg(fix(i/num)).id = fix(i/num);
        tRoadseg(fix(i/num)).nWaypoint = num;
        tRoadseg(fix(i/num)).waypoint = waypoint_offset+i-(num-1):waypoint_offset+i;
    elseif (i==length(route1_waypoint))
        tRoadseg(fix(i/num)+1).id = fix(i/num)+1;
        tRoadseg(fix(i/num)+1).nWaypoint = rem(i,num);
        tRoadseg(fix(i/num)+1).waypoint = waypoint_offset+i-rem(i,num)+1:waypoint_offset+i;
    end
end
% tRoadseg의 앞/뒤 waypoint 연결
for i=1:length(tRoadseg)
    tRoadseg(i).nWaypoint = tRoadseg(i).nWaypoint +2;
    tRoadseg(i).waypoint = [tRoadseg(i).waypoint, 0, 0];
    tRoadseg(i).waypoint = circshift(tRoadseg(i).waypoint,1);

    if i==1
        tRoadseg(i).waypoint(1) = tRoadseg(end).waypoint(end);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    elseif i==length(tRoadseg)
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(1).waypoint(2);
    else
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    end
end


idx_offset = length(tRoadseg);
waypoint_offset = waypoint_offset + length(route1_waypoint);
for i=1:length(route2_waypoint)
    if (rem(i,num)==0)
        tRoadseg(idx_offset+fix(i/num)).id = idx_offset+fix(i/num);
        tRoadseg(idx_offset+fix(i/num)).nWaypoint = num;
        tRoadseg(idx_offset+fix(i/num)).waypoint = waypoint_offset+i-(num-1):waypoint_offset+i;
    elseif (i==length(route2_waypoint))
        tRoadseg(idx_offset+fix(i/num)+1).id = idx_offset+fix(i/num)+1;
        tRoadseg(idx_offset+fix(i/num)+1).nWaypoint = rem(i,num);
        tRoadseg(idx_offset+fix(i/num)+1).waypoint = waypoint_offset+i-rem(i,num)+1:waypoint_offset+i;
    end
end
for i=idx_offset+1:length(tRoadseg)
    tRoadseg(i).nWaypoint = tRoadseg(i).nWaypoint +2;
    tRoadseg(i).waypoint = [tRoadseg(i).waypoint, 0, 0];
    tRoadseg(i).waypoint = circshift(tRoadseg(i).waypoint,1);

    if i==idx_offset+1
        tRoadseg(i).waypoint(1) = tRoadseg(end).waypoint(end);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    elseif i==length(tRoadseg)
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(idx_offset+1).waypoint(2);
    else
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    end
end

idx_offset = length(tRoadseg);
waypoint_offset = waypoint_offset + length(route2_waypoint);
for i=1:length(route3_waypoint)
    if(rem(i,num)==0)
        tRoadseg(idx_offset+fix(i/num)).id = idx_offset+fix(i/num);
        tRoadseg(idx_offset+fix(i/num)).nWaypoint = num;
        tRoadseg(idx_offset+fix(i/num)).waypoint = waypoint_offset+i-(num-1):waypoint_offset+i;
    elseif(i==length(route3_waypoint))
        tRoadseg(idx_offset+fix(i/num)+1).id = idx_offset+fix(i/num)+1;
        tRoadseg(idx_offset+fix(i/num)+1).nWaypoint = rem(i,num);
        tRoadseg(idx_offset+fix(i/num)+1).waypoint = waypoint_offset+i-rem(i,num)+1:waypoint_offset+i;
    end
end
for i=idx_offset+1:length(tRoadseg)
    tRoadseg(i).nWaypoint = tRoadseg(i).nWaypoint +2;
    tRoadseg(i).waypoint = [tRoadseg(i).waypoint, 0, 0];
    tRoadseg(i).waypoint = circshift(tRoadseg(i).waypoint,1);

    if i==idx_offset+1
        tRoadseg(i).waypoint(1) = tRoadseg(end).waypoint(end);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    elseif i==length(tRoadseg)
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(idx_offset+1).waypoint(2);
    else
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    end
end


idx_offset = length(tRoadseg);
waypoint_offset = waypoint_offset + length(route3_waypoint);
for i=1:length(route4_waypoint)
    if(rem(i,num)==0)
        tRoadseg(idx_offset+fix(i/num)).id = idx_offset+fix(i/num);
        tRoadseg(idx_offset+fix(i/num)).nWaypoint = num;
        tRoadseg(idx_offset+fix(i/num)).waypoint = waypoint_offset+i-(num-1):waypoint_offset+i;
    elseif(i==length(route4_waypoint))
        tRoadseg(idx_offset+fix(i/num)+1).id = idx_offset+fix(i/num)+1;
        tRoadseg(idx_offset+fix(i/num)+1).nWaypoint = rem(i,num);
        tRoadseg(idx_offset+fix(i/num)+1).waypoint = waypoint_offset+i-rem(i,num)+1:waypoint_offset+i;
    end
end
for i=idx_offset+1:length(tRoadseg)
    tRoadseg(i).nWaypoint = tRoadseg(i).nWaypoint +2;
    tRoadseg(i).waypoint = [tRoadseg(i).waypoint, 0, 0];
    tRoadseg(i).waypoint = circshift(tRoadseg(i).waypoint,1);

    if i==idx_offset+1
        tRoadseg(i).waypoint(1) = tRoadseg(end).waypoint(end);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    elseif i==length(tRoadseg)
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(idx_offset+1).waypoint(2);
    else
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    end
end

idx_offset = length(tRoadseg);
waypoint_offset = waypoint_offset + length(route4_waypoint);
for i=1:length(route5_waypoint)
    if(rem(i,num)==0)
        tRoadseg(idx_offset+fix(i/num)).id = idx_offset+fix(i/num);
        tRoadseg(idx_offset+fix(i/num)).nWaypoint = num;
        tRoadseg(idx_offset+fix(i/num)).waypoint = waypoint_offset+i-(num-1):waypoint_offset+i;
    elseif(i==length(route5_waypoint))
        tRoadseg(idx_offset+fix(i/num)+1).id = idx_offset+fix(i/num)+1;
        tRoadseg(idx_offset+fix(i/num)+1).nWaypoint = rem(i,num);
        tRoadseg(idx_offset+fix(i/num)+1).waypoint = waypoint_offset+i-rem(i,num)+1:waypoint_offset+i;
    end
end
for i=idx_offset+1:length(tRoadseg)
    tRoadseg(i).nWaypoint = tRoadseg(i).nWaypoint +2;
    tRoadseg(i).waypoint = [tRoadseg(i).waypoint, 0, 0];
    tRoadseg(i).waypoint = circshift(tRoadseg(i).waypoint,1);

    if i==idx_offset+1
        tRoadseg(i).waypoint(1) = tRoadseg(end).waypoint(end);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    elseif i==length(tRoadseg)
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(idx_offset+1).waypoint(2);
    else
        tRoadseg(i).waypoint(1) = tRoadseg(i-1).waypoint(end-1);
        tRoadseg(i).waypoint(end) = tRoadseg(i+1).waypoint(1);
    end
end




% drivable boundary 구조체 생성
tBoundary(1).id = 1;
tBoundary(1).x = 5.5;
tBoundary(1).y = -4;
tBoundary(2).id = 2;
tBoundary(2).x = 47.5;
tBoundary(2).y = -3.9;
tBoundary(3).id = 3;
tBoundary(3).x = 47.5;
tBoundary(3).y = -44.9;
tBoundary(4).id = 4;
tBoundary(4).x = 5.5;
tBoundary(4).y = -44.9;


% graph plot
hold on;

plot(waypoint(:,1),waypoint(:,2),'.b');
axis equal;

for i=1:length(tRoadseg)
    plot(waypoint(tRoadseg(i).waypoint(2),1),waypoint(tRoadseg(i).waypoint(2),2),'*');
end
axis equal;

hold off;




