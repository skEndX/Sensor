%% Initial vehicle pose based on world coordinate system
x_v= 13.75;
y_v= 2.5;
theta_vehicle = deg2rad(90);
%% Vehicle information
v = 1; % vehicle speed (m/s)
L = 2.7; % wheelbase (m)

%% Steering constraints
%최대조향각도 limit :-40.95<x<40.95 deg
steer_limit = deg2rad(40.95);
% 최대 조향각속도 limit : 24 deg/s
steer_rate_limit = deg2rad(24);
cmd_sta = deg2rad(0); % initial vehicle steering angle (deg)

%% target path (1/2): setting of left and right lines
path_lx = [linspace(0,10,110),linspace(10,12.5,30),linspace(12.5,12,60)]; % left x
path_ly = [5 * ones(1,110),linspace(5,5,30),linspace(5,0,60)]; % left y
path_rx = [linspace(0,12.5,110),linspace(12.5,15.5,30),linspace(15.5,14.5,60)]; % right x
path_ry = [10.5 * ones(1,110), linspace(10.5,8,30), linspace(8,0,60)]; % right y
%% target path (2/2): waypoint generation
for i = 1:1:length(path_lx) 
    path_cx(i) = (path_lx(i)+path_rx(i))/2;
    path_cy(i) = (path_ly(i)+path_ry(i))/2;
end
path_x = fliplr(path_cx);
path_y = fliplr(path_cy);
%% 밑그림 그리기
figure(); hold on;
set(gca,'color',[0.5, 0.5, 0.5]);
parking_line1 = [0. 5; 20, 5];
plot(parking_line1(:,1),parking_line1(:,2),'y','LineWidth',1.5);
parking_line2 = [0. 10.5; 20, 10.5];
plot(parking_line2(:,1),parking_line2(:,2),'y','LineWidth',1.5);
parking_line4 = [0. 15.5; 20, 15.5];
plot(parking_line4(:,1),parking_line4(:,2),'y','LineWidth',1.5);
parking_line3 = [2.5, 5, 7.5, 10, 12.5, 15, 17.5];
for i = 1 : length(parking_line3)
    plot(parking_line3(i)*ones(2,1),[5,0],'y','LineWidth',1.5);
    plot(parking_line3(i)*ones(2,1),[15.5,10.5],'y','LineWidth',1.5);
end
shape_ox = [0.9 0.9 -0.9 -0.9];
shape_oy = [-2.28, 2.28, 2.28, -2.28];
pos_ox = [1.25, 3.75, 6.25, 8.75, 11.25, 16.25, 18.75, 1.25, 3.75, 6.25, 8.75, 11.25, 13.75, 16.25, 18.75];
pos_oy = [ 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 13, 13, 13, 13, 13, 13, 13, 13];
Obstacle_x = zeros(15,1,4); 
Obstacle_y = zeros(15,1,4);
for i = 1:1: 15
    Obstacle_x(i,:) = shape_ox + ones(1,4)*pos_ox(i);
    Obstacle_y(i,:) = shape_oy + ones(1,4)*pos_oy(i);
    for i = 1:1:15
        obstacle(i) = fill(Obstacle_x(i,:),Obstacle_y(i,:),[0.9 , 0.9, 0.9],'EraseMode','normal');
    end
end

shape_vx = [-2.28, 2.28, 2.28, -2.28]; % 차량 그림 (차량의 중심점 기준)
shape_vy = [0.9 0.9 -0.9 -0.9];
shape_fx = [-0.32 0.32 0.32 -0.32]; % 앞바퀴 그림
shape_fy = [0.20 0.20 -0.20 -0.20];
shape_rx = [-0.32 0.32 0.32 -0.32]; % 뒷바퀴 그림
shape_ry = [0.20 0.20 -0.20 -0.20];
fd = [1.35, 0];
dfx = cos(theta_vehicle)*fd(1) - sin(theta_vehicle)*fd(2);
dfy = sin(theta_vehicle)*fd(1) + cos(theta_vehicle)*fd(2);
rd = [-1.35, 0];
drx = cos(theta_vehicle)*rd(1) - sin(theta_vehicle)*rd(2);
dry = sin(theta_vehicle)*rd(1) + cos(theta_vehicle)*rd(2);
dd = [fd;rd];
vehicle = fill(shape_vx,shape_vy,'w','EraseMode','normal');
fw = fill(shape_fx+fd(1) ,shape_fy+fd(2),'r','EraseMode','normal');
rw = fill(shape_rx+rd(1) ,shape_ry+rd(2),'b','EraseMode','normal');
pd = plot(dd(:,1),dd(:,2),'k','LineWidth',2);
log_fx = zeros(1,2);
log_rx = zeros(1,2);
log_v = zeros(1,8); % 차량의 박스의 모서리 좌표를 저장
pfw = plot(log_fx(:,1),log_fx(:,2),'r.');
prw = plot(log_rx(:,1),log_rx(:,2),'b.');
pfr = plot(log_v(:,1),log_v(:,2),'k.','MarkerSize',0.5);
pfl = plot(log_v(:,3),log_v(:,4),'k.','MarkerSize',0.5);
prr = plot(log_v(:,5),log_v(:,6),'k.','MarkerSize',0.5);
prl = plot(log_v(:,7),log_v(:,8),'k.','MarkerSize',0.5);
%pl4 = fplot(@(x1) x1,'r');
%pl5 = fplot(@(x1) x1,'b');
cx1 = path_x(1); cy1 = path_y(1);
pl6 = plot(cx1,cy1,'go');
plot(path_lx,path_ly,'r:','LineWidth',2);
plot(path_rx,path_ry,'b:','LineWidth',2);
plot(path_x,path_y,'g:','LineWidth',2);

%% Simulation
j = 0;
prev_sta_rad = 0;
k = 0.5;
sim_time = 35;
dt = 0.02;
look_ahead_idx = 5; % current waypoint idx + 5
esp = 10^(-4); % avoiding x0, /0

for i = 0:dt:sim_time
    idx = find_closest_point(x_v+dfx,y_v+dfy,path_x,path_y);
    look_ahead_point = idx + look_ahead_idx;

    cx1 = path_x(idx);
    cy1 = path_y(idx);
    if (idx == length(path_x))          % Check end condition 
        break;
    else
       cx2 = path_x(idx+1) + esp;
       cy2 = path_y(idx+1) + esp;
    end 
    %% Calculation x 
    slope_path = (cy2-cy1)/(cx2-cx1);
    fn_path = @(x1) slope_path * (x1 - cx1) + cy1;   % 차량과 가까운 점(cx1,cy1)을 지나는 직선방정식 "f_path = slope_path(x-cx1)+cy1"
    
    % X, 이탈거리 계산
    X = abs( (slope_path * (x_v) ) + (-1 * (y_v) ) - slope_path * cx1 + cy1) / sqrt(slope_path^2 + 1);
    
    theta_path = atan2(cy2-cy1,cx2-cx1);
    % 이탈거리 부호 판별
    lateral_flag =  (x_v - cx1)*sin(-theta_path) + (y_v - cy1)*cos(-theta_path);
    if lateral_flag > 0
        X = -X;
    end
    %% Calculation alpha
    alpha = atan2(k*X,v);
    
    %% Calculation thete error
    A = (dfy-dry)/(dfx-drx);
    Vehicle_heading = @(x2) A * (x2 - drx-x_v) + dry+y_v;  % 차량의 진행방향(yaw) 직선방정식

    theta_path = atan2(cy2-cy1,cx2-cx1);
    theta_e = theta_path - theta_vehicle;
    
    sta_cmd_rad = alpha + theta_e;
    %% Check constraints
        %------------steering 각속도 Limit------------%

    if abs(sta_cmd_rad-prev_sta_rad) > steer_rate_limit*dt
        if sta_cmd_rad > prev_sta_rad 
                sta_cmd_rad = prev_sta_rad + steer_rate_limit*dt;
        else
                sta_cmd_rad = prev_sta_rad - steer_rate_limit*dt;
        end
    end
    %---------------Steering_angle_Limit---------------%
    if sta_cmd_rad > steer_limit
        sta_cmd_rad = steer_limit;
    elseif sta_cmd_rad < -steer_limit
        sta_cmd_rad = -steer_limit;
    end
    prev_sta_rad = sta_cmd_rad;
    
    cmd_sta = sta_cmd_rad;

    x_v = x_v + v * cos( theta_vehicle + cmd_sta ) * dt;
    y_v = y_v + v * sin( theta_vehicle + cmd_sta ) * dt;
    theta_vehicle = theta_vehicle + v * tan(cmd_sta)/L * dt;
    
    dfx = cos(theta_vehicle)*fd(1) - sin(theta_vehicle)*fd(2);
    dfy = sin(theta_vehicle)*fd(1) + cos(theta_vehicle)*fd(2);
    drx = cos(theta_vehicle)*rd(1) - sin(theta_vehicle)*rd(2);
    dry = sin(theta_vehicle)*rd(1) + cos(theta_vehicle)*rd(2);
   
    updatedFx = [shape_fx(1) * cos(theta_vehicle+cmd_sta) - shape_fy(1) * sin(theta_vehicle+cmd_sta) + dfx + x_v
        shape_fx(2) * cos(theta_vehicle+cmd_sta) - shape_fy(2) * sin(theta_vehicle+cmd_sta) + dfx + x_v
        shape_fx(3) * cos(theta_vehicle+cmd_sta) - shape_fy(3) * sin(theta_vehicle+cmd_sta) + dfx + x_v
        shape_fx(4) * cos(theta_vehicle+cmd_sta) - shape_fy(4) * sin(theta_vehicle+cmd_sta) + dfx + x_v];
    updatedFy = [shape_fx(1) * sin(theta_vehicle+cmd_sta) + shape_fy(1) * cos(theta_vehicle+cmd_sta) + dfy + y_v
        shape_fx(2) * sin(theta_vehicle+cmd_sta) + shape_fy(2) * cos(theta_vehicle+cmd_sta) + dfy + y_v
        shape_fx(3) * sin(theta_vehicle+cmd_sta) + shape_fy(3) * cos(theta_vehicle+cmd_sta) + dfy + y_v
        shape_fx(4) * sin(theta_vehicle+cmd_sta) + shape_fy(4) * cos(theta_vehicle+cmd_sta) + dfy + y_v];
    updatedRx = [shape_rx(1) * cos(theta_vehicle) - shape_ry(1) * sin(theta_vehicle) + drx + x_v
        shape_rx(2) * cos(theta_vehicle) - shape_ry(2) * sin(theta_vehicle) + drx + x_v
        shape_rx(3) * cos(theta_vehicle) - shape_ry(3) * sin(theta_vehicle) + drx + x_v
        shape_rx(4) * cos(theta_vehicle) - shape_ry(4) * sin(theta_vehicle) + drx + x_v];
    updatedRy = [shape_rx(1) * sin(theta_vehicle) + shape_ry(1) * cos(theta_vehicle) + dry + y_v
        shape_rx(2) * sin(theta_vehicle) + shape_ry(2) * cos(theta_vehicle) + dry + y_v
        shape_rx(3) * sin(theta_vehicle) + shape_ry(3) * cos(theta_vehicle) + dry + y_v
        shape_rx(4) * sin(theta_vehicle) + shape_ry(4) * cos(theta_vehicle) + dry + y_v];
    updatedVx = [shape_vx(1) * cos(theta_vehicle) - shape_vy(1) * sin(theta_vehicle) + x_v
        shape_vx(2) * cos(theta_vehicle) - shape_vy(2) * sin(theta_vehicle) + x_v
        shape_vx(3) * cos(theta_vehicle) - shape_vy(3) * sin(theta_vehicle) + x_v
        shape_vx(4) * cos(theta_vehicle) - shape_vy(4) * sin(theta_vehicle) + x_v];
    updatedVy = [shape_vx(1) * sin(theta_vehicle) + shape_vy(1) * cos(theta_vehicle) + y_v
        shape_vx(2) * sin(theta_vehicle) + shape_vy(2) * cos(theta_vehicle) + y_v
        shape_vx(3) * sin(theta_vehicle) + shape_vy(3) * cos(theta_vehicle) + y_v
        shape_vx(4) * sin(theta_vehicle) + shape_vy(4) * cos(theta_vehicle) + y_v];
    
    set(vehicle, 'Xdata', updatedVx,'Ydata', updatedVy);
    set(fw, 'Xdata', updatedFx,'Ydata', updatedFy);
    set(rw, 'Xdata', updatedRx,'Ydata', updatedRy);

    dd = [dfx+x_v, dfy+y_v ; drx+x_v , dry+y_v ];
    j = j+1;
    log_error(j,:) = [X; theta_e];
    log_fw(j,:) = dd(1,:);
    log_rw(j,:) = dd(2,:);
    log_v(j,1:2) = [ updatedVx(1), updatedVy(1)];
    log_v(j,3:4) = [ updatedVx(2), updatedVy(2)];
    log_v(j,5:6) = [ updatedVx(3), updatedVy(3)];
    log_v(j,7:8) = [ updatedVx(4), updatedVy(4)];
    set(pd, 'Xdata', dd(:,1),'Ydata',dd(:,2));
    set(pfw, 'Xdata', log_fw(:,1),'Ydata',log_fw(:,2));
    set(prw, 'Xdata', log_rw(:,1),'Ydata',log_rw(:,2));
    set(pfr, 'Xdata', log_v(:,1),'Ydata',log_v(:,2));
    set(pfl, 'Xdata', log_v(:,3),'Ydata',log_v(:,4));
    set(prr, 'Xdata', log_v(:,5),'Ydata',log_v(:,6));
    set(prl, 'Xdata', log_v(:,7),'Ydata',log_v(:,8));
    set(pl6, 'Xdata', cx1,'Ydata',cy1);

    drawnow limitrate
    axis([0 20 0 20])
end