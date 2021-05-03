clear; clc;
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; % left back wheel speed
RR = exp1_014.Y(5).Data; % right back wheel speed

Yaw_rate = exp1_014.Y(9).Data;
Time = exp1_014.X.Data;

%% calc yaw_drift
yaw_mean = mean(Yaw_rate(1:5944));

%% initial Variable
x(1) = 0; y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % transform to m/s
wk = deg2rad(Yaw_rate-yaw_mean); % remove drift
Ts(1) = 0.001;
for i = 2:length(Time)
    Ts(i) = Time(i) - Time(i-1);
end

%% Dead Reckoning Using Euler
x_1(1) = 0; y_1(1) = 0;
th_1(1) = deg2rad(230);
vk_1 = (RL+RR)/2/3.6; % transform to m/s
wk_1 = deg2rad(Yaw_rate-yaw_mean); % remove drift

for k = 1:length(Time)-1    % 벡터의 길이를 맞추기 위해 -1
    th_1(k+1) = th_1(k) + (wk_1(k)*Ts(k));
    x_1(k+1) = x_1(k) + vk_1(k)*Ts(k)*cos(th_1(k));
    y_1(k+1) = y_1(k) + vk_1(k)*Ts(k)*sin(th_1(k));
end

%% Dead Reckoning Using 2nd Order Runge-Kutta
x_2(1) = 0; y_2(1) = 0;
th_2(1) = deg2rad(230);
vk_2 = (RL+RR)/2/3.6; % transform to m/s
wk_2 = deg2rad(Yaw_rate-yaw_mean); % remove drift
for k = 1:length(Time)-1    % 벡터의 길이를 맞추기 위해 -1
    th_2(k+1) = th_2(k) + (wk_2(k)*Ts(k));
    x_2(k+1) = x_2(k) + vk_2(k)*Ts(k)*cos(th_2(k) + (wk_2(k)*Ts(k)/2));
    y_2(k+1) = y_2(k) + vk_2(k)*Ts(k)*sin(th_2(k) + (wk_2(k)*Ts(k)/2));
end

%% Dead Reckoning Using Exact Method
x_3(1) = 0;
y_3(1) = 0;
th_3(1) = deg2rad(230);
vk_3 = (RL+RR)/2/3.6; % transform to m/s
wk_3 = deg2rad(Yaw_rate-yaw_mean); % remove drift
for k = 1:length(Time)-1    % 벡터의 길이를 맞추기 위해 -1
    th_3(k+1) = th_3(k) + (wk_3(k)*Ts(k));
    x_3(k+1) = x_3(k) + vk_3(k)/wk_3(k)*(sin(th_3(k+1)) - sin(th_3(k)));
    y_3(k+1) = y_3(k) - vk_3(k)/wk_3(k)*(cos(th_3(k+1)) - cos(th_3(k)));
end


subplot(2,1,1); hold on;
title('DR differnces between Exact-Euler & Exact-RK');
plot(1:length(Time),x_3-x_1);   % Exact-Euler
plot(1:length(Time),x_3-x_2);   % Exact-RK
yyaxis right
plot(1:length(Time),vk);
xlabel('Sample Number'); ylabel('Velocity (m/s)');
yyaxis left
ylabel('x Difference (mm)');
legend('Exact x - Euler x', 'Exac x - RK x', 'car velocity');

subplot(2,1,2); hold on;    
plot(1:length(Time),y_3-y_1);   % Exact-Euler
plot(1:length(Time),y_3-y_2);   % Exact-RK
yyaxis right
plot(1:length(Time),vk);
xlabel('Sample Number'); ylabel('Velocity (m/s)');
yyaxis left
ylabel('x Difference (mm)');
legend('Exact x - Euler x', 'Exac x - RK x', 'car velocity');