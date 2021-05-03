clear; clf; close all;
%% image read and grayscale, crop
img_ori = imread('lanedetect.bmp'); % 3채널 uint8
img_gray = rgb2gray(img_ori); % 1채널 unit8
% uint8은 필터링 과정에서 255를 넘어가 overflow 발생. double은 0-1사이 단계화.
img_gray = double(img_gray);
figure(); clf; imshow(uint8(img_gray)); %uint8, double type 데이터를 영상으로 출력
% 기존의 이미지를 자르기 위해 col와 row의 범위를 지정
col = length(img_gray(:,1));
row = length(img_gray(1,:));
img_gray = img_gray(col/2:col, 1:row);
figure(); clf; imshow(uint8(img_gray));

%% gaussian filter
%Smoothing the input image by an Gaussian filter

g1 = fspecial('gaussian', [5, 5], 5);
g2 = fspecial('gaussian', [5, 5], 10);
g3 = fspecial('gaussian', [11, 11], 5);
img1 = filter2(g1,img_gray, 'same');
img2 = filter2(g2,img_gray, 'same');
img3 = filter2(g3,img_gray, 'same');
figure(); clf;
subplot(2,2,1)
imshow(uint8(img_gray));
title('Original Image', 'FontSize', 10);
subplot(2,2,2)
imshow(uint8(img1));
title('Gaussian filtered Image, size = 5x5, \sigma = 5', 'FontSize', 10);
subplot(2,2,3)
imshow(uint8(img2));
title('Gaussian filtered Image, size = 5x5, \sigma = 10', 'FontSize', 10);
subplot(2,2,4)
imshow(uint8(img3));
title('Gaussian filtered Image, size = 11x11, \sigma = 5', 'FontSize', 10);

%% Calculating gradient with sobel mask

sobelMaskX = [-1, 0, 1; -2, 0, 2; -1, 0, 1];
sobelMaskY = [1, 2, 1; 0, 0, 0; -1, -2, -1];
% Convolution with horizontal and vertical filter
G_X = conv2(img1, sobelMaskX, 'same');
% same, G_X = filter2(sobelMaskX, img1, 'same');
G_Y = conv2(img1, sobelMaskY, 'same');
% same, G_Y = filter2(sobelMaskY, img1, 'same');
figure(); clf;

% x 방향 (수평방향)으로의 미분 : 수직방향의 에지 검출
subplot(211);imshow(uint8(G_X));title('G_x');
% y 방향 (수직방향)으로의 미분 : 수평방향의 에지 검출
subplot(212);imshow(uint8(G_Y));title('G_y');

%% Magnitude of the gradients
% (수평 및 수직 성분 기울기의 크기 = 에지의 세기)
% (Euclidean distance = 대각선의 크기, 대각선 에지에 민감)
% 에지의 세기가 클수록 하얀색에 가깝고 명확한 에지(경계선)를 의미
% 에지의 세기가 작을수록 검정색에 가깝고 불명확한 에지(경계선)를 의미

%Calcultae magnitude of edge
magnitude = sqrt((G_X.^2) + (G_Y.^2));
figure(); clf; imshow(uint8(magnitude));
title('Magnitude : sqrt(G_x^2 + G_y^2)');

%% Angle (orientation) of the gradients
% 픽셀들 사이의 차가 큰 쪽이 미분값이 크고
% 에지(경계선)에 수직인 픽셀들이 미분값이 클 확률이 높으므로 방위각은
% 에지(경계선)에 거의 수직

%Calculate directions/orientations
theta = atan2(G_Y, G_X);
theta = theta * 180/pi;
%Adjustment for negative directions, making all directions positive
col = length(img_gray(:,1));
row = length(img_gray(1,:));
for i=1:col
    for j=1:row
        %     한 점의 기울기로부터 Magnitude
        %     와 θ 를 구하고
        %     음수인 θ 가 나오면 360 [Deg] 를
        %     더하여 양수가 되도록 함.
        
        if (theta(i,j)<0)
            theta(i,j)= 360 + theta(i,j);
            % 0 <= theta <= 360
        end
    end
end

%% quantization theta
qtheta = zeros(col, row);
% Adjusting directions to nearest 0, 45, 90, or 135 degree
for i = 1 : col
    for j = 1 : row
        %     앞에서 구한 각도를 그룹화.
        %     각의 범위 (0 ~ 360˚) 를 4 가지 구간 (0, 1, 2, 3) 으로
        %     나누어 양자화. 0,45,90,135도와 가까운 각도를 기준.
        %     총 8 방향 (상하좌우 4개, 대각선 4개)
        %     각 구간은 시작 각도 ~ 끝 각도로 이루어 짐
        if (((theta(i, j) >= 0 ) && (theta(i, j) < 22.5 )) || ...
                ((theta(i, j) >= 157.5) && (theta(i, j) < 202.5)) || ...
                ((theta(i, j) >= 337.5) && (theta(i, j) <= 360 )))
            qtheta(i, j) = 0; % degree group 0
        elseif (((theta(i, j) >= 22.5) && (theta(i, j) < 67.5 )) || ...
                ((theta(i, j) >= 202.5)&& (theta(i, j) < 247.5)))
            qtheta(i, j) = 1; % degree group 1
        elseif (((theta(i, j) >= 67.5 && theta(i, j) < 112.5)) || ...
                ((theta(i, j) >= 247.5 && theta(i, j) < 292.5)))
            qtheta(i, j) = 2; % degree group 2
        elseif (((theta(i, j) >= 112.5 && theta(i, j) < 157.5)) || ...
                ((theta(i, j) >= 292.5 && theta(i, j) < 337.5)))
            qtheta(i, j) = 3; % degree group 3
        end
    end
end

%% Non-Maximum Supression
% 지역적으로 최대값이 아닌 에지(경계선)를 제거하는 과정.
% 에지에 기여하지 않는 픽셀의 제거 (0 으로 만듬).
% (진짜 에지가 아닌데도 검출되는 픽셀들이 있기 때문.
% Blur 현상. Sharp한 에지로 변경 필요함.)

BW = zeros (col, row);
for i=2:col-1
    for j=2:row-1
        if (qtheta(i,j)==0)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ...
                magnitude(i,j+1), magnitude(i,j-1)]));
        elseif (qtheta(i,j) == 1)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ...
                magnitude(i+1,j-1), magnitude(i-1,j+1)]));
        elseif (qtheta(i,j) == 2)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ...
                magnitude(i+1,j), magnitude(i-1,j)]));
        elseif (qtheta(i,j) == 3)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ...
                magnitude(i+1,j+1), magnitude(i-1,j-1)]));
            %현재 픽셀 값이 해당 각 범위 내의 값 중에서 최대면 1, 아니면 0
        end
    end
end
BW = BW.*magnitude;
figure(); imshow(BW);
title('BW');

%% Hysteresis Thresholding
% Strong 에지 픽셀들만 에지로 처리할 경우 영상의 잡음으로 인해
% 에지 픽셀임에도 제대로 인식되지 않을 수 있음.
% 그래서, strong 에지 픽셀과 인접한 weak 에지 픽셀을 정상적인
% 에지로 처리하여 에지 픽셀들 끼리 서로 연결하기 위함.

T_min = 0.1; T_max = 0.2;
T_min = T_min * max(max(BW)); 
T_max = T_max * max(max(BW)); % 전체 픽셀값의 최대
edge_final = zeros (col, row);
for i = 1 : col
    for j = 1 : row
        if (BW(i, j) < T_min) % no edge 현재 픽셀이 최소 임계점 미만이면 제거 (weak 에지)
            edge_final(i, j) = 0;
        elseif (BW(i, j) > T_max) % Strong edge 현재 픽셀이 최대 임계점을 초과하면 사용(strong 에지)
            edge_final(i, j) = 1;
            % Weak edge - Using 8-connected components
            % 현재 픽셀 이외의 인근 픽셀(8개)이 최대 임계점을 초과하면 사용
            %(주변이 모두 strong, 현재 픽셀도strong 하다고 판단)
        elseif (BW(i+1,j)>T_max || BW(i-1,j)>T_max || BW(i,j+1)>T_max ...
                || BW(i,j-1)>T_max || BW(i-1, j-1)>T_max || BW(i-1, j+1)>T_max ...
                || BW(i+1, j+1)>T_max || BW(i+1, j-1)>T_max)
            edge_final(i,j) = 1;
        end
    end
end
%Edge_final 은 0 (흑) 또는 1 (백) 이므로 255 를 곱하여 uint8의 극치 (최대 or 최소) 를 갖게 함.
img_canny = uint8(edge_final.*255);
output_img = img_canny;
figure(); clf; imshow(output_img);

%% edge function
img_ori = imread('lanedetect.bmp');
img_gray = rgb2gray(img_ori);
img_gray = img_gray(length(img_gray(:,1))/2:end, 1:end);
img_edge = edge(img_gray, 'Canny', [0.1 0.2], 0.5);
figure(); imshow(img_edge);

%% Hough transform

% 이진 영상 img_canny에 대해 표준 허프 변환(SHT)을 계산
% hough 함수는 r = x*cos(theta) + y*sin(theta)를 사용하여 직선을 검출
% 양자화 된 값인 표준 허프 변환 H를 반환. Row는 r 값에, column은 theta값에 대응.
% Cell의 값은 x-y 평면에 있는 Theta와 rho로 지정된 직선 위에 몇 개의 점이 있는지 알려줌
[H,T,R] = hough(img_canny); %허프행렬 : 주어진 ?, r 에 있는 x-y 공간상의 점의 개수 리턴
P = houghpeaks(H, 35,'threshold',ceil(0.3*max(H(:))));
[lines] = houghlines(img_canny,T,R,P,'FillGap',10,'MinLength', 8);
% x-y domain에서 점 p1을 지나는 모든 직선들은 
% Hough domain에서는 모두점이 되고, 이들 점들을 연결하면 곡선이 됨

%% lane selection 1
c1 = []; c2 = []; l = [];
for k = 1:length(lines)
    c1 = [c1; [lines(k).point1 3]]; % 시작점의 좌표
    c2 = [c2; [lines(k).point2 3]]; % 끝점의 좌표
    l = [l; lines(k).point1 lines(k).point2];
end
% Insert shapes in image
img_line = insertShape(uint8(img_gray), 'Line', l, 'Color','green', 'LineWidth',3);
img_line = insertShape(uint8(img_line), 'Circle', c1, 'Color','red');
img_line = insertShape(uint8(img_line), 'Circle', c2, 'Color','yellow');
figure();
imshow(img_line);

%% lane selection 2

c1 = []; c2 = []; l = [];
for k = 1:length(lines)
    % 실제 차선이 존재하는 각도 범위
    if(lines(k).theta < 75 && lines(k).theta > -75)
        c1 = [c1; [lines(k).point1 3]];
        c2 = [c2; [lines(k).point2 3]];
        l = [l; lines(k).point1 lines(k).point2];
    end
end
% Edge line을 green색으로시작점은 red끝 점은 yellow로 표현.
img_line2 = insertShape(uint8(img_gray), 'Line', l, 'Color', 'green', 'LineWidth',3);
img_line2 = insertShape(uint8(img_line2), 'Circle', c1, 'Color', 'red');
img_line2 = insertShape(uint8(img_line2), 'Circle', c2, 'Color', 'yellow');
figure();
imshow(img_line2);

