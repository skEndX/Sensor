load('gps_wgs84.mat'); % GPS wgs84 데이터 불러오기
len=length(gps(:,1));
start_point=gps(1,:); % GPS 시작점
end_point=gps(len,:); % GPS 종료점

figure(1); zlim([0 100]); hold on; grid on; title('201801730 �����');
xlabel('latitude'); ylabel('longitude'); zlabel('altitude');

plot3(start_point(1), start_point(2), start_point(3),'o',...
    'MarkerFaceColor','b');
plot3(end_point(1), end_point(2), end_point(3),'o',...
    'MarkerFaceColor','r');

for t=1:len
    if(gps(t,4)==2) % DGPS
        plot3(gps(t,1),gps(t,2),gps(t,3),'.y','MarkerSize',5);
    elseif(gps(t,4)==5) % Float RTK
        plot3(gps(t,1),gps(t,2),gps(t,3),'.g','MarkerSize',5);
    elseif(gps(t,4)==4) % RTK
        plot3(gps(t,1),gps(t,2),gps(t,3),'.m','MarkerSize',5);
    elseif(gps(t,4)==1) % data X
        plot3(gps(t,1),gps(t,2),gps(t,3),'.k','MarkerSize',5);
    end
end
view(3);

[utmX, utmY, utmzone, utmhemi]=wgs2utm(gps(:,1),gps(:,2),52,'s');
figure(2); hold on; grid on; title('201801730 �����');
img=imread('map.jpg');
imagesc([min(utmX) max(utmX)],[max(utmY) min(utmY)],img);
xlabel('x (m)'); ylabel('y (m)'); plot(utmX,utmY,'LineWidth',4);
plot(utmX(1,1),utmY(1,1),'o','MarkerFaceColor','b');
plot(utmX(len,1),utmY(len,1),'o','MarkerFaceColor','r');