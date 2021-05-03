clear;
%filename = 'challenge.mp4';
%filename = 'solidWhiteRight.mp4';
filename = 'solidYellowLeft.mp4';
%filename = 'my_lane.mp4';
VideoSource = vision.VideoFileReader(filename, 'VideoOutputDataType', 'double');
VideoOut = vision.VideoPlayer('Name', 'Output');

while ~isDone(VideoSource)
    img = step(VideoSource);
    % put your code here
    %% canny edge detection
    % 영상 사이즈 조정
    img_output = imresize(img, 0.5); 
    col = length(img_output(:,1));
    row = length(img_output(1,:));
    img_output = imcrop(img_output,[1 col*3/5 row col]);
    
    img_gray = rgb2gray(img_output);
    img_gray = double(img_gray);
    
    % 영상들에 따라 파라미터 조정
    BW1=Canny_acl(img_gray,3, 3, 0.05, 0.05); 
    
    %% Hough line transform
    [H,T,R] = hough(BW1);
    % 차선을 보다 정확하게 검출하기 위해 Max number of peaks 조절
    P  = houghpeaks(H, 25,'threshold',ceil(0.3*max(H(:))));
    % 차선의 길이에 따라 gap과 length를 조절하여 보다 정확하게 차선 검출
    [lines] = houghlines(BW1,T,R,P,'FillGap',10,'MinLength', 8);
    max_len = 0;
    
    %% Lane selection
    c1 = []; c2 = []; l = [];
    for k = 1:length(lines)
        % 실제 차선의 범위를 조절하여 자신의 차선만 검출
        if(lines(k).theta < 60 && lines(k).theta > -65)
            c1 = [c1; [lines(k).point1 2]];
            c2 = [c2; [lines(k).point2 2]];
            l = [l;lines(k).point1 lines(k).point2];
        end
    end
    img_output = insertShape(img_output, 'Line', l,'Color','green','LineWidth',3);
    img_output = insertShape(img_output, 'Circle', c1);
    img_output = insertShape(img_output, 'Circle', c2);
    
    step(VideoOut, img_output);
end
release(VideoOut);
