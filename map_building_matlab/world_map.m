extended_kalman;

X_left = zeros(length(sonar_data),1);
Y_left = zeros(length(sonar_data),1);
X_right = zeros(length(sonar_data),1);
Y_right = zeros(length(sonar_data),1);
X_forward = zeros(length(sonar_data),1);
Y_forward = zeros(length(sonar_data),1);

%left sonar
for i=1:length(sonar_data)
    if (sonar_data(i,1) < 400 && sonar_data(i,1) > 100 )
        X_left(i) = X_EKF(1,i) + sonar_data(i,1) * cos(X_EKF(3,i) - pi/2);
        Y_left(i) = X_EKF(2,i) + sonar_data(i,1) * sin(X_EKF(3,i) - pi/2);
    end
end


X_left = X_left(X_left ~= 0);
Y_left = Y_left(Y_left ~= 0);


%right sonar
for i=1:length(sonar_data)
    if sonar_data(i,2) < 550 && sonar_data(i,2) > 100
        X_right(i) = X_EKF(1,i) - sonar_data(i,2) * cos(X_EKF(3,i) - pi/2);
        Y_right(i) = X_EKF(2,i) - sonar_data(i,2) * sin(X_EKF(3,i) - pi/2);
    end
end

X_right = X_right(X_right ~= 0);
Y_right = Y_right(Y_right ~= 0);


% %forward sonar
% for i=1:length(sonar_data)
%     if sonar_data(i,3) < 400 && sonar_data(i,3) > 100
%         X_forward(i) = X_EKF(1,i) + sonar_data(i,3) * cos(X_EKF(3,i));
%         Y_forward(i) = X_EKF(2,i) + sonar_data(i,3) * sin(X_EKF(3,i));
%     end
% end

w_map = [X_left Y_left ; X_right Y_right];


plot(-X_EKF(1,:),X_EKF(2,:))
title('Trajectory')
xlabel('X, mm')
ylabel('Y, mm')
hold on
scatter(-w_map(:,1),w_map(:,2), 'X')

%creating binary image
w_map = round(w_map);
w_map = [1766 - (w_map(:,1) + 800), w_map(:,2) + 300];

% figure
% % scatter(w_map(:,1),w_map(:,2), 'X')

img = zeros(max(w_map(:,1)), max(w_map(:,2)));
for i=1:length(w_map)
    img(w_map(i,1),w_map(i,2)) = 1;
end


img = imdilate(img, strel('square',50));

% img = 1 - img;
img = fliplr(img);
img = img';
% imshow(img)

[H, T, R] = hough(img);
P  = houghpeaks(H,50);
lines = houghlines(img,T,R,P,'MinLength',10, 'FillGap',270);
figure, imshow(img), hold on
max_len = 0;

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',4,'Color','green');

%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

end

