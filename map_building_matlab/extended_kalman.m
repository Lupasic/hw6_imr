%%
% Extended Kalman filter.
% 
%% Car params
doc_parsing;
%% Algorithm params
%Process
motor_l_var = 5;
motor_r_var = 5;
enc_theta_var = 5;
%Measurement
gyro_var = 1;

%% Initial data

X_EKF = zeros(3,(length(gyro_data)));
P_previous = eye(3);
%for plots
X_model = zeros(3,(length(gyro_data)));
thetas = zeros((length(gyro_data)),1);

%calculate jacobians
syms dsr dsl pre_theta;
m_m = (dsr + dsl)/2;
th = (dsr - dsl) / dist_btw_wheels;
Fu = [diff(m_m * cos(pre_theta + th/2),dsr) diff(m_m * cos(pre_theta + th/2),dsl); diff(m_m * sin(pre_theta + th/2),dsr) diff(m_m * sin(pre_theta + th/2),dsl); diff(th,dsr) diff(th,dsr)];
    

%% Implementation
for i = 2:length(gyro_data)
    %Prediction
    left_ang_speed = wheel_speed(i,1) * 2 * pi / count_per_rot(i,1);
    right_ang_speed = wheel_speed(i,2) * 2 * pi / count_per_rot(i,2);
    
    delta_motor_l = left_ang_speed * rad_wheel * time_step;
    delta_motor_r = right_ang_speed * rad_wheel * time_step;
    mean_motor = (delta_motor_r + delta_motor_l)/2;
    theta_rad = (delta_motor_l - delta_motor_r) / dist_btw_wheels;
    
%     For plots
    thetas(i) = X_model(3,i-1) + theta_rad;

%     
    A = [1 0 0; 0 1 0; 0 0 1];
    Q = [motor_r_var * abs(delta_motor_r) 0 ;0 motor_l_var * abs(delta_motor_l)];
    X_prediction = A * X_EKF(:,i-1) + [mean_motor * cos(X_EKF(3,i-1) + theta_rad/2); mean_motor * sin(X_EKF(3,i-1) + theta_rad/2); theta_rad];    
    
    X_model(:,i) = A * X_model(:,i-1) + [mean_motor * cos(X_model(3,i-1) + theta_rad/2); mean_motor * sin(X_model(3,i-1) + theta_rad/2); theta_rad];

    
    dsr = delta_motor_r;
    dsl = delta_motor_l;
    pre_theta = X_EKF(3,i-1);
    Fu_cur = subs(Fu);
    Fu_cur = double(Fu_cur);
    %
    P_prediction = A * P_previous * A' + Fu_cur * Q * Fu_cur';
    %Update
    C = [0 0 1];
    R = gyro_var ^2;
    K = P_prediction * C' / (C * P_prediction * C' + R);
    X_EKF(:,i) =  X_prediction + K *(gyro_data(i) - C * X_prediction);
    P_cur = (eye(3) - K*C)*P_prediction;
    
    %Change vals
    P_previous = P_cur;
end

%% Plot
time = 1:(length(gyro_data));

% figure
% % Trajectory
% subplot(2,2,1)
% plot(-X_EKF(1,:),X_EKF(2,:))
% title('Trajectory')
% xlabel('X, mm')
% ylabel('Y, mm')
% legend('kalman filter')
% 
% % Angle
% subplot(2,2,2)
% plot(X_EKF(3,:),time,gyro_data, time,thetas', time)
% title('Angle')
% xlabel('angle, rad')
% legend('kalman filer','gyroscope','encoders')
