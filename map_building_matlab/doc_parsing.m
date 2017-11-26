f_dych = fopen('data_without_words.txt','r');
data = fscanf(f_dych,'%f');
sonar_data = [data(1:10:end) data(2:10:end) data(3:10:end)];
sonar_data = sonar_data * 10;

gyro_data = data(4:10:end);
gyro_data = gyro_data - gyro_data(1);
gyro_data = deg2rad(gyro_data);

count_per_rot = [data(5:10:end) data(8:10:end)];
wheel_position = [data(6:10:end) data(9:10:end)];
wheel_position = wheel_position - wheel_position(1,:);
wheel_speed = [data(7:10:end) data(10:10:end)];

sum = 0;
dist_btw_wheels = 120;
rad_wheel = 27;
time_step = 0.2;