%course starting speed
course_start_speed = 1; %m/s

RM25 = Car('RM25.txt', 'RM24Engine.csv');

% Example single run
%{
mich = Track('2018MichiganAXTrack_new.csv', 10, 1);
mich.speed_arr(1) = course_start_speed;
[lap_time, mich] = solve_track(mich, RM25);
speed_vs_distance(mich);
throttle_vs_distance(mich);
%}

[points, sum_pts] = dynamic_pts(RM25);

% Example 1D sweep.
%{
start_val = 0.1;
end_val = 0.9;
resolution = 15;
x_vec = zeros(resolution+1, 1);
time_vec = zeros(resolution+1, 1);
for i=1:resolution+1
    disp(i);
    %mich = Track('accel.csv', 1, 0);
    mich = Track('2018MichiganAXTrack_new.csv', 10, 1);
    mich.speed_arr(1) = course_start_speed;
    x_vec(i) = start_val+(end_val-start_val)*((i-1)/resolution);
    RM25.prop_front = x_vec(i);
    RM25.FDF = x_vec(i);
    [lap_time, mich] = solve_track(mich, RM25);
    time_vec(i) = lap_time;
end

figure('Name','x vs laptime');
plot(x_vec, time_vec);
xlabel('prop front mass');
ylabel('laptime (s)');
set(gca,'FontSize',16)
%}

% Example 1D sweep with data of each simulation displayed
%{
start_val = 0.4;
end_val = 0.5;
resolution = 4;
x_vec = zeros(resolution+1, 1);
time_vec = zeros(resolution+1, 1);
data_vec = Track.empty(resolution+1,0);
for i=1:resolution+1
    disp(i);
    %mich = Track('accel.csv', 1, 0);
    mich = Track('2018MichiganAXTrack.csv', 10, 1);
    mich.speed_arr(1) = course_start_speed;
    x_vec(i) = start_val+(end_val-start_val)*((i-1)/resolution);
    RM25.prop_front = x_vec(i);
    RM25.FDF = x_vec(i);
    [lap_time, mich] = solve_track(mich, RM25);
    time_vec(i) = lap_time;
    data_vec(i) = mich;
end

figure('Name','x vs laptime');
hold on;
for i=1:resolution+1
    label = x_vec(i);
    label = num2str(label);
    plot(data_vec(i).distance_double, data_vec(i).speed_arr, 'DisplayName', label);
end
plot(x_vec, time_vec);
xlabel('distance (m)');
ylabel('speed (m/s)');
hold off;

lgd = legend;
lgd.NumColumns = 2;
%}

% Example of 2D sweep
%{
%adjust these values to adjust the sweep, make sure to also make sure you
%are changing the correct vehicle parameters in the for loops
x_0 = 0.5;
x_f = 2;
x_name = 'Cd';
y_0 = 0;
y_f = 3;
y_name = 'Cl';
resolution = 6; %how many steps we want for x and y. Higher values will run slower
[X,Y] = meshgrid(x_0:(x_f-x_0)/resolution:x_f,y_0:(y_f-y_0)/resolution:y_f);
Z = zeros(resolution+1, resolution+1);
for r = 1:resolution+1
    for c = 1:resolution+1
        mich = Track('2018MichiganAXTrack_new.csv', 10, 1);
        mich.speed_arr(1) = course_start_speed;
        index = (resolution+1)*(r-1)+c;
        x_val = X(r,c);
        y_val = Y(r,c);
        %change this line
        RM25.Cd = x_val;
        RM25.Cl = y_val;
        [lap_time, ~] = solve_track(mich, RM25);
        Z(r, c) = lap_time;
        round_info = ['round ', num2str(index), ' of ', num2str((resolution+1)^2)];
        x_info = ['x: ', num2str(x_val)];
        y_info = ['y: ', num2str(y_val)];
        time_info = ['laptime: ', num2str(lap_time)];
        disp(round_info);
        disp(x_info);
        disp(y_info);
        disp(time_info);
    end
end
figure();
[C, h] = contourf(X,Y,Z,8,'ShowText','on');
xlabel(x_name);
ylabel(y_name);
zlabel('laptime');
hold on;
h.LevelList=round(h.LevelList,1);
%clabel(C,h,'FontSize',40, 'labelspacing', 1000, 'FontName', 'Times New Roman');
%{
scatter(0.347, 0.470, 20, 'r', 'filled');
scatter(0.344, 0.473, 20, 'r', 'filled');
scatter(0.341, 0.476, 20, 'r', 'filled');
scatter(0.337, 0.478, 20, 'r', 'filled');
scatter(0.334, 0.482, 20, 'r', 'filled');
%}
%}

% Example of 2D sweep for competition points
%{
%adjust these values to adjust the sweep, make sure to also make sure you
%are changing the correct vehicle parameters in the for loops
x_0 = 180;
x_f = 200;
x_name = 'mass car';
y_0 = 1.5;
y_f = 1.6;
y_name = 'Tire CoF';
resolution = 6; %how many steps we want for x and y. Higher values will run slower
[X,Y] = meshgrid(x_0:(x_f-x_0)/resolution:x_f,y_0:(y_f-y_0)/resolution:y_f);
Z = zeros(resolution+1, resolution+1);
for r = 1:resolution+1
    for c = 1:resolution+1
        index = (resolution+1)*(r-1)+c;
        x_val = X(r,c);
        y_val = Y(r,c);
        %change this line
        RM25.mass_car = x_val;
        RM25.CoF = y_val;
        [~, sum_pts] = dynamic_pts(RM25);
        Z(r, c) = sum_pts;
        round_info = ['round ', num2str(index), ' of ', num2str((resolution+1)^2)];
        x_info = ['x: ', num2str(x_val)];
        y_info = ['y: ', num2str(y_val)];
        time_info = ['points: ', num2str(sum_pts)];
        disp(round_info);
        disp(x_info);
        disp(y_info);
        disp(time_info);
    end
end
figure();
contourf(X,Y,Z,8,'ShowText','on');
xlabel(x_name);
ylabel(y_name);
zlabel('laptime');
hold on;
%}

% certain values
%{
cg_vec = [0.347, 0.344, 0.341, 0.337, 0.334];
dist_vec = [0.47, 0.473, 0.476, 0.478, 0.482];

for i=1:5
    disp(i);
    %mich = Track('accel.csv', 1, 0);
    mich = Track('2018MichiganAXTrack.csv', 10, 1);
    mich.speed_arr(1) = course_start_speed;
    RM25.CGH = cg_vec(i);
    RM25.prop_front = dist_vec(i);
    [lap_time, mich] = solve_track(mich, RM25);
    disp(lap_time);
end
%}

%{
mich = Track('2018MichiganAXTrack.csv', 10, 1);
mich.speed_arr(1) = course_start_speed;
[lap_time, mich] = solve_track(mich, RM25);
disp(lap_time);
%}

%{
mich = Track('2018MichiganAXTrack.csv', 10, 1);
mich.speed_arr(1) = course_start_speed;
[lap_time, mich] = solve_track(mich, RM25);
new_info = ['new method: ', num2str(lap_time)];
disp(new_info);

mich2 = Track('2018MichiganAXTrack.csv', 10, 1);
mich2.speed_arr(1) = course_start_speed;
[lap_time2, mich2] = solve_track_backup(mich2, RM25);
old_info = ['old method: ', num2str(lap_time2)];
disp(old_info);

figure('Name','Speed vs Distance');
hold on;
plot(mich.distance_double, mich.speed_arr, 'DisplayName', 'new method');
plot(mich2.distance_double, mich2.speed_arr, 'DisplayName', 'old method');
hold off;
lgd = legend;
lgd.NumColumns = 2;

figure('Name','Angular Velocty vs Distance');
hold on;
plot(mich.distance_double, mich.angular_velocity_arr, 'DisplayName', 'new method');
plot(mich2.distance_double, mich2.angular_velocity_arr, 'DisplayName', 'old method');
hold off;
lgd = legend;
lgd.NumColumns = 2;

speed_vs_distance(mich);
throttle_vs_distance(mich);
brake_vs_distance(mich);
long_g_vs_distance(mich);
lat_g_vs_distance(mich);
angular_velocity_vs_distance(mich);
%}

% animation test
%{
mich = Track('2018MichiganAXTrack.csv', 10, 1);
mich.speed_arr(1) = course_start_speed;
[lap_time, mich] = solve_track(mich, RM25);
new_info = ['laptime: ', num2str(lap_time)];
disp(new_info);

animate(mich);
%}