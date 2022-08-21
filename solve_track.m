%{
Rensselaer Motorsport Lap Sim
A program intended to evaluate the impact of various vehicle parameters on
laptime.
Author: Justin Qiu
Last Edited 11/26/19
%}

function [time, track] = solve_track(track, car)
% laptime of car on track
% track is an array of segments
original_cof = car.CoF;
car.CoF = original_cof*car.driver_skill;

for i = 1:track.num_seg
    track = solve_segment_forward(track, car, i);
end

% delete start speed and start angular velocity for segment past last
% segment which were inadvertently assigned. Doing it now is faster than
% checking every single segment to see if we are at the end
track.speed_arr(end) = [];
track.angular_velocity_arr(end) = [];

for i=track.num_seg-1:-1:2
    track = solve_segment_backward(track, car, i);
end

time = sum(track.time_arr);

car.CoF = original_cof;

end

%==========================================================================
%==========================================================================
%==========================================================================
% Functions to Solve Forward
%==========================================================================
%==========================================================================
%==========================================================================

function track = solve_segment_forward(track, car, i)
[can_make, track] = can_make_forward(0, track, car, i);
if can_make == 0
    [can_make, track] = ...
        can_make_forward(1, track, car, i);
    if (can_make == 1) || (can_make == 2)
        % car did not have enough long force
        % we know the optimal amount of throttle is between 0 & 1
        track = find_throttle(0,1, track, car, i);
    elseif can_make == 3
        % car did not have enough lateral force
        track = find_throttle(0,1, track, car, i); 
    else
        % can make corner at full throttle
        %{
        % DRS TEST
        orig_CD = car.Cd;
        car.Cd = 1;
        [can_make, track] = can_make_forward(1, track, car, i);
        car.Cd = orig_CD;
        if can_make == 0
            return;
        else
            [~, track] = can_make_forward(1, track, car, i);
        end
        %}
        return;
    end
else
    % cant make corner, even at 0 throttle
    % restart the segment at maximum speed
    % use 0 throttle for simplicity
    if (i == 1)
        prev_speed = 50;
    else
        prev_speed = track.speed_arr(2*i-2);
    end
    track = restart_at_segment(0, prev_speed, track, car, i);
end
end

function track = find_throttle(lower_throttle, upper_throttle, track, car, i)
% finds the maximum throttle we can nagivate a corner at to within 1
% percent, given that we know the optimum amount of throttle is between 0
% and 1
if (upper_throttle - lower_throttle) < 0.01
    [~, track] = can_make_forward(lower_throttle, track, car, i);
    %we have found, at least close enough, what the throttle should be
    return;
else
    [can_make, track] = can_make_forward(0.5*(lower_throttle+upper_throttle), track, car, i);
    if ~can_make
        %we know the possible throttle positions are in the upper half
        track = find_throttle(0.5*(lower_throttle+upper_throttle), upper_throttle, track, car, i);
    else
        %we know the possible throttle positions are in the lower half
        track = find_throttle(lower_throttle, 0.5*(lower_throttle+upper_throttle), track, car, i);
    end
end
end

function track = restart_at_segment(lower_speed, upper_speed, track, car, i)
%if we found we cannot take a segment at 0 throttle, we must start again
%from the maximum start speed and end with the maximum end speed.
%yaw acceleration and throttle will be 0 for simplicity
if (upper_speed - lower_speed ) < 0.2
    track.speed_arr(2*i-1) = lower_speed;
    if (i > 1)
        track.angular_velocity_arr(2*i-1) = track.speed_arr(2*i-1)*track.inv_rad_arr(i-1);
    else
        track.angular_velocity_arr(2*i-1) = 0;
    end
    
    if (i>1) && (lower_speed > track.speed_arr(2*i-2))
        warning('yaw inertia bullshit is occuring');
    end
    [~, track] = ...
        can_make_forward(0, track, car, i);
else
    average_speed = 0.5*(lower_speed+upper_speed);
    % reassign start angular velocity:
    if (i > 1)
        track.angular_velocity_arr(2*i-1) = average_speed*track.inv_rad_arr(i-1);
    else
        track.angular_velocity_arr(2*i-1) = 0;
    end
    track.speed_arr(2*i-1) = average_speed; %reassign start speed
    [can_make, track] = can_make_forward(0, track, car, i);
    if ~can_make
        %we know the highest starting speed is in the upper half
        track = restart_at_segment(0.5*(lower_speed+upper_speed), upper_speed, track, car, i);
    else
        %we know the highest starting speed is in the lower half
        track = restart_at_segment(lower_speed, 0.5*(lower_speed+upper_speed), track, car, i);
    end
end
end

function [can_make, track] = ...
can_make_forward(throttle, track, car, i)
% ADD DESCRIPTION HERE
start_speed = track.speed_arr(2*i-1);
if start_speed < 0
    error('Error: Vehicle speed drops too low');
end
avail_engine_force = interp1(car.vehicle_speed, car.max_wheel_force, start_speed,'linear');
%takes wheel force & aero drag using start speed, not perfect
engine_force = avail_engine_force*throttle; %N

[can_make, track] = ...
can_make_corner(0, 0, engine_force/2, engine_force/2, track, car, i);

track.throttle_arr(i) = throttle;
%assign start angular velocity for next segment:
track.angular_velocity_arr(2*i+1) = track.angular_velocity_arr(2*i);
end

%==========================================================================
%==========================================================================
%==========================================================================
% BOTH
%==========================================================================
%==========================================================================
%==========================================================================

function [can_make, track] = ...
    can_make_corner(F_fr, F_fl, F_rr, F_rl, track, car, i)
% given the 4 longitudinal wheel forces, returns int giving
% or not the car can make the corner. 0 is can make corner. 
% Positive forces are in the
% forward direction, negative in the backwards direction
start_speed = track.speed_arr(2*i-1);
aerodrag = 0.5*car.Cd*car.A*car.rho*start_speed^2; %N
long_force = F_fr + F_fl + F_rr + F_rl - aerodrag; %N, positive is forward
long_accel = long_force/get_mass(car); %m/s^2, positive is forward
end_speed = sqrt(start_speed^2 + 2*long_accel*track.len_arr(i)); %m/s
critical_speed = max(start_speed, end_speed); %the speed we need to worry about
lat_accel = critical_speed^2*track.inv_rad_arr(i); %m/s^2, + for RH corner
lat_force = lat_accel*get_mass(car); %N, + for RH corner
time_elapsed = (end_speed-start_speed)/long_accel; %segment time
%==========================================================================
%yaw calcs
start_angular_velocity = track.angular_velocity_arr(2*i-1);
delta_theta = track.len_arr(i)*track.inv_rad_arr(i);
yaw_accel = 2*(delta_theta-start_angular_velocity*time_elapsed)/(time_elapsed^2); %rad/s^2 + is right
yaw_moment = yaw_accel*car.yaw_inertia; %Nm
end_angular_velocity = 2*delta_theta/time_elapsed - start_angular_velocity;

%==========================================================================
if ((yaw_accel > 0) && (track.inv_rad_arr(i) > 0)) || ...
        ((yaw_accel < 0) && (track.inv_rad_arr(i) < 0))
    %yaw accel is in same direction as corner, so front tires exert more
    %lateral force than rear to create moment
    rear_lat_necessary = (yaw_moment-lat_force*get_a(car))/(-car.wheelbase);
    front_lat_necessary = lat_force - rear_lat_necessary;
else
    %yaw accel is in opposite direction as corner, so rear tires exert more
    %lateral force than front to create moment
    rear_lat_necessary = (yaw_moment+lat_force*get_a(car))/(car.wheelbase);
    front_lat_necessary = lat_force - rear_lat_necessary;
end
%==========================================================================

%find wheel forces based on long and lat accel
downforce = 0.5*car.Cl*car.A*car.rho*min(end_speed, start_speed)^2;
%for moment_long, car right is positive
moment_long = car.CGH*(F_fr + F_fl + F_rr + F_rl) + (car.cp_height-car.CGH)*aerodrag; %Nm
% w_rear from taking moment about front axle
w_rear = ...
    (moment_long + get_aero_pos(car)*downforce + get_a(car)*get_mass(car)*9.81)/car.wheelbase; %N
w_front = get_mass(car)*9.81 + downforce - w_rear;
% moment_lat - rearward is positive (aka positive for RH turn)
moment_lat = lat_force*car.CGH; %total moment caused by lateral forces
%WT is defined as the difference in force between a side or axle in
%equilibrium would have compared to when it is countering the moment
%produced by acceleration
WTfront = (moment_lat/2)/car.f_track; %N, positve for RH corner
WTrear = (moment_lat/2)/car.r_track; %N, positve for RH corner
%We are assuming the roll moment is equally distributed between the axles
%Should add roll centers/ roll rates to improve accuracy in future
%tire forces without load sensitivity:
w_front_right = w_front/2-WTfront;
w_front_left = w_front/2+WTfront;
w_rear_right = w_rear/2-WTrear;
w_rear_left = w_rear/2+WTrear;
if (w_front_right < 0) || (w_front_left < 0) || (w_rear_right < 0) || (w_rear_left < 0)
    %car has flipped
    %acceleration be too aggresive
    can_make = 1;
    return;
end
% Max possible wheel forces:
front_right = (car.CoF-car.loadS*w_front_right)*w_front_right; %N
front_left = (car.CoF-car.loadS*w_front_left)*w_front_left; %N
rear_right = (car.CoF-car.loadS*w_rear_right)*w_rear_right; %N
rear_left = (car.CoF-car.loadS*w_rear_left)*w_rear_left; %N
%make sure load sensitivity does not make the max forces negative
front_right = max(front_right, 0);
front_left = max(front_left, 0);
rear_right = max(rear_right, 0);
rear_left = max(rear_left, 0);
% Start to find remaining lateral force
front_right_lat = front_right^2-(F_fr)^2;
front_left_lat = front_left^2-(F_fl)^2;
rear_right_lat = rear_right^2-(F_rr)^2; % N^2, must take square root
rear_left_lat = rear_left^2-(F_rl)^2; % N^2
if (front_right_lat < 0) || (front_left_lat < 0) || (rear_right_lat < 0) || (rear_left_lat < 0)
    %long force is more than tire can generate
    can_make = 2;
    return;
end
front_right_lat = sqrt(front_right_lat); % N
front_left_lat = sqrt(front_left_lat); % N
rear_right_lat = sqrt(rear_right_lat); % N
rear_left_lat = sqrt(rear_left_lat); % N
front_lat_total = front_right_lat+front_left_lat; % N, lateral
rear_lat_total = rear_right_lat + rear_left_lat;

%check if we have enough available lateral force
if (abs(rear_lat_necessary) < rear_lat_total) && (abs(front_lat_necessary) < front_lat_total)
    can_make = 0;
    track.speed_arr(2*i) = end_speed;
    track.speed_arr(2*i+1) = end_speed; %assign start speed for next segment
    track.time_arr(i) = time_elapsed;
    track.angular_velocity_arr(2*i) = end_angular_velocity;
    track.lat_g_arr(i) = lat_accel/9.81;
    track.long_g_arr(i) = long_accel/9.81;
    
    track.max_fr(i) = front_right;
    track.max_fl(i) = front_left;
    track.max_rr(i) = rear_right;
    track.max_rl(i) = rear_left;
    
    track.f_fr(i,1) = F_fr;
    track.f_fr(i,2) = (front_lat_necessary/front_lat_total)*front_right_lat;
    track.f_fl(i,1) = F_fl;
    track.f_fl(i,2) = (front_lat_necessary/front_lat_total)*front_left_lat;
    track.f_rr(i,1) = F_rr;
    track.f_rr(i,2) = (rear_lat_necessary/rear_lat_total)*rear_right_lat;
    track.f_rl(i,1) = F_rl;
    track.f_rl(i,2) = (rear_lat_necessary/rear_lat_total)*rear_left_lat;
else
    can_make = 3;
end
end

%==========================================================================
%==========================================================================
%==========================================================================
% Functions to Solve Backward
%==========================================================================
%==========================================================================
%==========================================================================

function track = solve_segment_backward(track, car, i)
segment_end_speed = track.speed_arr(2*i);
next_segment_start_speed = track.speed_arr(2*i+1);
if abs(segment_end_speed - next_segment_start_speed) < 0.05
    %speed of next segment is same as end speed of current segment, so no
    %braking event necessary
   return;
else
    %reassign ending speeds and angular velocities of braking zone
    track.speed_arr(2*i) = next_segment_start_speed;
    track.angular_velocity_arr(2*i) = track.angular_velocity_arr(2*i+1);
    track.throttle_arr(i) = 0;
    track = find_brake(0, get_max_brake_force(car), track, car, i);
end
end

function track = find_brake(lower_brake, upper_brake, track, car, i)
if (upper_brake-lower_brake) < 1
    [~, track] = can_make_backward(lower_brake, track, car, i);
    if track.speed_arr(2*i-1) > track.speed_arr(2*i-2)
        %more braking has occurred than necessary
        track.speed_arr(2*i-1) = track.speed_arr(2*i-2);
        elapsed_time = ...
            2*track.len_arr(i)/(track.speed_arr(2*i-1)+track.speed_arr(2*i));
        track.time_arr(i) = elapsed_time;
    end
else
    average_brake = 0.5*(upper_brake+lower_brake);
    [can_brake, track] = can_make_backward(average_brake, track, car, i);
    if ~can_brake
        %we know the correct braking amount is in the top half
        track = find_brake(average_brake, upper_brake, track, car, i);
    else
        %we know the correct braking amount is in the lower half
        track = find_brake(lower_brake, average_brake, track, car, i);
    end
end
end

function [can_brake, track] = can_make_backward(brake, track, car, i)
% given a brake force and a corner, outputs boolean can_brake saying
% whether or not the car can brake that amount for the segment and still
% make the corner
% end speed indicates speed at end of segment i. Can be kind of confusing
% as we are working backwards here

F_front = -brake * car.brake_bias;
F_rear = -brake * (1-car.brake_bias);
F_fr = F_front/2;
F_fl = F_fr;
F_rr = F_rear/2;
F_rl = F_rr;

track.brake_arr(i) = brake; %slightly inaccurate if more braking has occured than necessary

end_speed = track.speed_arr(2*i+1);
aerodrag = 0.5*car.Cd*car.A*car.rho*end_speed^2; %N
long_force = - (brake + aerodrag); %N, positive is forward
long_accel = long_force/get_mass(car); %m/s^2, positive is forward

start_speed = sqrt(end_speed^2 - 2*long_accel*track.len_arr(i)); %m/s
track.speed_arr(2*i-1) = start_speed;

% lat_accel = start_speed^2*track.inv_rad_arr(i); %m/s^2, + for RH corner
% lat_force = lat_accel*get_mass(car); %N, + for RH corner
time_elapsed = (end_speed-start_speed)/long_accel; %segment time
%==========================================================================
%yaw calcs
yaw_final = track.angular_velocity_arr(2*i); % rad/s, + is down (use right hand rule)
delta_theta = track.len_arr(i)*track.inv_rad_arr(i); %rad, + is down
% yaw_accel = 2*(yaw_final*time_elapsed-delta_theta)/(time_elapsed^2); %rad/s^2 + is down
% yaw_moment = yaw_accel*car.yaw_inertia; %Nm
if (start_speed < track.speed_arr(2*i-2))
    % reassign angular velocity if we know we will need to do more braking
    start_angular_velocity = 2*delta_theta/time_elapsed - yaw_final;
    track.angular_velocity_arr(2*i-1) = start_angular_velocity;
end

[can_brake, track] = can_make_corner(F_fr, F_fl, F_rr, F_rl, track, car, i);

end