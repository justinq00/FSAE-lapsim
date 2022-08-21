function [points, sum_pts] = dynamic_pts(car)

% Michigan 2019
% best_times = [4.1,4.865,48.8,1268];
% Michigan 2019 only Umich, Polymtl
best_times = [4.1,5.358,53.501,1327.128];
endurance_fuel_used = 5;
CO2_min = 6.62;
points = zeros(5, 1); 

% accel--------------------------------------------------------------------

accel = Track('accel.csv', 1, 1);
accel.speed_arr(1) = 0.1;
[Tyour, ~] = solve_track(accel, car);

%disp(Tyour);

Tmin = min(Tyour, best_times(1));
Tmax = Tmin*1.5;

if (Tyour > Tmax)
    accel_score = 4.5;
else
    accel_score = 95.5*(Tmax/Tyour-1)/(Tmax/Tmin-1)+4.5;
end

points(1) = accel_score;

%disp(Tyour);

% skidpad------------------------------------------------------------------

old_inertia = car.yaw_inertia;
car.yaw_inertia = 0; % angular velocity oscillates like crazy

skidpad = Track('skidpad.csv', 1, 1);
skidpad.speed_arr(1) = 0.1;
[~, skidpad] = solve_track(skidpad, car);

final_array = skidpad.time_arr(543:1083);

Tyour = sum(final_array);

%disp(Tyour);

Tmin = min(Tyour, best_times(2));
Tmax = Tmin*1.25;


if (Tyour > Tmax)
    skidpad_score = 3.5;
else
    skidpad_score = 71.5*((Tmax/Tyour)^2-1)/((Tmax/Tmin)^2-1)+3.5;
end

points(2) = skidpad_score;

car.yaw_inertia = old_inertia;

% #3
% autocross----------------------------------------------------------------

car.driver_skill = 0.95;

autocross = Track('2018MichiganAXTrack_new.csv', 10, 1);
autocross.speed_arr(1) = 0.1;
[Tyour, ~] = solve_track(autocross, car);

%disp(Tyour);

Tmin = min(Tyour, best_times(3));
Tmax = Tmin*1.45;

if (Tyour > Tmax)
    autox_score = 6.5;
else
    autox_score = 118.5*(Tmax/Tyour-1)/(Tmax/Tmin-1)+6.5;
end

points(3) = autox_score;

% endurance----------------------------------------------------------------

car.driver_skill = 0.95;

endurance = Track('2019MichiganEnduranceTrack.csv', 10, 1);
endurance.speed_arr(1) = 8;
[lap_time, ~] = solve_track(endurance, car);

%disp(lap_time);

Tyour = lap_time*11;

%disp(Tyour);

Tmin = min(Tyour, best_times(4));
Tmax = Tmin*1.45;

if (Tyour > Tmax)
    endurance_score = 25;
else
    endurance_score = 250*(Tmax/Tyour-1)/(Tmax/Tmin-1)+25;
end

points(4) = endurance_score;

% efficiency --------------------------------------------------------------

Tmin = best_times(4);
LapTotal_tmin = 11;
LapTotal_CO2min = 11;

CO2_your = endurance_fuel_used*2.31; % kg
Lapyours = 11;
Tyours = Lapyours*lap_time;

E_F = (Tmin/LapTotal_tmin)/(Tyours/Lapyours)*...
    (CO2_min/LapTotal_CO2min)/(CO2_your/Lapyours);

laps_min = 11;
E_F_min = (Tmin/LapTotal_tmin)/(1.45*Tmin/laps_min)*...
    (CO2_min/LapTotal_CO2min)/(60.06*0.22/laps_min);

E_F_max = max(E_F, 0.989);

if (CO2_your/22 > 60.06*0.22)||(Tyour>1.45*Tmin)
    efficiency_score = 0;
else
    efficiency_score = 100* (E_F_min/E_F - 1)/...
        (E_F_min/E_F_max - 1);
end
points(5) = efficiency_score;

% -------------------------------------------------------------------------

sum_pts = sum(points);
disp(sum_pts);
end