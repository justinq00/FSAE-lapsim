%==========================================================================
%Vehicle class definition
%==========================================================================

classdef Car
    properties
        driver_skill;
        %imported values
        mass_car; %kg
        mass_driver; %kg
        prop_front; %unitless
        f_track; %Front Track Width, m
        r_track; %Rear Track Width, m
        wheelbase; %Wheelbase, m
        CGH; %CG Height, m
        yaw_inertia; %kg*m^2
    % tire data -----------------------------------------------------------
        CoF; %coefficient of friction, at 0 load
        loadS; %load sensitivity, 1/N
    % aero data -----------------------------------------------------------
        Cd; %unitless, overall
        Cl; %unitless, ovearll
        A; %m^2
        rho; %kg/m^3
        FDF; %Front Downforce Ratio
        cp_height; %m
        brake_bias; %proportion front
    %Transmission and Tire Info -------------------------------------------
        primary_drive;
        engine_sprocket_teeth;
        diff_sprocket_teeth;
        rear_tire_radius; %m
        num_gears;
        gear_ratios;
        final_drive;
        
        
        vehicle_speed; %array of possible vehicle speeds
        max_wheel_force; %array of wheel forces, function of vehicle speed
    end
    methods
        %constructor
        %takes text file of car info
        function obj = Car(filename, eng_fname)
            fid = fopen(filename);
            token = fgetl(fid);
            while ischar(token)
                if token == "" || token(1) == '%'
                    %comment or empty line in driver file
                    token = fgetl(fid);
                    continue;
                elseif token == "driver_skill"
                    token = fgetl(fid);
                    obj.driver_skill = str2double(token);
                elseif token == "mass_car"
                    token = fgetl(fid);
                    obj.mass_car = str2double(token);
                elseif token == "mass_driver"
                    token = fgetl(fid);
                    obj.mass_driver = str2double(token);
                elseif token == "prop_front"
                    token = fgetl(fid);
                    obj.prop_front = str2double(token);
                elseif token == "f_track"
                    token = fgetl(fid);
                    obj.f_track = str2double(token);
                elseif token == "r_track"
                    token = fgetl(fid);
                    obj.r_track = str2double(token);
                elseif token == "wheelbase"
                    token = fgetl(fid);
                    obj.wheelbase = str2double(token);
                elseif token == "CGH"
                    token = fgetl(fid);
                    obj.CGH = str2double(token);
                elseif token == "yaw_inertia"
                    token = fgetl(fid);
                    obj.yaw_inertia = str2double(token);
                elseif token == "CoF"
                    token = fgetl(fid);
                    obj.CoF = str2double(token);
                elseif token == "loadS"
                    token = fgetl(fid);
                    obj.loadS = str2double(token);
                elseif token == "Cd"
                    token = fgetl(fid);
                    obj.Cd = str2double(token);
                elseif token == "Cl"
                    token = fgetl(fid);
                    obj.Cl = str2double(token);
                elseif token == "A"
                    token = fgetl(fid);
                    obj.A = str2double(token);
                elseif token == "rho"
                    token = fgetl(fid);
                    obj.rho = str2double(token);
                elseif token == "FDF"
                    token = fgetl(fid);
                    obj.FDF = str2double(token);
                elseif token == "cp_height"
                    token = fgetl(fid);
                    obj.cp_height = str2double(token);
                elseif token == "brake_bias"
                    token = fgetl(fid);
                    obj.brake_bias = str2double(token);
                elseif token == "primary_drive"
                    token = fgetl(fid);
                    obj.primary_drive = str2double(token);
                elseif token == "engine_sprocket_teeth"
                    token = fgetl(fid);
                    obj.engine_sprocket_teeth = str2double(token);
                elseif token == "diff_sprocket_teeth"
                    token = fgetl(fid);
                    obj.diff_sprocket_teeth = str2double(token);
                elseif token == "rear_tire_radius"
                    token = fgetl(fid);
                    obj.rear_tire_radius = str2double(token);
                elseif token == "gear_ratios"
                    token = fgetl(fid); %num gears
                    obj.num_gears = str2double(token);
                    obj.gear_ratios = zeros(1, 20); %have room for 20 gears
                    for i = 1:obj.num_gears
                        token = fgetl(fid); %gear
                        obj.gear_ratios(i) = str2double(token);
                    end
                else
                    error('unrecognized command in car input text file');
                end
                token = fgetl(fid);
            end
            
            %==============================================================
            %engine data gathering
            %==============================================================
            [obj.vehicle_speed,obj.max_wheel_force] = calc_wheel_force(obj, eng_fname);
        end
        %Other functions:
        function [vehicle_speed, max_wheel_force] = calc_wheel_force(car1, eng_fname)
            EngineData = csvread(eng_fname); %two column csv
            %first column is Engine RPM
            %second column is Engine Torque in N*m
            Engine_RPM = EngineData(:,1); %1/min
            Torque = EngineData(:,2); %N*m
            Engine_RPM(end+1) = Engine_RPM(end)+0.1;
            Torque(end+1) = 0;
            
            %find top speed (RPM limited)
            max_wheel_rpm = Engine_RPM(end)/(car1.gear_ratios(car1.num_gears)*get_final_drive(car1));
            max_wheel_rps = max_wheel_rpm/60; %1/s
            max_speed = max_wheel_rps*pi*car1.rear_tire_radius*2; %m/s
            
            %find lowest speed
            
            %{
            min_wheel_rpm = Engine_RPM(1)/(car1.gear_ratios(1)*get_final_drive(car1));
            min_wheel_rps = min_wheel_rpm/60; %1/s
            min_speed = min_wheel_rps*pi*car1.rear_tire_radius*2; %m/s
            %}
            
            min_speed = 8;
            
            vehicle_speed = zeros(101,1); %vehicle speed vector
            max_wheel_force = zeros(1,length(vehicle_speed)); %max wheel force for each vehicle speed
            
            for i=1:101
                speed = (i-1)/100 * max_speed;
                % rpm for each gear at given speed, index corresponds to
                % gear
                RPMS = zeros(car1.num_gears, 1);
                % wheel forces for each gear at given speed, index 
                % corresponds to gear
                wheel_forces = zeros(car1.num_gears, 1);
                
                for j=1:car1.num_gears
                    RPMS(j) = speed*get_final_drive(car1)*car1.gear_ratios(j)*60/(car1.rear_tire_radius*2*pi());
                    if speed < min_speed
                        EngineTorque = 40;
                    else
                        EngineTorque = interp1(Engine_RPM,Torque,RPMS(j),'linear');
                    end
                    F = EngineTorque*get_final_drive(car1)*car1.gear_ratios(j)/car1.rear_tire_radius;
                    wheel_forces(j) = F;
                end
                [~,I] = max(wheel_forces);
                vehicle_speed(i) = speed;
                max_wheel_force(i) = wheel_forces(I);
            end
            vehicle_speed(end+1) = vehicle_speed(end)+0.1;
            max_wheel_force(end+1) = 0; %past max speed, the engine cannot produce any force
        end
        function mass = get_mass(car1)
            mass = car1.mass_car + car1.mass_driver;
        end
        function mass_front = get_mass_front(car1)
            mass_front = get_mass(car1)*car1.prop_front;
        end
        function mass_rear = get_mass_rear(car1)
            mass_rear = get_mass(car1)*(1-car1.prop_front);
        end
        function a = get_a(car1)
            %a is distance from front axle to cg
            a = get_cg_pos(car1);
        end
        function cg_pos = get_cg_pos(car1)
            % distance from front axle to cg
            cg_pos = car1.wheelbase*(1-car1.prop_front);
        end
        function aero_pos = get_aero_pos(car1)
            % distance from front axle to cp
            aero_pos = car1.wheelbase*(1-car1.FDF);
        end
        function b = get_b(car1)
            %a is distance from front axle to cg
            b = car1.wheelbase - get_cg_pos(car1);
        end
        function final_d = get_final_drive(car1)
            final_d = ...
                car1.primary_drive*car1.diff_sprocket_teeth/car1.engine_sprocket_teeth;
        end
        function min_speed = get_min_speed(car1)
            min_speed = car1.vehicle_speed(1);
        end
        function max_brake = get_max_brake_force(car1)
            max_downforce = 0.5*car1.Cl*car1.A*car1.rho*car1.vehicle_speed(end)^2;
            max_brake = (get_mass(car1)*9.81+max_downforce)*car1.CoF;
        end
    end
end