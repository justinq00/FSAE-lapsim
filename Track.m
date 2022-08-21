%==========================================================================
%Tracj class definition
%==========================================================================

classdef Track
    properties
        num_seg; % number of segments
        len_arr; % array storing length of each segment
        inv_rad_arr; %array storing inv_rad of each segment
        throttle_arr;
        brake_arr;
        lat_g_arr;
        long_g_arr;
        time_arr; % array storing time of each segment
        
        % double length arrays --------------------------------------------
        speed_arr; % double length
        angular_velocity_arr; % double length

        distance_double;
        % wheel load vectors
        max_fr;
        max_fl;
        max_rr;
        max_rl;
        
        f_fr;
        f_fl;
        f_rr;
        f_rl;
        
        % raw input vectors:
        distance_0;
        inv_rad_arr_0;
    end
    methods
        function obj = Track(filename, segment_length, num_smooth)
            % segment_length is how many original segments will make up a 
            % new segment. Increase this for faster run time, although
            % potentially less accurate
            % num_smooth is how many times the original inv_rad data is
            % smooth. Suggest keeping this low (1 or 0)
            TrackData = csvread(filename);
            %{
            track data in rows format:
            first row is distance from start
            second row is inverse corner radius
            %}
            obj.distance_0 = TrackData(:,1); %m
            obj.inv_rad_arr_0 = TrackData(:,2); %1/m
            inv_rad_arr = obj.inv_rad_arr_0;
            %==============================================================
            for i = 1:num_smooth
                inv_rad_arr = smoothdata(inv_rad_arr);
            end
            %==============================================================
            obj.num_seg = fix(size(inv_rad_arr, 1)/segment_length)-1;
            obj.len_arr = zeros(obj.num_seg,1);
            obj.inv_rad_arr = zeros(obj.num_seg,1);
            for i = 1:obj.num_seg
                obj.len_arr(i) = ...
                    obj.distance_0(segment_length*i+1) - obj.distance_0(segment_length*(i-1)+1);
                inv_rad = 0;
                for j = 1:segment_length
                    seg_inv_rad = inv_rad_arr(segment_length*(i-1)+1+j);
                    seg_end_dist = obj.distance_0(segment_length*(i-1)+1+j);
                    seg_start_dist = obj.distance_0(segment_length*(i-1)+j);
                    seg_length = seg_end_dist-seg_start_dist;
                    inv_rad = inv_rad + seg_inv_rad*(seg_length/obj.len_arr(i));
                end
                obj.inv_rad_arr(i) = inv_rad;
            end
            %initiaize arrays to correct size
            obj.speed_arr = zeros(obj.num_seg*2, 1);
            obj.angular_velocity_arr = zeros(obj.num_seg*2, 1);
            obj.throttle_arr = zeros(obj.num_seg, 1);
            obj.brake_arr = zeros(obj.num_seg, 1);
            obj.time_arr = zeros(obj.num_seg, 1);
            obj.lat_g_arr = zeros(obj.num_seg, 1);
            obj.long_g_arr = zeros(obj.num_seg, 1);
            
            obj.max_fr = zeros(obj.num_seg, 1);
            obj.max_fl = zeros(obj.num_seg, 1);
            obj.max_rr = zeros(obj.num_seg, 1);
            obj.max_rl = zeros(obj.num_seg, 1);
            
            %first row is long force, second is lat
            obj.f_fr = zeros(obj.num_seg, 2);
            obj.f_fl = zeros(obj.num_seg, 2);
            obj.f_rr = zeros(obj.num_seg, 2);
            obj.f_fl = zeros(obj.num_seg, 2);
            
            %==============================================================
            % create distance_double array, where each end/start distance 
            % is replicated twice for purpose of graphing with speed and
            % angular velocity arrays
            obj.distance_double = zeros(obj.num_seg*2, 1);
            obj.distance_double(2) = obj.len_arr(1);
            
            for i = 2:obj.num_seg
                %j = fix(i/2)+1;
                obj.distance_double(2*i-1) = obj.distance_double(2*i-2);
                obj.distance_double(2*i) = ...
                    obj.distance_double(2*i-1)+obj.len_arr(i);
            end
        end
        function speed_vs_distance(track1)
            figure('Name','Speed vs Distance');
            plot(track1.distance_double, track1.speed_arr);
            xlabel('distance (m)');
            ylabel('speed (m/s)');
        end
        function angular_velocity_vs_distance(track1)
            figure('Name','Angular Velocity vs Distance');
            plot(track1.distance_double, track1.angular_velocity_arr);
            xlabel('distance (m)');
            ylabel('angular velocity (rad/s)');
        end
        function inv_rad_vs_distance(track1)
            y = zeros(track1.num_seg*2, 1);
            for i = 1:track1.num_seg
                y(2*i-1) = track1.inv_rad_arr(i);
                y(2*i) = track1.inv_rad_arr(i);
            end
            figure('Name','inv rad vs Distance');
            xlabel('distance (m)');
            ylabel('inv rad (g)');
            plot(track1.distance_double, y, 'r', ...
                track1.distance_0, track1.inv_rad_arr_0, 'b');
        end
        function long_g_vs_distance(track1)
            y = zeros(track1.num_seg*2, 1);
            for i = 1:track1.num_seg
                y(2*i-1) = track1.long_g_arr(i);
                y(2*i) = track1.long_g_arr(i);
            end
            figure('Name','longG vs Distance');
            plot(track1.distance_double, y);
            xlabel('distance (m)');
            ylabel('long G (g)');
        end
        function lat_g_vs_distance(track1)
            y = zeros(track1.num_seg*2, 1);
            for i = 1:track1.num_seg
                y(2*i-1) = track1.lat_g_arr(i);
                y(2*i) = track1.lat_g_arr(i);
            end
            figure('Name','latG vs Distance');
            plot(track1.distance_double, y);
            xlabel('distance (m)');
            ylabel('lat G (g)');
        end
        function brake_vs_distance(track1)
            y = zeros(track1.num_seg*2, 1);
            for i = 1:track1.num_seg
                y(2*i-1) = track1.brake_arr(i);
                y(2*i) = track1.brake_arr(i);
            end
            figure('Name','Brake vs Distance');
            plot(track1.distance_double, y);
            xlabel('distance (m)');
            ylabel('brake (N)');
        end
        function throttle_vs_distance(track1)
            y = zeros(track1.num_seg*2, 1);
            for i = 1:track1.num_seg
                y(2*i-1) = track1.throttle_arr(i);
                y(2*i) = track1.throttle_arr(i);
            end
            figure('Name','Throttle vs Distance');
            plot(track1.distance_double, y);
            xlabel('distance (m)');
            ylabel('throttle');
        end
        function animate(track1)
            laptime = sum(track1.time_arr);
            if (track1.num_seg < 1) || (laptime < 0.1)
                error('track too short or time not calculated');
            end
            % time from start of lap
            elapsed_time_arr = zeros(track1.num_seg, 1);
            elapsed_time_arr(1) = track1.time_arr(1);
            for i = 2:track1.num_seg
                elapsed_time_arr(i) = elapsed_time_arr(i-1) + track1.time_arr(i);
            end
            
            freq = 15; %frames per second
            scale_factor = 100;
            
            %define axes
            axis(gca, 'equal');
            axis([-20 20 -20 20]);
            
            FR = [10 10];
            FL = [-10 10];
            RR = [10 -10];
            RL = [-10 -10];
            
            % track segment
            i = 1;
            for f = 1:laptime*freq % f is frame
                if (f/freq > elapsed_time_arr(i))
                    i = i+1;
                end
                
                FR_force = line([FR(1) FR(1)+(track1.f_fr(i,2)/scale_factor)], ...
                    [FR(2) FR(2)+(track1.f_fr(i,1)/scale_factor)]);
                FL_force = line([FL(1) FL(1)+(track1.f_fl(i,2)/scale_factor)], ...
                    [FL(2) FL(2)+(track1.f_fl(i,1)/scale_factor)]);
                RR_force = line([RR(1) RR(1)+(track1.f_rr(i,2)/scale_factor)], ...
                    [RR(2) RR(2)+(track1.f_rr(i,1)/scale_factor)]);
                RL_force = line([RL(1) RL(1)+(track1.f_rl(i,2)/scale_factor)], ...
                    [RL(2) RL(2)+(track1.f_rl(i,1)/scale_factor)]);
                
                pause(1/freq);
                delete(FR_force);
                delete(FL_force);
                delete(RR_force);
                delete(RL_force);
            end
        end
    end
end