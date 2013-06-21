classdef field < handle
    %Class to run vornoi and gradient based symmetric searchs
    
    properties
        sigma = .1;
        tau = .1;
        mu = .03;
        gamma = .1;
        meas = sensor.empty(0,0);
        sensors = sensor.empty(0,0);
        runTime;
        n_robots = 4;
        k1 = 1;
        k2 = 1;
        k3 = .1;
        t;
        D;
        a = sensor.empty(0,0);
        b = sensor.empty(0,0);
        c = sensor.empty(0,0);
        d = sensor.empty(0,0);
        q = 0;
    end
    
    methods
        
        function obj = field()
            obj.a = sensor(.1, .5, 0);
            obj.b = sensor(-.3, .3, 0);
            obj.c = sensor(.8, .7, 0);
            obj.d = sensor(.9, 0, 0);
            obj.meas = obj.a.measure(obj.meas);
            obj.meas = obj.b.measure(obj.meas);
            obj.meas = obj.c.measure(obj.meas);
            obj.meas = obj.d.measure(obj.meas);
        end
        
        function [ commands ] = control_law(obj, t, states)
            
            
            obj.t = t;
            
            obj.sensors = [obj.a; obj.b; obj.c; obj.d];

            
            %uncomment to go based on ideal position
            %obj.meas = obj.a.goToCentroid(obj.meas, C(1,:));
            %obj.meas = obj.b.goToCentroid(obj.meas, C(2,:));
            %obj.meas = obj.c.goToCentroid(obj.meas, C(3,:));
            
            
            commands = zeros(obj.n_robots,3);
            obj.q = obj.q + 1
            for i=1:obj.n_robots
                
                F = obj.bestDirection(obj.sensors(i), states(i,6));
                % Get current states of the robot, x,y,z,heading, and
                % velocities
                x = states(i,1);
                y = states(i,2);
                z = states(i,3);
                v_x = states(i,4);
                v_y = states(i,5);
                theta = states(i,6);
                theta_dot = states(i,7);
                
                
                
                
                xgoal = F(1);
                ygoal = F(2);
                
                
                % angle that the current heading is displaced from desired
                % heading
                phi = wrapToPi(atan2(ygoal-y,xgoal-x)-theta);
                
                % if statement to determine control laws for angular
                % velocity
                if (phi <= pi/2) && (phi > -pi/2)
                    u_theta = (obj.k2)*sin(phi);
                else
                    u_theta = -(obj.k2)*sin(phi);
                end
                
                r = ((xgoal-x)^2+(ygoal-y)^2)^.5; % distance to goal
                % position
                
                
                
                % control law for forward velocity
                u_x = ((obj.k1)*r*cos(phi));
                
                % pass forward velocity and angular velocity to the command
                % matrix
                commands(i,1) = u_x;
                commands(i,2) = u_theta;
                
                
                
                obj.remove();
            end
            
            %comment to go based on ideal position
            obj.a.x = states(1,1);
            obj.b.x = states(2,1);
            obj.c.x = states(3,1);
            obj.d.x = states(4,1);
            obj.a.y = states(1,2);
            obj.b.y = states(2,2);
            obj.c.y = states(3,2);
            obj.c.y = states(4,2);
            obj.a.t = t;
            obj.b.t = t;
            obj.c.t = t;
            obj.d.t = t;
            
            obj.meas = obj.a.measure(obj.meas);
            obj.meas = obj.b.measure(obj.meas);
            obj.meas = obj.c.measure(obj.meas);
            obj.meas = obj.d.measure(obj.meas);
            
        end
        
        function [] = remove(obj)
            obj.meas(1).t;
            tMin = obj.t - (2 * obj.tau);
            for i=1:length(obj.meas)
                if (obj.meas(1).t) <= tMin
                    obj.meas = obj.meas(2:length(obj.meas));
                end
            end
            
        end
        
        function [ D ] = fieldGen(obj)
            C = zeros(length(obj.meas), length(obj.meas));
            for i=1:length(obj.meas)
                for j=1:length(obj.meas)
                    C(i,j) = exp(-abs(((sqrt((obj.meas(i).x - obj.meas(j).x)^2 ...
                        + (obj.meas(i).y - obj.meas(j).y)^2)/ obj.sigma))) ...
                        - abs((obj.meas(i).t - obj.meas(j).t)/ obj.tau));
                    
                end
            end
            C = C + obj.mu * eye(length(obj.meas));
            D = inv(C);
        end
          
        function [ centroids ] = centroid( obj )
            obj.D = obj.fieldGen;
            densitySums = zeros(1,length(obj.sensors));
            sumsx = zeros(1, length(obj.sensors));
            sumsy = zeros(1, length(obj.sensors));
            for i=-1:.1:1
                for j=-1:.1:1
                    r = 1000;
                    m = 0;
                    n = 0;
                    for k=1:length(obj.sensors)
                        rPrime = ((i - obj.sensors(k).x)^2 + ...
                            (j - obj.sensors(k).y)^2)^.5;
                        if rPrime < r
                            r = rPrime;
                            m = k;
                        elseif rPrime == r
                            n = 1;
                            m1 = k;
                        end
                    end
                    density = obj.errorField(i,j);
                    
                    if n == 0
                        densitySums(m) = densitySums(m) + density;
                        sumsx(m) = sumsx(m) + (i * density);
                        sumsy(m) = sumsy(m) + (j * density);
                        
                    elseif n==1
                        densitySums(m) = densitySums(m) + density / 2;
                        densitySums(m1) = densitySums(m1) + density / 2;
                        sumsx(m) = sumsx(m) + (i/2 * density);
                        sumsy(m) = sumsy(m) + (j/2 * density);
                        sumsx(m1) = sumsx(m1) + (i/2 * density);
                        sumsy(m1) = sumsy(m1) + (j/2 * density);
                    end
                    
                    
                    
                    %field(i,j)
                end
            end
            centroids = zeros(length(obj.sensors), 2);
            for i=1:length(obj.sensors)
                centroids(i,1) = sumsx(i) / densitySums(i);
                centroids(i,2) = sumsy(i) / densitySums(i);
            end
            
            
            
        end
        
        function [ A ] = errorField(obj, x, y)
            num = size(obj.meas);
            
            M = 0;
            
            %conditions for outside sample area
            if x > 1 || x < -1 || y > 1 || y < -1
                A = 0;
            else
                
                for i=1:num(1)
                    if obj.meas(i).t > obj.t - 2 * obj.tau
                        for j=1:num(1)
                            if obj.meas(j).t > obj.t - 2 * obj.tau
                                
                                if obj.meas(i).t <= obj.t ...
                                        && obj.meas(j).t <= obj.t
                                    M = M + (exp(-abs(((sqrt((x - ...
                                    obj.meas(i).x).^2 + (y -...
                                    obj.meas(i).y).^2)/ obj.sigma)))...
                                    - abs((obj.t - obj.meas(i).t)...
                                    / obj.tau)) .* obj.D(i,j) ...
                                    .* exp(-abs(((sqrt((obj.meas(j).x...
                                    - x).^2 + (obj.meas(j).y - y).^2)...
                                    / obj.sigma))) ...
                                    - abs(((obj.meas(j).t) - obj.t)...
                                    / obj.tau)));
                                    
                                end
                            end
                        end
                    end
                end
                
                A = 1 - M;
            end
        end
        
        function [ vertices, cells ] = voronoi(obj)
            X = zeros(length(obj.sensors), 2);
            for i=1:length(obj.sensors)
                X(i,:) = [obj.sensors(i).x, obj.sensors(i).y];
            end
            [vertices, cells] = voronoin(X);
            voronoi(X(:,1), X(:,2));
        end
        
        
        function [ commands ] = voronoi_control_law(obj, t, states)
            
            
            obj.t = t;
            
            obj.sensors = [obj.a; obj.b; obj.c; obj.d];
            %obj.meas = a.moveSensor(obj.meas);
            %obj.meas = b.moveSensor(obj.meas);
            %obj.meas = c.addField(obj.meas);
            C = obj.centroid();
            
            %uncomment to go based on ideal position
            %obj.meas = obj.a.goToCentroid(obj.meas, C(1,:));
            %obj.meas = obj.b.goToCentroid(obj.meas, C(2,:));
            %obj.meas = obj.c.goToCentroid(obj.meas, C(3,:));
            
            
            commands = zeros(obj.n_robots,3);
            obj.q = obj.q + 1
            for i=1:obj.n_robots
                
                % Get current states of the robot, x,y,z,heading, and
                % velocities
                x = states(i,1);
                y = states(i,2);
                z = states(i,3);
                v_x = states(i,4);
                v_y = states(i,5);
                theta = states(i,6);
                theta_dot = states(i,7);
                
                
                
                
                xgoal = C(i,1);
                ygoal = C(i,2);
                
                
                % angle that the current heading is displaced from desired
                % heading
                phi = wrapToPi(atan2(ygoal-y,xgoal-x)-theta);
                
                % if statement to determine control laws for angular
                % velocity
                if (phi <= pi/2) && (phi > -pi/2)
                    u_theta = (obj.k2)*sin(phi);
                else
                    u_theta = -(obj.k2)*sin(phi);
                end
                
                r = ((xgoal-x)^2+(ygoal-y)^2)^.5; % distance to goal
                % position
                
                
                
                % control law for forward velocity
                u_x = ((obj.k1)*r*cos(phi));
                
                % pass forward velocity and angular velocity to the command
                % matrix
                commands(i,1) = u_x;
                commands(i,2) = u_theta;
                
                
                
                obj.remove();
            end
            
            %comment to go based on ideal position
            obj.a.x = states(1,1);
            obj.b.x = states(2,1);
            obj.c.x = states(3,1);
            obj.d.x = states(4,1);
            obj.a.y = states(1,2);
            obj.b.y = states(2,2);
            obj.c.y = states(3,2);
            obj.c.y = states(4,2);
            obj.a.t = t;
            obj.b.t = t;
            obj.c.t = t;
            obj.d.t = t;
            
            obj.meas = obj.a.measure(obj.meas);
            obj.meas = obj.b.measure(obj.meas);
            obj.meas = obj.c.measure(obj.meas);
            obj.meas = obj.d.measure(obj.meas);
            
        end
        
        function [ F ] = bestDirection(obj, sensor, theta)
            b = Inf;
            F = [0,0];
            for i=-obj.gamma:.01:obj.gamma
                for j=-sqrt((obj.gamma)^2 - i^2),sqrt((obj.gamma)^2 - i^2)
                    x = sensor.x + i;
                    y = sensor.y + j;
                    temp = sensor(x, y, sensor.t + .04);
                    obj.meas = [obj.meas; temp];
                    fun = @(x,y) obj.errorField(x,y);
                    k = integral2(fun, -1, 1, -1, 1);
                    if k < b
                        F = [x,y];
                        b = k;
                    elseif k == b
                        theta1 = wrapTo2Pi(atan2(x,y) - theta);
                        theta2 = wrapTo2Pi(atan2(F(1),F(2)) - theta);
                        if theta1 < theta2
                            F = [x,y];
                        end
                    end
                    
                    
                    obj.meas = obj.meas(1:length(obj.meas)-1);
                end
            end
        end
        
    end
    
    
end

