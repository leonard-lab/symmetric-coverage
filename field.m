classdef field < handle
    %Class to run voronoi and gradient based symmetric searchs
    
    properties
        sigma = .14;   % time constant for spatial separation of measurements
        tau = .08;     % time constant for temporal separation of measurements
        mu = .1;       % uncertainty in measurements, a characteristic of the sensors
        gamma = .04;   % radius over which a gradient is determined for motion
        meas = sensor.empty(15,0);  % array of sensor object, which contains all relevant past measurements
        measurements = zeros(0,4);
        sensors = sensor.empty(4,0);% array of sensors as they exist at this instant in time
        runTime;       % how many seconds the Miabots will run for
        n_robots = 3;  % number of robots
        k1 = 1;     % coefficient for forward velocity in control law
        k2 = 2;     % coefficient for angular velocity in control law
        k3 = 0;        % coefficient for forward velocity in integral control law
        k4 = 0;
        t;             % current time
        D;
        
        
        
      
        q = 0;
        
        
    end
    
    methods
        
        function obj = field()
            % generates a new field object
            
            % initialize sensors
            robot1 = sensor(sqrt(3)/20, -.05, 0, 0);
            robot2 = sensor(-sqrt(3)/20, -.05, 0, 0);
            robot3 = sensor(0, .1, 0, 0);
            %obj.d = sensor(-sqrt(3)/20, .05, 0, 0);
            %obj.e = sensor(sqrt(3)/20, .05, 0, 0);
            %obj.f = sensor(0, -.1, 0, 0);
            %obj.a = sensor(0, .2, 0);
            %obj.b = sensor(0, -.2,0);
            
            % take initial sensor measurements
            %obj.meas = robot1.measure(obj.meas);
            %obj.meas = robot2.measure(obj.meas);
            %obj.meas = robot3.measure(obj.meas);
            %obj.meas = obj.d.measure(obj.meas);
            %obj.meas = obj.e.measure(obj.meas);
            %obj.meas = obj.f.measure(obj.meas);
            %obj.meas = obj.d.measure(obj.meas);
            obj.sensors = [robot1; robot2; robot3];% obj.d; obj.e; obj.f];
        end
        
        function [ commands ] = control_law(obj, t, states)
            % gradient control law which views gamma-close spots to a
            % sensor and directs a Miabot to the best location
             obj.t = t;
             for i=1:obj.n_robots

                 [obj.meas, obj.measurements] = obj.sensors(i).measure(obj.meas, obj.measurements);
             end
             
            
            r = mod(obj.q,3);
            rot1 = [cos(4*pi/3) sin(4*pi/3); -sin(4*pi/3) cos(4*pi/3)];
            rot2 = [cos(2*pi/3) sin(2*pi/3); -sin(2*pi/3) cos(2*pi/3)];
            if r == 0
                GoalPoint = obj.bestDirection(obj.sensors(1), states(1,6));
                %H = obj.bestDirection(obj.sensors(4), states(4,6));
                F = [GoalPoint; GoalPoint*rot1; GoalPoint*rot2];% H; H*rot1; H*rot2];
            elseif r == 1
                GoalPoint = obj.bestDirection(obj.sensors(2), states(2,6));
                %H = obj.bestDirection(obj.sensors(5), states(5,6));
                
                F = [GoalPoint*rot2; GoalPoint; GoalPoint*rot1]; %H*rot2; H; H*rot1];
            elseif r == 2
                GoalPoint = obj.bestDirection(obj.sensors(3), states(3,6));
                % H = obj.bestDirection(obj.sensors(6), states(6,6));
                F = [GoalPoint*rot1; GoalPoint*rot2; GoalPoint];% H*rot1; H*rot2; H];
            end
            
            
            
            commands = zeros(obj.n_robots,3);
            obj.q = obj.q + 1;
            
            % use the goal points to determine commands for u_x and u_theta
            for i=1:obj.n_robots
                
                %F = obj.bestDirection(obj.sensors(i), states(i,6));
                % Get current states of the robot, x,y,z,heading, and
                % velocities
                
                x = states(i,1);
                y = states(i,2);
                z = states(i,3);
                v_x = states(i,4);
                v_y = states(i,5);
                theta = states(i,6);
                theta_dot = states(i,7);
                
                xgoal = F(i,1);
                ygoal = F(i,2);
                
                
                % angle that the current heading is displaced from desired
                % heading
                phi = wrapToPi(atan2(ygoal-y,xgoal-x)-theta);
                
                % if statement to determine control laws for angular
                % velocity
                if (phi <= pi/2) && (phi >= -pi/2)
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
              
                %comment to go based on ideal position

                
            end
            %commands
            
            obj.remove();
            
            
            
            for i=1:obj.n_robots
                 obj.sensors(i).x = states(i,1);
                 obj.sensors(i).y = states(i,2);
                 obj.sensors(i).z = states(i,3);
                 obj.sensors(i).t = t;
            end
            
            
        end
        
        function [] = remove(obj)
            % removes measurements from the meas array when they are no
            % longer relevant
            
            %tMin = obj.t - 2*(obj.tau);
            %for i=1:length(obj.meas)
            %    if (obj.meas(1).t) <= tMin
            %        obj.meas = obj.meas(2:length(obj.meas));
            %    end
            %end
            
            measMax = 3 * obj.n_robots;
            while length(obj.measurements(:,1)) > measMax
                obj.meas = obj.meas(2:length(obj.meas));
                obj.measurements = obj.measurements((2:length(obj.measurements(:,1))),:);
            end
            
        end
        
        function [ D ] = fieldGen(obj)
            % generates the covariance between two measurement points, and
            % returns a matrix of it
            C = zeros(length(obj.measurements(:,1)), length(obj.measurements(:,1)));
            for i=1:length(obj.measurements(:,1))
                for j=1:length(obj.measurements(:,1))
                    
                    C(i,j) = exp(-abs(((sqrt((obj.measurements(i,1) - obj.measurements(j,1))^2 ...
                        + (obj.measurements(i,2) - obj.measurements(j,2))^2 + (obj.measurements(i,3) - obj.measurements(j,3))^2)/ obj.sigma))) ...
                        - abs((obj.measurements(i,4) - obj.measurements(j,4))/ obj.tau));

                    
                end
            end
            D = C + obj.mu * eye(length(obj.measurements(:,1)));

        end
        
        function [ centroids ] = centroid( obj )
            % finds the centroid of each voronoi region surrounding a
            % sensor weighted by uncertainty
            
            obj.D = inv(obj.fieldGen);
            densitySums = zeros(11,length(obj.sensors));
            sumsx = zeros(11, length(obj.sensors));
            sumsy = zeros(11, length(obj.sensors));
            sumsz = zeros(11, length(obj.sensors));
            sensors = obj.sensors;
            
            parfor u=1:11
                i = (.2 * (u-6));
                tempDensitySums = zeros(length(sensors), 1);
                tempSumsx = zeros(length(sensors), 1);
                tempSumsy = zeros(length(sensors), 1);
                tempSumsz = zeros(length(sensors), 1);
                
                for j=-1:.2:1
                    for z=-1:.2:1
                        r = ((i - sensors(1).x)^2 + ...
                            (j - sensors(1).y)^2 + ...
                            (z - sensors(1).z)^2)^.5;
                        m = 1;
                        m1 = 0;
                        n = 0;
                        for k=2:length(sensors)
                            rPrime = ((i - sensors(k).x)^2 + ...
                                (j - sensors(k).y)^2 + (z - sensors(k).z)^2)^.5;
                            if rPrime < r
                                r = rPrime;
                                m = k;
                            elseif rPrime == r
                                n = 1;
                                m1 = k;
                            end
                        end
                        density = obj.errorField(i,j,z);
                        
                        if n == 0
                            tempDensitySums(m) = tempDensitySums(m) + density;
                            tempSumsx(m) = tempSumsx(m) + (i * density);
                            tempSumsy(m) = tempSumsy(m) + (j * density);
                            tempSumsz(m) = tempSumsz(m) + (z * density);
                            
                        elseif n==1
                            tempDensitySums(m) = tempDensitySums(m) + density / 2;
                            tempDensitySums(m1) = tempDensitySums(m1) + density / 2;
                            tempSumsx(m) = tempSumsx(m) + (i/2 * density);
                            tempSumsy(m) = tempSumsy(m) + (j/2 * density);
                            tempSumsz(m) = tempSumsz(m) + (z/2 * density);
                            tempSumsx(m1) = tempSumsx(m1) + (i/2 * density);
                            tempSumsy(m1) = tempSumsy(m1) + (j/2 * density);
                            tempSumsz(m1) = tempSumsz(m1) + (z/2 * density);
                        end
                        
                        
                    end
                end
                densitySums(u,:) = tempDensitySums;
                sumsx(u,:) = tempSumsx;
                sumsy(u,:) = tempSumsy;
                sumsz(u,:) = tempSumsz;
            end
            %densitySums
            sumx = zeros(1,length(obj.sensors));
            sumy = zeros(1,length(obj.sensors));
            sumz = zeros(1,length(obj.sensors));
            
            densitySum = zeros(1,length(obj.sensors));
            h=1:length(obj.sensors);
            sumx(h) = sum(sumsx(:,h));
            sumy(h) = sum(sumsy(:,h));
            sumz(h) = sum(sumsz(:,h));
            densitySum(h) = sum(densitySums(:,h));
            
            centroids = zeros(length(obj.sensors), 3);
            for i=1:length(obj.sensors)
                centroids(i,1) = sumx(i) / densitySum(i);
                centroids(i,2) = sumy(i) / densitySum(i);
                centroids(i,3) = sumz(i) / densitySum(i);
            end
            
            
            
        end
        
        function [ A ] = errorField(obj, x, y, z)
            % generates the uncertainty at a point in space at the current
            % time, used by the voronoi control law
            M = 0;
            
            % conditions for outside sample area, currently set to an
            % equilateral triangle
            if x > sqrt(3)/2 || x < -sqrt(3)/2 || y > (-sqrt(3)*x + 1) || y > (sqrt(3)*x + 1) || y < -.5
                %if x > 1 || x < -1 || y > 1 || y < -1
                A = 0;
            else
                
                for i=1:length(obj.measurements(:,1))
                    if obj.measurements(i,4) > obj.t - 2 * obj.tau
                        if abs(((obj.measurements(i,1)).^2 ...
                                + (obj.measurements(i,2)).^2 + (obj.measurements(i,3)).^2).^.5 - (x.^2 + y.^2 + z.^2).^.5) ...
                                < 3 * obj.sigma
                            for j=1:length(obj.measurements(:,1))
                                if obj.measurements(j,4) > obj.t - 2 * obj.tau
                                    if abs(((obj.measurements(j,1)).^2 ...
                                            + (obj.measurements(j,2)).^2 + (obj.measurements(j,3)).^2).^.5 ...
                                            - (x.^2 + y.^2)^.5) < 3 ...
                                            * obj.sigma
                                        M = M + (exp(-abs(((sqrt((x ...
                                            - obj.measurements(i,1)).^2 + (y ...
                                            - obj.measurements(i,2)).^2 + (z - obj.measurements(i,3)).^2) ...
                                            ./ obj.sigma)))...
                                            - abs((obj.t - obj.measurements(i,4))...
                                            ./ obj.tau)) .* obj.D(i,j) ... % MAKE D(i,j,k)
                                            .* exp(-abs(((sqrt((obj.measurements(j,1)...
                                            - x).^2 + (obj.measurements(j,2) - y).^2 + (obj.measurements(j,3) - z).^2)...
                                            ./ obj.sigma))) ...
                                            - abs(((obj.measurements(j,4)) - obj.t)...
                                            ./ obj.tau)));
                                        
                                    else
                                        M = M + 0;
                                    end
                                else
                                    M = M + 0;
                                end
                            end
                        else
                            M = M + 0;
                        end
                    else
                        M = M + 0;
                    end
                end
                
                
                A = 1 - M;
            end
            
        end
        
        function [ commands ] = voronoi_control_law(obj, t, states)
            % control law for the voronoi based control law, which is the
            % standard to which the gradient control law is compared
            
            
            obj.t = t;
            %comment to go based on ideal position
            for i=1:obj.n_robots
                obj.sensors(i).x = states(i,1);
                obj.sensors(i).y = states(i,2);
                obj.sensors(i).z = states(i,3);
                obj.sensors(i).t = t;
                [obj.meas, obj.measurements] = obj.sensors(i).measure(obj.meas, obj.measurements);
            end
            
            
            C = obj.centroid();
            
            %uncomment to go based on ideal position
            %obj.meas = obj.a.goToCentroid(obj.meas, C(1,:));
            %obj.meas = obj.b.goToCentroid(obj.meas, C(2,:));
            %obj.meas = obj.c.goToCentroid(obj.meas, C(3,:));
            
            
            commands = zeros(obj.n_robots,3);
            %obj.q = obj.q + 1
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
                zgoal = C(i,3);
                
                
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
                
                u_z = obj.k4*(z-zgoal);
                % pass forward velocity and angular velocity to the command
                % matrix
                commands(i,1) = u_x;
                commands(i,2) = u_theta;
                commands(i,3) = u_z;
                
                
                
            end
            obj.remove();

            
        end
        
        function [ F ] = bestDirection(obj, sensor1, theta)
            % used in the gradient control law, determines what direction
            % the robot should move in
            b = Inf;
            F = [0,0];
            C = zeros(length(obj.measurements(:,1)) + 1, length(obj.measurements(:,1)) + 1);
            C(1:end-1,1:end-1) = obj.fieldGen;
            angle=(theta+pi/3):pi/3:(2*pi+theta);
            Ftemp = zeros(length(angle), 2);
            btemp = zeros(length(angle), 1);
            parfor index=1:length(angle)
                i = obj.gamma * cos(angle(index));
                j = obj.gamma * sin(angle(index));
                [Ftemp(index, :),btemp(index)] = locationTest(obj, sensor1, i, j, theta, F, C);
            end
            
            for index=1:length(angle)
                if btemp(index) < b
                    b = btemp(index);
                    F = Ftemp(index,:);
                elseif btemp(index) == b
                    'no'
                    if Ftemp(index,1) == 0
                        theta1 = wrapTo2Pi(pi/2 - theta);
                    else
                        theta1 = wrapTo2Pi(atan2(Ftemp(index,2),Ftemp(index,1)) - theta);
                    end
                    
                    if (F(1)-sensor1.x) == 0
                        theta2 = wrapTo2Pi(pi/2 - theta);
                    else
                        theta2 = wrapTo2Pi(atan2(F(2)-sensor1.y,F(1)-sensor1.x) - theta);
                    end
                    if theta1 < theta2
                        F = Ftemp(index,:);
                        b = btemp(index);
                        'yes'
                    end
                end
            end
            
        end
        
        function [ A ] = timeErrorField(obj, x, y, t, tempMeas, D)
            % generates the uncertainty at a given place in space and time,
            % used in the gradient control law
            
            A = zeros(1,length(x));
            
            %conditions for outside sample area
            for h=1:length(x)
                
                if (x(h) > sqrt(3)/2) || (x(h) < -sqrt(3)/2) || (y(h) > (-sqrt(3)*x(h) + 1)) || (y(h) > (sqrt(3)*x(h) + 1)) || (y(h) < -.5)
                    %if (x(h)^2 + y(h)^2)^.5 > .75
                    A(h) = 1;
                else
                    M = 0;
                    for i=1:length(tempMeas(:,1))
                        if abs(((tempMeas(i,1)).^2 ...
                                + (tempMeas(i,2))^2).^.5 - (x(h).^2 + y(h).^2).^.5) ...
                                < obj.sigma
                            for j=1:length(tempMeas(:,1))
                                if abs(((tempMeas(j,1)).^2 ...
                                        + (tempMeas(j,2)).^2).^.5 ...
                                        - (x(h).^2 + y(h).^2).^.5) < ...
                                        obj.sigma
                                    M = M + (exp(-abs(((sqrt((x(h) ...
                                        - tempMeas(i,1)).^2 + (y(h) ...
                                        - tempMeas(i,2)).^2) ...
                                        ./ obj.sigma)))...
                                        - abs((t - tempMeas(i,4))...
                                        ./ obj.tau)) .* D(i,j) ...
                                        .* exp(-abs(((sqrt((tempMeas(j,1)...
                                        - x(h)).^2 + (tempMeas(j,2) - y(h)).^2)...
                                        ./ obj.sigma))) ...
                                        - abs(((tempMeas(j,4)) - t)...
                                        ./ obj.tau)));
                                    
                                end
                                
                            end
                            
                        end
                    end
                    
                    
                    A(h) = 1 - M;
                end
            end
            
        end
        
        function [] = start(obj, runtime, theta)
            % testing client used to compare results to an ideal without
            % the constraints of robot motion
            obj.t = 0;
            A = zeros(floor(runtime/.04),2);
            B = zeros(floor(runtime/.04),2);
            C = zeros(floor(runtime/.04),2);
            %D = zeros(floor(runtime/.04),2);
            while obj.t < runtime
                obj.sensors = [obj.a; obj.b; obj.c; obj.d];
                A(floor(obj.t/.04) + 1,1) = obj.a.x;
                A(floor(obj.t/.04) + 1,2) = obj.a.y;
                B(floor(obj.t/.04) + 1,1) = obj.b.x;
                B(floor(obj.t/.04) + 1,2) = obj.b.y;
                C(floor(obj.t/.04) + 1,1) = obj.c.x;
                C(floor(obj.t/.04) + 1,2) = obj.c.y;
                %D(floor(obj.t/.04) + 1,1) = obj.d.x;
                %D(floor(obj.t/.04) + 1,2) = obj.d.y;
                rot1 = [cos(-2*pi/3) sin(-2*pi/3); -sin(-2*pi/3) cos(-2*pi/3)];
                rot2 = [cos(2*pi/3) sin(2*pi/3); -sin(2*pi/3) cos(2*pi/3)];
                F = obj.bestDirection(obj.sensors(1), theta(1));
                G = F*rot1;
                H = F * rot2;
                obj.meas = obj.a.goToCentroid(obj.meas, F);
                obj.meas = obj.b.goToCentroid(obj.meas, G);
                obj.meas = obj.c.goToCentroid(obj.meas, H);
                %obj.meas = obj.d.goToCentroid(obj.meas, obj.bestDirection(obj.sensors(4), theta(4)));
                obj.meas = obj.a.measure(obj.meas);
                obj.meas = obj.b.measure(obj.meas);
                obj.meas = obj.c.measure(obj.meas);
                %obj.meas = obj.d.measure(obj.meas);
                obj.t = obj.t + .04
                obj.remove();
                
            end
            figure
            plot(A(:,1), A(:,2), B(:,1), B(:,2), C(:,1), C(:,2));
            line([-.866 0], [-.5 1]);
            line([.866 0], [-.5 1]);
            line([-.866 .866], [-.5 -.5]);
            %legend('Robot 1', 'Robot 2');
            axis([-.9 .9 -.5 1]);
            length(A(:,1))
            figure
            plot(A(:,1), .04:.04:3);
        end
        
        function [ k ] = lineSum(obj, sensor1, C, theta, tempMeas)
            % finds the sum of the uncertainty field over the region of
            % gamma close points to the gradient field
            k = 0;
            
            D = inv(obj.finishField(C, tempMeas));
       
            
            angle = (theta+pi/3):pi/3:(2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            
            
            h = obj.timeErrorField(sensor1.x + v,sensor1.y + w, tempMeas(end,4), tempMeas, D);
            k = k + sum(h);
            
            
        end
        
        function [ F, b ] = locationTest(obj, sensor1, i, j, theta, F, C)
            
            
            tempMeas = [obj.measurements; sensor1.x + i, sensor1.y + j, 0, obj.t + .04];
                        
            b = obj.lineSum(sensor1, C, theta, tempMeas);
            F = [sensor1.x + i, sensor1.y + j];
            
                       
            
        end
        
        function [ D ] = finishField(obj, C, tempMeas)
            
            B = zeros(length(tempMeas(:,1)), length(tempMeas(:,1)));
            i=length(tempMeas(:,1));
            for j=1:length(tempMeas(:,1))
                B(i,j) = exp(-abs(((sqrt((tempMeas(i,1) - tempMeas(j,1))^2 ...
                    + (tempMeas(i,2) - tempMeas(j,2))^2)/ obj.sigma))) ...
                    - abs((tempMeas(i,4) - tempMeas(j,4))/ obj.tau));
                
            end
            
            j=length(tempMeas(:,1));
            for i=1:length(tempMeas(:,1))
                B(i,j) = exp(-abs(((sqrt((tempMeas(i,1) - tempMeas(j,1))^2 ...
                    + (tempMeas(i,2) - tempMeas(j,2))^2)/ obj.sigma))) ...
                    - abs((tempMeas(i,4) - tempMeas(j,4))/ obj.tau));
                
            end
            B(end,end) = B(end,end) + obj.mu;
            D = C + B;
        end
    end
    
end

