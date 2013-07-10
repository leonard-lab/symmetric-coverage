classdef field < handle
    %Class to run voronoi and gradient based symmetric searchs
    
    properties
        sigma = .3;   % time constant for spatial separation of measurements
        tau = .3;     % time constant for temporal separation of measurements
        mu = .1;       % uncertainty in measurements, a characteristic of the sensors
        gamma = .1;   % radius over which a gradient is determined for motion
        measurements = zeros(0,4);
        sensors = sensor.empty(3,0);% array of sensors as they exist at this instant in time
        runTime;       % how many seconds the Miabots will run for
        n_robots = 3;  % number of robots
        k1 = 6;     % coefficient for forward velocity in control law
        k2 = 6;     % coefficient for angular velocity in control law
        k3 = 0;     % coefficient for z velocity in control law
        t;             % current time
        tPast = 0;         % previous time
        D;
        radius = 1;    % distance to edge of survey area
        shape = 'triangle'
        runspeed = 'fast';
        precision = 6;
        polygon;
        
        
        
        q = 0;
        
        
    end
    
    methods
        
        function obj = field()
            % generates a new field object
            
            % initialize sensors
            for i=1:obj.n_robots
                obj.sensors(i) = sensor(0, 0, 0, 0);
            end
            
        end
        
        function [ commands ] = control_law(obj, t, states)
            % gradient control law which views gamma-close spots to a
            % sensor and directs a Miabot to the best location
            obj.t = t;
            for i=1:obj.n_robots
                obj.sensors(i).x = states(i,1);
                obj.sensors(i).y = states(i,2);
                obj.sensors(i).z = states(i,3);
                obj.sensors(i).t = t;
                [obj.measurements] = obj.sensors(i).measure(obj.measurements);
                
            end
            
            if strcmp(obj.runspeed,'fast')==true
                r = mod(obj.q,obj.n_robots);
                
                Goals = zeros(obj.n_robots,2);
                robot = [obj.sensors(r+1).x obj.sensors(r+1).y obj.sensors(r+1).z obj.sensors(r+1).t];
                
                GoalPoint = obj.bestDirection(robot, states(r+1,6));
                for i=1:obj.n_robots
                    Goals(i,:) = GoalPoint*[cos(2*(i-r-1)*pi/(obj.n_robots)) sin(2*(i-r-1)*pi/(obj.n_robots)); -sin(2*(i-r-1)*pi/(obj.n_robots)) cos(2*(i-r-1)*pi/(obj.n_robots))];
                end
                
                
                commands = zeros(obj.n_robots,3);
                obj.q = obj.q + 1;
                
                % use the goal points to determine commands for u_x and u_theta
                for i=1:obj.n_robots
                    
                    % F = obj.bestDirection(obj.sensors(i), states(i,6));
                    % Get current states of the robot, x,y,z,heading, and
                    % velocities
                    
                    x = states(i,1);
                    y = states(i,2);
                    z = states(i,3);
                    v_x = states(i,4);
                    v_y = states(i,5);
                    theta = states(i,6);
                    theta_dot = states(i,7);
                    
                    xgoal = Goals(i,1);
                    ygoal = Goals(i,2);
                    
                    
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
            else
                commands = zeros(obj.n_robots,3);

                
                % use the goal points to determine commands for u_x and u_theta
                for i=1:obj.n_robots
                    robot = [obj.sensors(i).x obj.sensors(i).y obj.sensors(i).z obj.sensors(i).t];
                    Goals = obj.bestDirection(robot, states(i,6));
                    % Get current states of the robot, x,y,z,heading, and
                    % velocities
                    
                    x = states(i,1);
                    y = states(i,2);
                    z = states(i,3);
                    v_x = states(i,4);
                    v_y = states(i,5);
                    theta = states(i,6);
                    theta_dot = states(i,7);
                    
                    xgoal = Goals(1);
                    ygoal = Goals(2);
                    
                    
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
                    
                end
            end
            
            obj.remove();
            %states
            %commands
            obj.tPast = obj.t;
            
            
        end
        
        function [] = remove(obj)
            % removes measurements from the meas array when they are no
            % longer relevant
            
            measMax = 6 * obj.n_robots;
            while length(obj.measurements(:,1)) > measMax
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
            
            % adds the uncertainty of the sensors to their variance
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
                        density = obj.uncertaintyField(i,j,z);
                        
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
        
        function [ A ] = uncertaintyField(obj, x, y, z)
            % generates the uncertainty at a point in space at the current
            % time, used by the voronoi control law

            % conditions for outside sample area, currently set to an
            % equilateral triangle
            if x > sqrt(3)/2 || x < -sqrt(3)/2 || y > (-sqrt(3)*x + 1) || y > (sqrt(3)*x + 1) || y < -.5
                %if x > 1 || x < -1 || y > 1 || y < -1
                A = 0;
            else
                M = 0;
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
                                            ./ obj.tau)) .* obj.D(i,j) ... 
                                            .* exp(-abs(((sqrt((obj.measurements(j,1)...
                                            - x).^2 + (obj.measurements(j,2) - y).^2 + (obj.measurements(j,3) - z).^2)...
                                            ./ obj.sigma))) ...
                                            - abs(((obj.measurements(j,4)) - obj.t)...
                                            ./ obj.tau)));                       
                                    
                                    end
                                
                                end
                            end                      
                        end 
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
                obj.measurements = obj.sensors(i).measure(obj.measurements);
            end
            
            
            C = obj.centroid();
            
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
                
                u_z = obj.k3*(z-zgoal);
                % pass forward velocity and angular velocity to the command
                % matrix
                commands(i,1) = u_x;
                commands(i,2) = u_theta;
                commands(i,3) = u_z;
                
                
                
            end
            obj.remove();
            
            
        end
        
        function [ F ] = bestDirection(obj, robot, theta)
            % used in the gradient control law, determines what direction
            % the robot should move in
            best = Inf;
            
            C = zeros(length(obj.measurements(:,1)) + 1, length(obj.measurements(:,1)) + 1);
            C(1:end-1,1:end-1) = obj.fieldGen;
            angle=(theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            Ftemp = zeros(length(angle), 2);
            bestTemp = zeros(length(angle), 1);
            dt = obj.t - obj.tPast;
            parfor index=1:length(angle)
                i = obj.gamma * cos(angle(index));
                j = obj.gamma * sin(angle(index));
                [Ftemp(index, :),bestTemp(index)] = locationTest(obj, robot, i, j, theta, C, dt);
            end
            
            for index=1:length(angle)
                if bestTemp(index) < best
                    best = bestTemp(index);
                    F = Ftemp(index,:);
                elseif bestTemp(index) == best
                    
                    if Ftemp(index,1) == 0
                        theta1 = wrapTo2Pi(pi/2 - theta);
                    else
                        theta1 = wrapTo2Pi(atan2(Ftemp(index,2),Ftemp(index,1)) - theta);
                    end
                    
                    if (F(1)-robot(1)) == 0
                        theta2 = wrapTo2Pi(pi/2 - theta);
                    else
                        theta2 = wrapTo2Pi(atan2(F(2)-robot(2),F(1)-robot(1)) - theta);
                    end
                    if theta1 < theta2
                        F = Ftemp(index,:);
                        best = bestTemp(index);
                        
                    end
                end
            end
            
        end
        
        function [ A ] = timeUncertaintyField(obj, x, y, t, tempMeas, D)
            % generates the uncertainty at a given place in space and time,
            % used in the gradient control law
            if strcmp(obj.shape,'triangle')==true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for h=1:length(x)
                    
                    if (x(h) > sqrt(3)/2) || (x(h) < -sqrt(3)/2) || (y(h) > (-sqrt(3)*x(h) + 1)) || (y(h) > (sqrt(3)*x(h) + 1)) || (y(h) < -.5)
                        %if (x(h)^2 + y(h)^2)^.5 > .75
                        A(h) = 1;
                    else
                        M = 0;
                        for i=1:length(tempMeas(:,1))
                            %if abs(((tempMeas(i,1)).^2 ...
                            %        + (tempMeas(i,2))^2).^.5 - (x(h).^2 + y(h).^2).^.5) ...
                            %        < 3 * obj.sigma
                                for j=1:length(tempMeas(:,1))
                                    
                             %       if abs(((tempMeas(j,1)).^2 ...
                             %               + (tempMeas(j,2)).^2).^.5 ...
                             %               - (x(h).^2 + y(h).^2).^.5) < ...
                             %               3 * obj.sigma
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
                                    
                              %  end
                                
                            %end
                        end
                        
                        
                        A(h) = 1 - M;
                    end
                end
            elseif strcmp(obj.shape,'circle') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for h=1:length(x)
                    
                    if (x(h)^2 + y(h)^2)^.5 > obj.radius
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
            elseif strcmp(obj.shape,'square') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for h=1:length(x)
                    
                    if x(h) > obj.radius || x(h) < - obj.radius || y(h) > obj.radius || y(h) < -obj.radius
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
            elseif strcmp(obj.shape,'custom') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for h=1:length(x)
                    
                    if inpolygon(x(h),y(h),obj.polygon(:,1),obj.polygon(:,2)) == 0
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
        end
        
        function [ k ] = lineSum(obj, robot, C, theta, tempMeas)
            % finds the sum of the uncertainty field over the region of
            % gamma close points to the gradient field
            k = 0;
            
            D = inv(obj.finishField(C, tempMeas));
            
            
            angle = (theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            
            
            h = obj.timeUncertaintyField(robot(1) + v,robot(2) + w, tempMeas(end,4), tempMeas, D);
            k = k + sum(h);
            
            
        end
        
        function [ F, b ] = locationTest(obj, robot, i, j, theta, C, dt)
            
            
            tempMeas = [obj.measurements; robot(1) + i, robot(2) + j, 0, robot(4) + dt];
            
            b = obj.lineSum(robot, C, theta, tempMeas);
            F = [robot(1) + i, robot(2) + j];
            
            
            
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

