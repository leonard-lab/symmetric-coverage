classdef field < handle
    %Class to run voronoi and gradient based symmetric searchs
    
    properties
        sigma = .1;   % time constant for spatial separation of measurements
        tau = .15;     % time constant for temporal separation of measurements
        mu = .1;       % uncertainty in measurements, a characteristic of the sensors
        gamma = .06;   % radius over which a gradient is determined for motion
        measurements = zeros(0,4);
        sensors = sensor.empty(3,0);% array of sensors as they exist at this instant in time
        runTime;       % how many seconds the Miabots will run for
        n_robots = 6;  % number of robots
        k1 = 4;     % coefficient for forward velocity in control law
        k2 = 4;     % coefficient for angular velocity in control law
        k3 = 0;     % coefficient for z velocity in control law
        t;             % current time
        tPast = -.04;         % previous time
        D;          % matrix of covariances between measurements
        radius = 1;    % distance to edge of survey area
        shape = 'triangle'
        % shape of the boundary area. Currently accepted are circle,
        % square, triangle, and custom.
        
        runspeed = 'fast';
        % fast or slow, where slow follows the proper control law, and fast
        % alternates "leaders" every time step to increase speed and force
        % symmetry
        
        precision = 6; % number of spots considered for goal points
        polygon;       % vertices for a custom shape
        q = 0;         % counter used in fast runspeed to determine leader
        
        
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
            
            % initialize the sensor objects to the current positions, and
            % record the current measurements
            for i=1:obj.n_robots
                obj.sensors(i).x = states(i,1);
                obj.sensors(i).y = states(i,2);
                obj.sensors(i).z = states(i,3);
                obj.sensors(i).t = t;
                [obj.measurements] = obj.sensors(i).measure(obj.measurements);
                
            end
            
            % runs the 'fast' version of the control law
            if strcmp(obj.runspeed,'fast')==true
                r = mod(obj.q,obj.n_robots);
                
                Goals = zeros(obj.n_robots,2);
                
                % determines the goal point of 'leader' robot
                robot = [obj.sensors(r+1).x obj.sensors(r+1).y obj.sensors(r+1).z obj.sensors(r+1).t];
                GoalPoint = obj.bestDirection(robot, states(r+1,6));
                
                % rotates goal point to other robots
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
                    
                    
                end
            else
                commands = zeros(obj.n_robots,3);
                
                % calculates each robot individually, per the actual control
                % law
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
            
            measMax = 4 * obj.n_robots;
            % since measurements are stored oldest to newest, we can remove
            % the first safely
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
            
            % finds the inverse of covariance, needed for the uncertainty
            obj.D = inv(obj.fieldGen);
            
            % arrays for finding the center of mass of each region
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
                    for z=0
                        % put each point being measured into its region
                        r = ((i - sensors(1).x)^2 + ...
                            (j - sensors(1).y)^2 + ...
                            (z - sensors(1).z)^2)^.5;
                        m = 1;
                        m1 = 0;
                        n = 0;
                        
                        % designates a point with its nearest sensor
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
                        
                        % finds density of uncertainty at the given point
                        density = obj.uncertaintyField(i,j,z);
                        
                        % put a point into its proper region
                        if n == 0
                            tempDensitySums(m) = tempDensitySums(m) + density;
                            tempSumsx(m) = tempSumsx(m) + (i * density);
                            tempSumsy(m) = tempSumsy(m) + (j * density);
                            tempSumsz(m) = tempSumsz(m) + (z * density);
                            
                            % splits a point if it falls along a border
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
            
            sumx = zeros(1,length(obj.sensors));
            sumy = zeros(1,length(obj.sensors));
            sumz = zeros(1,length(obj.sensors));
            
            % total the individual values found inside the parfor loop
            densitySum = zeros(1,length(obj.sensors));
            h=1:length(obj.sensors);
            sumx(h) = sum(sumsx(:,h));
            sumy(h) = sum(sumsy(:,h));
            sumz(h) = sum(sumsz(:,h));
            densitySum(h) = sum(densitySums(:,h));
            
            centroids = zeros(length(obj.sensors), 3);
            
            % calculate centroids based on a weighted center of mass
            % equation
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
            if strcmp(obj.shape,'triangle')==true
                if x > sqrt(3)/2 || x < -sqrt(3)/2 || y > (-sqrt(3)*x + 1) || y > (sqrt(3)*x + 1) || y < -.5
                    
                    M = 1;
                else
                    M = 0;
                    
                    
                    % for speed, doesn't not compute for measurements far
                    % away from each other spatially or temporally
                    for i=1:length(obj.measurements(:,1))
                        if obj.measurements(i,4) > obj.t - 3 * obj.tau
                            if abs(((obj.measurements(i,1)).^2 ...
                                    + (obj.measurements(i,2)).^2 + (obj.measurements(i,3)).^2).^.5 - (x.^2 + y.^2 + z.^2).^.5) ...
                                    < 3 * obj.sigma
                                for j=1:length(obj.measurements(:,1))
                                    if obj.measurements(j,4) > obj.t - 3 * obj.tau
                                        if abs(((obj.measurements(j,1)).^2 ...
                                                + (obj.measurements(j,2)).^2 + (obj.measurements(j,3)).^2).^.5 ...
                                                - (x.^2 + y.^2)^.5) < 3 ...
                                                * obj.sigma
                                            
                                            % sums all components of
                                            % certainty
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
                end
                A = 1 - M;
                
                % conditions for a square region of search
            elseif strcmp(obj.shape,'square')==true
                if x > obj.radius || x < -obj.radius || y > obj.radius || y < -obj.radius
                    
                    M = 1;
                else
                    M = 0;
                    % compares all measurements to all other measurements
                    for i=1:length(obj.measurements(:,1))
                        % for speed, doesn't not compute for measurements far
                        % away from each other spatially or temporally
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
                                            % sums all components of
                                            % certainty
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
                end
                A = 1 - M;
                
                % conditions for a circular region of search
            elseif strcmp(obj.shape,'circle')==true
                if (x^2 + y^2 > obj.radius^2)
                    
                    M = 1;
                else
                    M = 0;
                    % compares all measurements to all other measurements
                    
                    for i=1:length(obj.measurements(:,1))
                        % for speed, doesn't not compute for measurements far
                        % away from each other spatially or temporally
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
                                            % sums all components of
                                            % certainty
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
                end
                A = 1 - M;
                
                % conditions for a custom region of search
            elseif strcmp(obj.shape,'custom') == 1
                if inpolygon(x,y,obj.polygon(:,1),obj.polygon(:,2)) == 0
                    M = 1;
                else
                    M = 0;
                    % compares all measurements to all other measurements
                    
                    for i=1:length(obj.measurements(:,1))
                        % for speed, doesn't not compute for measurements far
                        % away from each other spatially or temporally
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
                                            % sums all components of
                                            % certainty
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
                end
                A = 1 - M;
            end
            
        end
        
        function [ commands ] = voronoi_control_law(obj, t, states)
            % control law for the voronoi based control law, which is the
            % standard to which the gradient control law is compared
            
            
            obj.t = t;
            
            % sets sensors objects to their current position, and makes a
            % measurement
            for i=1:obj.n_robots
                obj.sensors(i).x = states(i,1);
                obj.sensors(i).y = states(i,2);
                obj.sensors(i).z = states(i,3);
                obj.sensors(i).t = t;
                obj.measurements = obj.sensors(i).measure(obj.measurements);
            end
            
            % computes the centroid of the voronoi region where each
            % sensors sits
            C = obj.centroid();
            
            commands = zeros(obj.n_robots,3);
            
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
            obj.remove(); % removes old measurements
            
            
        end
        
        function [ F ] = bestDirection(obj, robot, theta)
            % used in the gradient control law, determines what direction
            % the robot should move in
            
            best = Inf; % tracks what direction would bring the most certainty
            
            % set all of the covariance field except for the sensor being
            % guessed
            C = zeros(length(obj.measurements(:,1)) + 1, length(obj.measurements(:,1)) + 1);
            C(1:end-1,1:end-1) = obj.fieldGen();
            
            % array of angles to be checked
            angle=(theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            Ftemp = zeros(length(angle), 2);
            bestTemp = zeros(length(angle), 1);
            dt = obj.t - obj.tPast; % we assume timesteps are roughly equal and use this for the future step
            
            % check each spot and determine their quality
            parfor index=1:length(angle)
                i = obj.gamma * cos(angle(index));
                j = obj.gamma * sin(angle(index));
                [Ftemp(index, :),bestTemp(index)] = locationTest(obj, robot, i, j, theta, C, dt);
            end
            
            % pick the best direction to move in
            for index=1:length(angle)
                if bestTemp(index) < best
                    best = bestTemp(index);
                    F = Ftemp(index,:);
                    
                    % if two spots tie, pick the first going counterclockwise
                    % for the current heading
                elseif bestTemp(index) == best
                    
                    % theta1 and theta2 are the angles from the heading
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
                % conditions for outside sample area, currently set to an
                % equilateral triangle
                
                for index=1:length(x)
                    
                    if (x(index) > sqrt(3)/2) || (x(index) < -sqrt(3)/2) || (y(index) > (-sqrt(3)*x(index) + 1)) || (y(index) > (sqrt(3)*x(index) + 1)) || (y(index) < -.5)
                        
                        A(index) = 1;
                    else
                        M = 0;
                        
                        % compares all measurements to all other measurements
                        for i=1:length(tempMeas(:,1))
                            for j=1:length(tempMeas(:,1))
                                % sums all components of certainty
                                M = M + (exp(-abs(((sqrt((x(index) ...
                                    - tempMeas(i,1)).^2 + (y(index) ...
                                    - tempMeas(i,2)).^2) ...
                                    ./ obj.sigma)))...
                                    - abs((t - tempMeas(i,4))...
                                    ./ obj.tau)) .* D(i,j) ...
                                    .* exp(-abs(((sqrt((tempMeas(j,1)...
                                    - x(index)).^2 + (tempMeas(j,2) - y(index)).^2)...
                                    ./ obj.sigma))) ...
                                    - abs(((tempMeas(j,4)) - t)...
                                    ./ obj.tau)));
                                
                            end
                            
                            %  end
                            
                            %end
                        end
                        
                        
                        A(index) = 1 - M;
                    end
                end
                
                % conditions for a circular region of search
            elseif strcmp(obj.shape,'circle') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for index=1:length(x)
                    
                    if (x(index)^2 + y(index)^2)^.5 > obj.radius
                        A(index) = 1;
                    else
                        M = 0;
                        % compares all measurements to all other measurements
                        for i=1:length(tempMeas(:,1))
                            
                            for j=1:length(tempMeas(:,1))
                                
                                % sums all components of certainty
                                M = M + (exp(-abs(((sqrt((x(index) ...
                                    - tempMeas(i,1)).^2 + (y(index) ...
                                    - tempMeas(i,2)).^2) ...
                                    ./ obj.sigma)))...
                                    - abs((t - tempMeas(i,4))...
                                    ./ obj.tau)) .* D(i,j) ...
                                    .* exp(-abs(((sqrt((tempMeas(j,1)...
                                    - x(index)).^2 + (tempMeas(j,2) - y(index)).^2)...
                                    ./ obj.sigma))) ...
                                    - abs(((tempMeas(j,4)) - t)...
                                    ./ obj.tau)));
                                
                            end
                            
                        end
                        
                        
                        A(index) = 1 - M;
                    end
                end
                
                % conditions for a square region of search
            elseif strcmp(obj.shape,'square') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for index=1:length(x)
                    
                    if x(index) > obj.radius || x(index) < - obj.radius || y(index) > obj.radius || y(index) < -obj.radius
                        A(index) = 1;
                    else
                        M = 0;
                        % compares all measurements to all other measurements
                        for i=1:length(tempMeas(:,1))
                            
                            for j=1:length(tempMeas(:,1))
                                
                                % sums all components of certainty
                                M = M + (exp(-abs(((sqrt((x(index) ...
                                    - tempMeas(i,1)).^2 + (y(index) ...
                                    - tempMeas(i,2)).^2) ...
                                    ./ obj.sigma)))...
                                    - abs((t - tempMeas(i,4))...
                                    ./ obj.tau)) .* D(i,j) ...
                                    .* exp(-abs(((sqrt((tempMeas(j,1)...
                                    - x(index)).^2 + (tempMeas(j,2) - y(index)).^2)...
                                    ./ obj.sigma))) ...
                                    - abs(((tempMeas(j,4)) - t)...
                                    ./ obj.tau)));
                                
                                
                                
                            end
                        end
                        
                        
                        A(index) = 1 - M;
                    end
                end
                
                % conditions for a custom region of search
            elseif strcmp(obj.shape,'custom') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for index=1:length(x)
                    
                    if inpolygon(x(index),y(index),obj.polygon(:,1),obj.polygon(:,2)) == 0
                        A(index) = 1;
                    else
                        M = 0;
                        % compares all measurements to all other measurements
                        for i=1:length(tempMeas(:,1))
                            
                            for j=1:length(tempMeas(:,1))
                                
                                % sums all components of certainty
                                M = M + (exp(-abs(((sqrt((x(index) ...
                                    - tempMeas(i,1)).^2 + (y(index) ...
                                    - tempMeas(i,2)).^2) ...
                                    ./ obj.sigma)))...
                                    - abs((t - tempMeas(i,4))...
                                    ./ obj.tau)) .* D(i,j) ...
                                    .* exp(-abs(((sqrt((tempMeas(j,1)...
                                    - x(index)).^2 + (tempMeas(j,2) - y(index)).^2)...
                                    ./ obj.sigma))) ...
                                    - abs(((tempMeas(j,4)) - t)...
                                    ./ obj.tau)));
                                
                                
                                
                            end
                        end
                        
                        
                        A(index) = 1 - M;
                    end
                end
            end
        end
        
        function [ k ] = lineSum(obj, robot, C, theta, tempMeas)
            % finds the sum of the uncertainty field over the region of
            % gamma close points to the gradient field
            
            k = 0;
            
            % completes the covariance based on the trial sensor
            D = inv(obj.finishCovariance(C, tempMeas));
            
           
            angle = (theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            
            % checks the new uncertainty at each of the possible points,
            % and returns their sum
            h = obj.timeUncertaintyField(robot(1) + v,robot(2) + w, tempMeas(end,4), tempMeas, D);
            k = k + sum(h);
            
            
        end
        
        function [ F, b ] = locationTest(obj, robot, i, j, theta, C, dt)
            % tests a point to see how much cumulative uncertainty moving
            % to it would leave behind
            
            % generates a temporary matrix including the new test
            % measurement
            tempMeas = [obj.measurements; robot(1) + i, robot(2) + j, 0,...
                robot(4) + dt];
            
            % finds and returns the sum uncertainties at with the new
            % measurement
            b = obj.lineSum(robot, C, theta, tempMeas);
            F = [robot(1) + i, robot(2) + j];
            
            
            
        end
        
        function [ D ] = finishCovariance(obj, C, tempMeas)
            % calculates the covariances of the new measurement with all
            % previous ones, and combines this with the previously found
            % matrix
            
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

