classdef field < handle
    %Class to run voronoi and gradient based symmetric searchs
    
    properties
        sigma = .1;    % time constant for spatial separation of measurements
        tau = .8;       % time constant for temporal separation of measurements
        mu = .1;        % uncertainty in measurements, a characteristic of the sensors
        gamma = .02;    % radius over which a gradient is determined for motion
        timeToDelete = 15;
        gridSize = -1:.2:1;
        zGridSize = 0;
        runTime;        % how many seconds the Miabots will run for
        n_robots;   % number of robots
        k1 = 4;         % coefficient for forward velocity in control law
        k2 = 1;         % coefficient for angular velocity in control law
        k3 = 1;         % coefficient for z velocity in control law
        % matrix of covariances between measurements
        radius = 1;    % distance to edge of survey area
        shape = 'triangle'
        % shape of the boundary area. Currently accepted are circle,
        % square, triangle, and custom.
        
        runspeed = 'slow';
        % fast or slow, where slow follows the proper control law, and fast
        % alternates "leaders" every time step to increase speed and force
        % symmetry
        
        precision = 6; % number of spots considered for goal points
        t;              % current time
        tPast;   % previous time
        D;
        polygon;        % vertices for a custom shape
        q = 0;          % counter used in fast runspeed to determine leader
        measurements = zeros(0,4);
        robots = zeros(0,4);
        
    end
    
    methods
        
        function obj = field(n)
            % generates a new field object
            obj.n_robots = n;
            % initialize sensors
            for i=1:obj.n_robots
                obj.robots(i,:) = [0 0 0 0];
            end
            
        end
        
        function [ commands ] = control_law(obj, t, states)
            % gradient control law which views gamma-close spots to a
            % sensor and directs a Miabot to the best location
            
            obj.t = t;
            
            % initialize the sensor objects to the current positions, and
            % record the current measurements
            for i=1:obj.n_robots
                obj.robots(i,:) = [states(i,1) states(i,2) states(i,3) t];
                obj.measurements = [obj.measurements; obj.robots(i,:)];
                
            end
            
            obj.D = zeros(length(obj.measurements(:,1)) + 1, length(obj.measurements(:,1)) + 1);
            obj.D(1:end-1,1:end-1) = obj.fieldGen();
            
            % runs the 'fast' version of the control law
            if strcmp(obj.runspeed,'fast')==true
                commands = zeros(obj.n_robots,3);
                if t < .04
                    for i=1:obj.n_robots
                        commands(i,:) = [.2 0 0];
                    end
                else
                    r = mod(obj.q,obj.n_robots);
                    
                    Goals = zeros(obj.n_robots,3);
                    
                    % determines the goal point of 'leader' robot
                    GoalPoint = obj.bestDirection(obj.robots(r+1,:), states(r+1,6));
                    
                    % rotates goal point to other robots
                    for i=1:obj.n_robots
                        Goals(i,1:2) = GoalPoint(1:2)*[cos(-2*(i-r-1)*pi/(obj.n_robots)) sin(-2*(i-r-1)*pi/(obj.n_robots)); -sin(-2*(i-r-1)*pi/(obj.n_robots)) cos(-2*(i-r-1)*pi/(obj.n_robots))];
                    end
                    
                    
                    
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
                        zgoal = 0;
                        
                        
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
                        
                        u_z = (obj.k3)*(zgoal-z);
                        
                        % pass forward velocity and angular velocity to the command
                        % matrix
                        commands(i,1) = u_x;
                        commands(i,2) = u_theta;
                        commands(i,3) = u_z;
                        
                        
                    end
                end
            elseif strcmp(obj.runspeed,'average_fast')==true
                
                commands = zeros(obj.n_robots,3);
                if t < .04
                    for i=1:obj.n_robots
                        commands(i,:) = [.2 0 0];
                    end
                else
                    locations = zeros(length(obj.n_robots),3);
                    for i=1:obj.n_robots
                        locations(i,:) = [states(i,1) states(i,2) states(i,3)];
                        locations(i,1:2) = locations(i,1:2)*[cos(2*(i-1)*pi/(obj.n_robots)) sin(2*(i-1)*pi/(obj.n_robots)); -sin(2*(i-1)*pi/(obj.n_robots)) cos(2*(i-1)*pi/(obj.n_robots))];
                    end
                    location = [sum(locations(:,1))/obj.n_robots sum(locations(:,2))/obj.n_robots sum(locations(:,3))/obj.n_robots obj.t];
                    Goal = obj.bestDirection(location, states(1,6));
                    
                    Goals = zeros(obj.n_robots,3);
                    for i=1:obj.n_robots
                        Goals(i,1:2) = Goal(1:2)*[cos(-2*(i-1)*pi/(obj.n_robots)) sin(-2*(i-1)*pi/(obj.n_robots)); -sin(-2*(i-1)*pi/(obj.n_robots)) cos(-2*(i-1)*pi/(obj.n_robots))];
                        Goals(i,3) = Goal(3);
                    end
                    
                    
                    
                    
                    % use the goal points to determine commands for u_x and u_theta
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
                        
                        xgoal = Goals(i,1);
                        ygoal = Goals(i,2);
                        zgoal = 0;
                        
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
                        
                        u_z = (obj.k3)*(zgoal-z);
                        % pass forward velocity and angular velocity to the command
                        % matrix
                        commands(i,1) = u_x;
                        commands(i,2) = u_theta;
                        commands(i,3) = u_z;
                        
                    end
                end
                
            elseif strcmp(obj.runspeed,'average_slow')==true
                commands = zeros(obj.n_robots,3);
                
                if t < .04
                    for i=1:obj.n_robots
                        commands(i,:) = [.2 0 0];
                    end
                else
                    Goals = zeros(length(obj.n_robots),3);
                    parfor i=1:obj.n_robots
                        Goals(i,:) = obj.bestDirection(obj.robots(i,:), states(i,6));
                    end
                    
                    for i=1:obj.n_robots
                        Goals(i,1:2) = Goals(i,1:2)*[cos(2*(i-1)*pi/(obj.n_robots)) sin(2*(i-1)*pi/(obj.n_robots)); -sin(2*(i-1)*pi/(obj.n_robots)) cos(2*(i-1)*pi/(obj.n_robots))];
                    end
                    Goal = [sum(Goals(:,1))/obj.n_robots sum(Goals(:,2))/obj.n_robots];
                    
                    for i=1:obj.n_robots
                        Goals(i,1:2) = Goal(1:2)*[cos(-2*(i-1)*pi/(obj.n_robots)) sin(-2*(i-1)*pi/(obj.n_robots)); -sin(-2*(i-1)*pi/(obj.n_robots)) cos(-2*(i-1)*pi/(obj.n_robots))];
                    end
                    
                    
                    
                    
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
                        zgoal = 0;
                        
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
                        
                        u_z = (obj.k3)*(zgoal-z);
                        % pass forward velocity and angular velocity to the command
                        % matrix
                        commands(i,1) = u_x;
                        commands(i,2) = u_theta;
                        commands(i,3) = u_z;
                        
                    end
                end
            else
                commands = zeros(obj.n_robots,3);
                
                % calculates each robot individually, per the actual control
                % law
                parfor i=1:obj.n_robots
                    if t < .04
                        commands(i,:) = [.2 0 0];
                    else
                        Goals = obj.bestDirection(obj.robots(i,:), states(i,6));
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
                        zgoal = Goals(3);
                        
                        
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
                        
                        u_z = (obj.k3)*(zgoal-z);
                        % pass forward velocity and angular velocity to the command
                        % matrix
                        commands(i,:) = [u_x u_theta u_z];
                        
                    end
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
            
            measMax = obj.timeToDelete * obj.n_robots;
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
            densitySums = zeros(length(obj.gridSize),length(obj.robots(:,1)));
            sumsx = zeros(length(obj.gridSize), length(obj.robots(:,1)));
            sumsy = zeros(length(obj.gridSize), length(obj.robots(:,1)));
            sumsz = zeros(length(obj.gridSize), length(obj.robots(:,1)));
            sensors = obj.robots;
            
            parfor u=1:length(obj.gridSize)
                i = obj.gridSize(u);
                tempDensitySums = zeros(length(sensors(:,1)), 1);
                tempSumsx = zeros(length(sensors(:,1)), 1);
                tempSumsy = zeros(length(sensors(:,1)), 1);
                tempSumsz = zeros(length(sensors(:,1)), 1);
                
                for j=obj.gridSize
                    for z=obj.zGridSize
                        % put each point being measured into its region
                        r = ((i - sensors(1,1))^2 + ...
                            (j - sensors(1,2))^2 + ...
                            (z - sensors(1,3))^2)^.5;
                        m = 1;
                        m1 = 0;
                        n = 0;
                        
                        % designates a point with its nearest sensor
                        for k=2:length(sensors(:,1))
                            rPrime = ((i - sensors(k,1))^2 + ...
                                (j - sensors(k,2))^2 + (z - sensors(k,3))^2)^.5;
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
            
            sumx = zeros(1,length(obj.robots(:,1)));
            sumy = zeros(1,length(obj.robots(:,1)));
            sumz = zeros(1,length(obj.robots(:,1)));
            % total the individual values found inside the parfor loop
            densitySum = zeros(1,length(obj.robots(:,1)));
            
            index=1:length(obj.robots(:,1));
            sumx(index) = sum(sumsx(:,index));
            sumy(index) = sum(sumsy(:,index));
            sumz(index) = sum(sumsz(:,index));
            densitySum(index) = sum(densitySums(:,index));
            
            centroids = zeros(length(obj.robots(:,1)), 3);
            
            % calculate centroids based on a weighted center of mass
            % equation
            for i=1:length(obj.robots(:,1))
                centroids(i,1) = sumx(i) / densitySum(i);
                centroids(i,2) = sumy(i) / densitySum(i);
                centroids(i,3) = sumz(i) / densitySum(i);
            end
            
            
            
        end
        
        function [ M ] = uncertainty(obj, x, y, z, measurements)
            M = 0;
            
            
            % for speed, doesn't not compute for measurements far
            % away from each other spatially or temporally
            for i=1:length(measurements(:,1))
                if measurements(i,4) > obj.t - 3 * obj.tau
                    if abs(((measurements(i,1)).^2 ...
                            + (measurements(i,2)).^2 + (measurements(i,3)).^2).^.5 - (x.^2 + y.^2 + z.^2).^.5) ...
                            < 4 * obj.sigma
                        for j=1:length(measurements(:,1))
                            if measurements(j,4) > obj.t - 3 * obj.tau
                                if abs(((measurements(j,1)).^2 ...
                                        + (measurements(j,2)).^2 + (measurements(j,3)).^2).^.5 ...
                                        - (x.^2 + y.^2)^.5) < 4 * obj.sigma
                                    
                                    % sums all components of
                                    % certainty
                                    M = M + (exp(-abs(((sqrt((x ...
                                        - measurements(i,1)).^2 + (y ...
                                        - measurements(i,2)).^2 + (z - measurements(i,3)).^2) ...
                                        ./ obj.sigma))) - abs((obj.t - measurements(i,4))...
                                        ./ obj.tau)) .* obj.D(i,j) .* exp(-abs(((sqrt((measurements(j,1)...
                                        - x).^2 + (measurements(j,2) - y).^2 + (measurements(j,3) - z).^2)...
                                        ./ obj.sigma))) - abs(((measurements(j,4)) - obj.t)...
                                        ./ obj.tau)));
                                    
                                end
                                
                            end
                        end
                    end
                end
            end
        end
        
        function [ A ] = uncertaintyField(obj, x, y, z)
            % generates the uncertainty at a point in space at the current
            % time, used by the voronoi control law
            
            % conditions for outside sample area, currently set to an
            % equilateral triangle
            measurements = obj.measurements;
            if strcmp(obj.shape,'triangle')==true
                if x > sqrt(3)/2 || x < -sqrt(3)/2 || y > (-sqrt(3)*x + 1) || y > (sqrt(3)*x + 1) || y < -.5
                    
                    M = 1;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = 1 - M;
                
                % conditions for a square region of search
            elseif strcmp(obj.shape,'square')==true
                if x > obj.radius || x < -obj.radius || y > obj.radius || y < -obj.radius
                    
                    M = 1;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = 1 - M;
                
                % conditions for a circular region of search
            elseif strcmp(obj.shape,'circle')==true
                if (x^2 + y^2 > obj.radius^2)
                    
                    M = 1;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = 1 - M;
                
            elseif strcmp(obj.shape,'sphere')==true
                if (x^2 + y^2 + z^2 > obj.radius^2)
                    
                    M = 1;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = 1 - M;
                
                % conditions for a custom region of search
            elseif strcmp(obj.shape,'custom') == 1
                if inpolygon(x,y,obj.polygon(:,1),obj.polygon(:,2)) == 0
                    M = 1;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = 1 - M;
            end
            
        end
        
        function [ waypoints ] = voronoi_control_law(obj, t, states)
            % control law for the voronoi based control law, which is the
            % standard to which the gradient control law is compared
            
            
            obj.t = t;
            
            % sets sensors objects to their current position, and makes a
            % measurement
            for i=1:obj.n_robots
                obj.robots(i,:) = [states(i,1) states(i,2) states(i,3) t];
                obj.measurements = [obj.measurements; obj.robots(i,:)];
            end
            
            % computes the centroid of the voronoi region where each
            % sensors sits
            C = obj.centroid();
            waypoints = [C states(:,6)];
            
            obj.remove(); % removes old measurements
            
            
        end
        
        function [ F ] = bestDirection(obj, robot, theta)
            % used in the gradient control law, determines what direction
            % the robot should move in
            
            best = Inf; % tracks what direction would bring the most certainty
            
            
            % array of angles to be checked
            angle=(theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            Ftemp = zeros(length(angle), 3);
            bestTemp = zeros(length(angle), 1);
            dt = obj.t - obj.tPast; % we assume timesteps are roughly equal and use this for the future step
            
            % check each spot and determine their quality
            if strcmp(obj.runspeed,'fast') == 1 || strcmp(obj.runspeed,'average_fast') == 1
                parfor index=1:length(angle)
                    i = obj.gamma * cos(angle(index));
                    j = obj.gamma * sin(angle(index));
                    k = 0;
                    [Ftemp(index, :),bestTemp(index)] = locationTest(obj, robot, i, j, k, theta, dt);
                end
            else
                for index=1:length(angle)
                    i = obj.gamma * cos(angle(index));
                    j = obj.gamma * sin(angle(index));
                    k = 0;
                    [Ftemp(index, :),bestTemp(index)] = locationTest(obj, robot, i, j, k, theta, dt);
                end
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
                    
                    theta1 = wrapTo2Pi(atan2(Ftemp(index,2),Ftemp(index,1)) - theta);
                    
                    
                    theta2 = wrapTo2Pi(atan2(F(2)-robot(2),F(1)-robot(1)) - theta);
                    
                    if theta1 < theta2
                        F = Ftemp(index,:);
                        best = bestTemp(index);
                        
                    end
                end
            end
            
        end
        
        function [ A ] = uncertaintyCalculate(obj, x, y, z, t, tempMeas, D)
            % Calculates the net uncertainty field at a given point in
            % time, used within timeUncertaintyField
            M = 0;
            
            % compares all measurements to all other measurements
            for i=1:length(tempMeas(:,1))
                if abs(((tempMeas(i,1)).^2 + (tempMeas(i,2)).^2 ...
                        + (tempMeas(i,3)).^2).^.5 - (x.^2 + y.^2 + z.^2).^.5) ...
                        < 5 * obj.sigma
                    for j=1:length(tempMeas(:,1))
                        if abs(((tempMeas(j,1)).^2 + (tempMeas(j,2)).^2 ...
                                + (tempMeas(j,3)).^2).^.5 - (x.^2 + y.^2 + z.^2).^.5) ...
                                < 5 * obj.sigma
                            % sums all components of certainty
                            M = M + (exp(-abs(((sqrt((x ...
                                - tempMeas(i,1)).^2 + (y ...
                                - tempMeas(i,2)).^2 + (z - tempMeas(i,3)).^2) ...
                                ./ obj.sigma)))...
                                - abs((t - tempMeas(i,4))...
                                ./ obj.tau)) .* D(i,j) ...
                                .* exp(-abs(((sqrt((tempMeas(j,1)...
                                - x).^2 + (tempMeas(j,2) - y).^2 + (tempMeas(j,3) - z).^2)...
                                ./ obj.sigma))) ...
                                - abs(((tempMeas(j,4)) - t)...
                                ./ obj.tau)));
                            
                        end
                    end
                end
                
            end
            
            
            A = 1 - M;
        end
        
        function [ A ] = timeUncertaintyField(obj, x, y, z, t, tempMeas, D)
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
                        A(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
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
                        A(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
                    end
                end
                
            elseif strcmp(obj.shape,'sphere') == true
                A = zeros(1,length(x));
                
                %conditions for outside sample area
                for index=1:length(x)
                    
                    if (x(index)^2 + y(index)^2 + z(index)^2)^.5 > obj.radius
                        A(index) = 1;
                    else
                        A(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
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
                        A(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
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
                        A(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
                        
                    end
                end
            end
        end
        
        function [ k ] = lineSum(obj, robot, theta, tempMeas)
            % finds the sum of the uncertainty field over the region of
            % gamma close points to the gradient field
            
            k = 0;
            
            % completes the covariance based on the trial sensor
            D = inv(obj.finishCovariance(obj.D, tempMeas));
            
            
            angle = (theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            u = zeros(length(angle));
            
            % checks the new uncertainty at each of the possible points,
            % and returns their sum
            h = obj.timeUncertaintyField(robot(1) + v,robot(2) + w, robot(3) + u, tempMeas(end,4), tempMeas, D);
            k = k + sum(h);
            
            
        end
        
        function [ F, b ] = locationTest(obj, robot, i, j, k, theta, dt)
            % tests a point to see how much cumulative uncertainty moving
            % to it would leave behind
            
            % generates a temporary matrix including the new test
            % measurement
            tempMeas = [obj.measurements; robot(1) + i, robot(2) + j, robot(3) + k,...
                robot(4) + dt];
            
            % finds and returns the sum uncertainties at with the new
            % measurement
            b = obj.lineSum(robot, theta, tempMeas);
            F = [robot(1) + i, robot(2) + j, robot(3) + k];
            
            
            
        end
        
        function [ D ] = finishCovariance(obj, C, tempMeas)
            % calculates the covariances of the new measurement with all
            % previous ones, and combines this with the previously found
            % matrix
            
            B = zeros(length(tempMeas(:,1)), length(tempMeas(:,1)));
            i=length(tempMeas(:,1));
            for j=1:length(tempMeas(:,1))
                B(i,j) = exp(-abs(((sqrt((tempMeas(i,1) - tempMeas(j,1))^2 ...
                    + (tempMeas(i,2) - tempMeas(j,2))^2 + (tempMeas(i,3) - tempMeas(j,3))^2)/ obj.sigma))) ...
                    - abs((tempMeas(i,4) - tempMeas(j,4))/ obj.tau));
                
            end
            
            j=length(tempMeas(:,1));
            for i=1:length(tempMeas(:,1))
                B(i,j) = exp(-abs(((sqrt((tempMeas(i,1) - tempMeas(j,1))^2 ...
                    + (tempMeas(i,2) - tempMeas(j,2))^2 + (tempMeas(i,3) - tempMeas(j,3))^2)/ obj.sigma))) ...
                    - abs((tempMeas(i,4) - tempMeas(j,4))/ obj.tau));
                
            end
            
            B(end,end) = B(end,end) + obj.mu;
            D = C + B;
        end
    end
    
end

