classdef field < handle
    % This class runs voronoi and gradient based searches through the
    % Miabots class. The user must provide a runtime, a shape of the area
    % to be searched, and initial conditions.
    %
    % Summary provided below. More help can be found by typing doc field
    % or help field.method_name.
    %
    % SYNTAX
    %
    % S = field(length(init(:,1)), shape, radius);
    %
    % INPUTS
    % init: n_robots X [x y z theta] matrix containing the initial
    % positions and headings of the robots.
    %
    % radius: the distance from the origin of the area being surveyed,
    % radius of the circle or sphere, distance to midpoints of lines on
    % square, and distance to vertices of triangle
    %
    % shape: 'circle', 'triangle', 'square', 'sphere', or 'custom', these
    % determine the area being surveyed. NOTE: at present custom only
    % supports 2 dimensional areas
    %
    % run_time: Time in seconds to run the system or simulation. Provide
    % Inf to run system indefinitely (run mode only).
    %
    % 'sim': Logical. Default: false. true to simulate dynamics in MATLAB.
    % false to run ROS.
    %
    % 'sim_noise': Length 4 Vector. Default: [0 0 0 0]. Standard deviation
    % of the random gaussian noise applied to [x y z theta] measuremente
    % estimates during simulation.
    %
    % PROPERTIES
    %
    %    sigma: constant for spatial separation of measurements
    %
    %    tau: time constant for temporal separation of measurements
    %
    %    mu: uncertainty in measurements, a characteristic of the sensors
    %
    %    gamma: radius over which a gradient is determined for motion
    %
    %    timeToDelete: the number of time steps robot positions are saved for
    %
    %    gridSize: the grid size used for the voronoi control law
    %
    %    zGridSize: the z grid size used for the voronoi control law
    %
    %    runTime: how many seconds the Miabots will run for
    %
    %    n_robots: number of robots
    %
    %    k1: coefficient for forward velocity in control law
    %
    %    k2: coefficient for angular velocity in control law
    %
    %    k3: coefficient for z velocity in control law
    %
    %    radius = 1;        % distance to edge of survey area
    %
    %    shape: shape of the boundary area. Currently accepted are circle,
    %    square, triangle, and custom.
    %
    %    runspeed: fast, average_fast, average_slow, precise_slow, or slow,
    %    where slow follows the proper control law, fast
    %    alternates "leaders" every time step to increase speed and force
    %    symmetry, average_fast averages positions to find where to go,
    %    average_slow averages goals, and precise_slow does the proper
    %    control law, but runs faster than slow with more possible points
    %
    %    precision: number of spots considered for goal points
    %
    %    t: current time
    %
    %    tPast: previous time
    %
    %    D: matrix of covariances between measurements
    %
    %    polygon: vertices for a custom shape
    %
    %    measurements: retains matrix of past positions
    %
    %    robots: stores current positions of robots
    %
    %    origin: center of the survey area, which is treated as the origin
    %
    %    timeCounter: counter used in fast runspeed to determine leader
    %
    %    measureCounter: counter used in overwriting old measurements
    %
    %    firstStepTime: time for which the robots move staight along heading
    %
    %    firstStepSpeed: speed that the robots move at for the first step
    %
    % METHODS
    %
    %   field: generates a new field object
    %
    %   control_law: gradient control law which views gamma-close spots
    %   to a sensor and directs a Miabot to the best location, according
    %   to the law written in the symmetric coverage paper
    %
    %   fieldGen: generates the covariance between two
    %   measurement points, and returns a matrix of all these covariances
    %
    %   bestDirection: used in the gradient control
    %   law, determines what direction the robot should move in, and
    %   returns its corrdinates
    %
    %   uncertaintyCalculate: Calculates the net uncertainty field at a
    %   given point in time, used within timeUncertaintyField
    %
    %   timeUncertaintyField: generates the uncertainty at a given place
    %   in space and time, used in the gradient control law
    %
    %   locationTest: tests a point to see how much cumulative uncertainty
    %   moving to it would leave behind on the other gamma close points
    %
    %   finishCovariance: calculates the covariances of the new measurement
    %   with all previous ones, and combines this with the previously found
    %   matrix
    %
    %   compileLocationTest: This method allows for more straightfoward
    %   running of the program if multiple robots are being calculated
    %   simultaneously outside of a parfor loop
    %
    %   commandGen: generates commands for the Miabots based on their goal
    %   points and the coefficients for the control law
    %
    %   determineEntropy: determines the information entropy of the area
    %   being searched at a given time
    %
    %   centroid: finds the centroid of each voronoi region surrounding a
    %   sensor, weighted by uncertainty
    %
    %   certainty: Calculates the net certainty at a given point
    %   in time, used within uncertaintyField
    %
    %   uncertaintyField: generates the uncertainty at a point in space at
    %   the current time, used by the voronoi control law
    %
    %   voronoi_control_law: control law for the voronoi based control law,
    %   which is the standard to which the gradient control law is compared
    %
    %   twoRobotsCircle: sets the properties used for the demo of two
    %   robots in a circular survey area
    %
    %   twoRobotsSquare: sets the properties used for the demo of two
    %   robots in a square survey area
    
    properties
        sigma = .2;        % time constant for spatial separation of measurements
        tau = .8;          % time constant for temporal separation of measurements
        mu = .1;           % uncertainty in measurements, a characteristic of the sensors
        gamma = .08;       % radius over which a gradient is determined for motion
        spacetimeAverage = 1;  % coefficient for field covariance
        timeToDelete = 3;  % the number of time steps robot positions are saved for
        gridSize = -1:.2:1;% the grid size used for the voronoi control law
        zGridSize = 0;     % the z grid size used for the voronoi control law
        runTime;           % how many seconds the Miabots will run for
        n_robots;          % number of robots
        k1 = 1;            % coefficient for forward velocity in control law
        k2 = 1;            % coefficient for angular velocity in control law
        k3 = 1;            % coefficient for z velocity in control law
        radius = 1;        % distance to edge of survey area
        firstStepTime = .15    % time for which the robots move staight along heading
        firstStepSpeed = .2    % speed that the robots move at for the first step
        
        shape = 'triangle'
        % shape of the boundary area. Currently accepted are circle,
        % square, triangle, and custom.
        
        runspeed = 'slow'; % fast, average_fast, average_slow, precise_slow, or slow,...
        % where slow follows the proper control law, fast
        % alternates "leaders" every time step to increase speed and force
        % symmetry, average_fast averages positions to find where to go,
        % average_slow averages goals, and precise_slow does the proper
        % control law, but runs faster than slow with more possible points
        precision = 6;        % number of spots considered for goal points
        D;                    % matrix of covariances between measurements
        polygon;              % vertices for a custom shape
        origin = [0 -0.5 0];  % center of the survey area, which is treated as the origin
    end
    
    properties (SetAccess = private)
        t;                         % current time
        tPast = -.04;              % previous time
        measurements = zeros(0,4); % retains matrix of past positions
        robots = zeros(0,4);       % stores current positions of robots
    end
    
    properties (GetAccess = private)
        timeCounter = 0;   % counter used in fast runspeed to determine leader
        measureCounter = 0;% counter used in overwriting old measurements
    end
    
    methods
        
        function obj = field(n, shape, radius)
            % FIELD generates a new field object
            %
            % SYNOPSIS obj = field(n, shape, radius)
            %
            % INPUT n: the number of robots
            % shape: the shape of the area being surveyed, 'square',
            % 'circle', 'sphere', 'triangle', or 'custom
            % radius: the scaling factor of the area being surveyed
            
            obj.n_robots = n;
            
            % initialize sensors
            for i=1:obj.n_robots
                obj.robots(i,:) = [0 0 0 0];
            end
            obj.radius = radius;
            obj.shape = shape;
            
            % set the polygon that describes the shape, it it is a preset
            if strcmp(obj.shape,'triangle') == 1
                obj.polygon = obj.radius .* [sqrt(3)/2 -.5; -sqrt(3)/2 -.5; 0 1];
            elseif strcmp(obj.shape,'square') == 1
                obj.polygon = [obj.radius obj.radius; obj.radius -obj.radius;...
                    -obj.radius -obj.radius; -obj.radius obj.radius];
            elseif strcmp(obj.shape, 'circle') == 1
                angle=0:0.01:2*pi;
                x=obj.radius*cos(angle);
                y=obj.radius*sin(angle);
                
                obj.polygon = [transpose(x) transpose(y)];
            end
        end
        
        function [] = twoRobotsCircle(obj)
            % TWOROBOTSCIRCLE sets the properties used for the demo of two
            % robots in a circular survey area
            
            obj.sigma = .1;
            obj.tau = 1;
            obj.mu = .15;
            obj.gamma = .1;
            obj.timeToDelete = 4;
            obj.k1 = 1;
            obj.k2 = 1;
            obj.k3 = 1;
            obj.origin = [0 -.50 0];
        end
        
        function [] = twoRobotsSquare(obj)
            % TWOROBOTSSQUARE sets the properties used for the demo of two
            % robots in a square survey area
            
            obj.sigma = .2;
            obj.tau = 8;
            obj.mu = .1;
            obj.gamma = .08;
            obj.timeToDelete = 2;
            obj.k1 = 1;
            obj.k2 = 1;
            obj.k3 = 1;
            obj.origin = [0 -.50 0];
        end
    end
    
    methods
        % gradient control law methods
        
        function [ commands ] = control_law(obj, t, states)
            % CONTROL_LAW gradient control law which views gamma-close spots
            % to a sensor and directs a Miabot to the best location,
            % according to the law written in the symmetric coverage paper
            %
            % SYNOPSIS [ commands ] = control_law(obj, t, states)
            %
            % INPUT obj: the object
            % t: the current time
            % states: the current n_robots X 7 matrix of the robots' states
            %
            % OUTPUT commands: n_robots X 3 matrix of commands for the
            % robots
            
            obj.t = t;
            
            % initialize the sensor objects to the current positions, and
            % record the current measurements
            for i=1:obj.n_robots
                obj.robots(i,:) = [states(i,1)-obj.origin(1) ...
                    states(i,2)-obj.origin(2) ...
                    states(i,3)-obj.origin(3) t];
                
                obj.measurements(mod(obj.measureCounter,obj.n_robots*obj.timeToDelete)+1,:) = obj.robots(i,:);
                obj.measureCounter = obj.measureCounter+1;
            end
            
            % find the covariance between the measurements, and leave a
            % space for testing the movements for the next time step
            obj.D = zeros(length(obj.measurements(:,1)) + 1,...
                length(obj.measurements(:,1)) + 1);
            obj.D(1:end-1,1:end-1) = obj.fieldGen(obj.measurements);
            
            % runs the 'fast' version of the control law
            if strcmp(obj.runspeed,'fast')==true
                commands = zeros(obj.n_robots,3);
                if t < obj.firstStepTime
                    for i=1:obj.n_robots
                        commands(i,:) = [obj.firstStepSpeed 0 0];
                    end
                else
                    % determine the index of the robot whose turn it is to
                    % lead
                    r = mod(obj.timeCounter,obj.n_robots);
                    
                    Goals = zeros(obj.n_robots,3);
                    
                    % determines the goal point of 'leader' robot
                    GoalPoint = obj.bestDirection(obj.robots(r+1,:),...
                        states(r+1,6));
                    
                    % rotates goal point to other robots
                    for i=1:obj.n_robots
                        Goals(i,1:2)...
                            = GoalPoint(1:2)*[cos(-2*(i-r-1)...
                            *pi/(obj.n_robots)) sin(-2*(i-r-1)...
                            *pi/(obj.n_robots)); -sin(-2*(i-r-1)...
                            *pi/(obj.n_robots)) cos(-2*(i-r-1)...
                            *pi/(obj.n_robots))];
                    end
                    
                    commands = obj.commandGen(states, Goals);
                    
                end
            elseif strcmp(obj.runspeed,'average_fast')==true
                % TO FIX: which we rotate to affects answer
                commands = zeros(obj.n_robots,3);
                if t < obj.firstStepTime
                    for i=1:obj.n_robots
                        commands(i,:) = [obj.firstStepSpeed 0 0];
                    end
                else
                    % rotate states to match each other
                    locations = zeros(length(obj.n_robots),3);
                    for i=1:obj.n_robots
                        locations(i,:) = [states(i,1) states(i,2) states(i,3)];
                        locations(i,1:2) = locations(i,1:2)*[cos(2*(i-1)...
                            *pi/(obj.n_robots)) sin(2*(i-1)...
                            *pi/(obj.n_robots)); -sin(2*(i-1)...
                            *pi/(obj.n_robots)) cos(2*(i-1)...
                            *pi/(obj.n_robots))];
                    end
                    location = [sum(locations(:,1))/obj.n_robots ...
                        sum(locations(:,2))/obj.n_robots ...
                        sum(locations(:,3))/obj.n_robots obj.t];
                    
                    % calculate goal from the average of states
                    Goal = obj.bestDirection(location, states(1,6));
                    
                    Goals = zeros(obj.n_robots,3);
                    % rotate goal to all robots
                    for i=1:obj.n_robots
                        Goals(i,1:2) = Goal(1:2)*[cos(-2*(i-1)...
                            *pi/(obj.n_robots)) sin(-2*(i-1)...
                            *pi/(obj.n_robots)); -sin(-2*(i-1)...
                            *pi/(obj.n_robots)) cos(-2*(i-1)...
                            *pi/(obj.n_robots))];
                        Goals(i,3) = Goal(3);
                    end
                    
                    commands = obj.commandGen(states, Goals);
                end
                
            elseif strcmp(obj.runspeed,'average_slow')==true
                % calculates each robot's goal, then averages
                commands = zeros(obj.n_robots,3);
                
                if t < obj.firstStepTime
                    for i=1:obj.n_robots
                        commands(i,:) = [obj.firstStepSpeed 0 0];
                    end
                else
                    Goals = zeros(length(obj.n_robots),3);
                    % calculate robots' goals
                    parfor i=1:obj.n_robots
                        Goals(i,:) = obj.bestDirection(obj.robots(i,:), states(i,6));
                    end
                    
                    % rotate goals to match
                    for i=1:obj.n_robots
                        Goals(i,1:2) = Goals(i,1:2)*[cos(2*(i-1)...
                            *pi/(obj.n_robots)) sin(2*(i-1)...
                            *pi/(obj.n_robots)); -sin(2*(i-1)...
                            *pi/(obj.n_robots)) cos(2*(i-1)...
                            *pi/(obj.n_robots))];
                    end
                    
                    % average goals
                    Goal = [sum(Goals(:,1))/obj.n_robots ...
                        sum(Goals(:,2))/obj.n_robots];
                    % rotate goals to each robot
                    for i=1:obj.n_robots
                        Goals(i,1:2) = Goal(1:2)*[cos(-2*(i-1)...
                            *pi/(obj.n_robots)) sin(-2*(i-1)...
                            *pi/(obj.n_robots)); -sin(-2*(i-1)...
                            *pi/(obj.n_robots)) cos(-2*(i-1)...
                            *pi/(obj.n_robots))];
                    end
                    commands = obj.commandGen(states, Goals);
                    
                end
            elseif strcmp(obj.runspeed,'precise_slow') == 1
                % calculates each robot individually, per the actual control
                % law
                Goals = obj.bestDirection(obj.robots, states(:,6));
                commands = obj.commandGen(states, Goals);
            else
                % calculates each robot individually, per the actual control
                % law
                parfor i=1:obj.n_robots
                    Goals(i,:) = obj.bestDirection(obj.robots(i,:), states(i,6));
                end
                commands = obj.commandGen(states, Goals);
            end
            
            obj.timeCounter = obj.timeCounter + 1;
            obj.tPast = obj.t;
            
        end
        
        function [ covariances ] = fieldGen(obj, measurements)
            % FIELDGEN generates the covariance between two measurement
            % points, and returns a matrix of all these covariances
            %
            % SYNOPSIS [ covariances ] = fieldGen(obj, measurements)
            %
            % INPUT obj: the object
            % measurements: the matrix of past and current robot positions
            %
            % OUTPUT covariances: the matrix of covariances between
            % measurements
            
            C = zeros(length(measurements(:,1)), length(measurements(:,1)));
            for i=1:length(measurements(:,1))
                for j=1:length(measurements(:,1))
                    % equation for covariance, number 4 in the paper
                    C(i,j) = exp(-abs(((sqrt((measurements(i,1) ...
                        - measurements(j,1))^2 + (measurements(i,2)...
                        - measurements(j,2))^2 + (measurements(i,3)...
                        - measurements(j,3))^2)/ obj.sigma))) ...
                        - abs((measurements(i,4) - measurements(j,4))/ obj.tau));
                    
                end
            end
            
            % adds the uncertainty of the sensors to their variance
            covariances = C + obj.mu * eye(length(measurements(:,1)));
            
        end
        
        function [ goal ] = bestDirection(obj, robots, theta)
            % BESTDIRECTION used in the gradient control law, determines
            % what direction the robot should move in, and returns its
            % corrdinates
            %
            % SYNOPSIS [ goal ] = bestDirection(obj, robots, theta)
            %
            % INPUT obj: the object
            % robots: the matrix of current robot positions
            % theta: the matrix of current robot headings
            %
            % OUTPUT goal: the objective (x,y,z) for each robot
            
            best = Inf(length(robots(:,1)),1); % tracks what direction would bring the most certainty
            
            goal = zeros(length(robots(:,1)),3);
            % array of angles to be checked
            
            goaltemp = zeros(length(robots(:,1)), 3, obj.precision);
            bestTemp = zeros(length(robots(:,1)),obj.precision);
            dt = obj.t - obj.tPast; % we assume timesteps are roughly equal and use this for the future step
            
            % check each spot and determine their quality, using different
            % methods depending on the controller being used
            if  strcmp(obj.runspeed,'precise_slow') == 1
                
                parfor index=1:obj.precision
                    [goaltemp(:,:,index), bestTemp(:,index)] = obj.compileLocationTest(robots, theta, index, dt);
                    
                end
                
            elseif strcmp(obj.runspeed,'fast') == 1 || strcmp(obj.runspeed,'average_fast') == 1
                angle=theta+pi/(.5*obj.precision):pi/(.5*obj.precision):(2*pi+theta);
                parfor index=1:length(angle)
                    i = obj.gamma * cos(angle(index));
                    j = obj.gamma * sin(angle(index));
                    k = 0;
                    
                    [goaltemp(:,:,index),bestTemp(:,index)] = obj.locationTest(robots, i, j, k, theta, dt);
                    
                end
            else
                angle=theta+pi/(.5*obj.precision):pi/(.5*obj.precision):(2*pi+theta);
                for index=1:length(angle)
                    i = obj.gamma * cos(angle(index));
                    j = obj.gamma * sin(angle(index));
                    k = 0;
                    for u=1:length(robots(:,1))
                        [goaltemp(u,:,index),bestTemp(u,index)] = obj.locationTest(robots(u,:), i, j, k, theta, dt);
                    end
                end
            end
            
            
            % pick the best direction to move in
            for i=1:length(robots(:,1))
                for index=1:obj.precision
                    
                    if bestTemp(i,index) < best(i)
                        best(i) = bestTemp(i,index);
                        goal(i,:) = goaltemp(i,:,index);
                        
                        % if two spots tie, pick the first going counterclockwise
                        % for the current heading
                    elseif bestTemp(i,index) == best(i)
                        
                        % theta1 and theta2 are the angles from the heading
                        theta1 = wrapTo2Pi(atan2(goaltemp(i,2,index),...
                            goaltemp(i,1,index)) - theta(i));
                        theta2 = wrapTo2Pi(atan2(goal(i,2)-robots(i,2),...
                            goal(i,1)-robots(i,1)) - theta(i));
                        % choose the angle that comes first moving
                        % counterclockwise
                        if theta1 < theta2
                            goal(i,:) = goaltemp(i,:,index);
                            best(i) = bestTemp(i,index);
                            
                        end
                    end
                end
            end
            
        end
        
        function [ uncertainty ] = uncertaintyCalculate(obj, x, y, z, t, tempMeas, D)
            % UNCERTAINTYCALCULATE Calculates the net uncertainty field at
            % a given point in time, used within timeUncertaintyField
            %
            % SYNOPSIS [ uncertainty ] = uncertaintyCalculate(obj, x, y, z, t, tempMeas, D)
            %
            % INPUT obj: the object
            % x,y,z,t: the location in space and time being considered
            % tempMeas: the list of measurements which provide certainty to
            % the point
            % D: the inverse of the matrix of covariances between
            % measurements
            %
            % OUTPUT uncertainty: the uncertainty of data at a point
            
            M = 0;
            
            % compares all measurements to all other measurements,
            % following equations 5 and 6 from the paper
            for i=1:length(tempMeas(:,1))
                for j=1:length(tempMeas(:,1))
                    % sums all components of certainty
                    M = M + (obj.spacetimeAverage*exp(-abs(((sqrt((x - tempMeas(i,1)).^2 ...
                        + (y - tempMeas(i,2)).^2 + (z - tempMeas(i,3)).^2) ...
                        ./ obj.sigma))) - abs((t - tempMeas(i,4))./ obj.tau))...
                        .* D(i,j) .* exp(-abs(((sqrt((tempMeas(j,1) - x).^2 ...
                        + (tempMeas(j,2) - y).^2 + (tempMeas(j,3) - z).^2)...
                        ./ obj.sigma))) - abs(((tempMeas(j,4)) - t)./ obj.tau)));
                    
                end
                
            end
            
            
            uncertainty = obj.spacetimeAverage - M;
        end
        
        function [ uncertainty ] = timeUncertaintyField(obj, x, y, z, t, tempMeas, D)
            % TIMEUNCERTAINTYFIELD generates the uncertainty at a given place in space and time,
            % used in the gradient control law
            %
            % SYNOPSIS [ uncertainty ] = timeUncertaintyField(obj, x, y, z, t, tempMeas, D)
            %
            % INPUT obj: the object
            % x,y,z,t: the location in space and time being considered
            % tempMeas: the list of measurements which provide certainty to
            % the point
            % D: the inverse of the matrix of covariances between
            % measurements
            %
            % OUTPUT uncertainty: the uncertainty of data at a point
            
            % determines whether a point is inside the region being
            % surveyed or not, depending on its shape
            if strcmp(obj.shape,'triangle')==true
                uncertainty = zeros(1,length(x));
                
                for index=1:length(x)
                    % conditions for outside triangle
                    if (x(index) > sqrt(3)/2*obj.radius) || (x(index)...
                            < -sqrt(3)/2)*obj.radius || (y(index)...
                            > (-sqrt(3)*x(index) + 1)*obj.radius) ||...
                            (y(index) > (sqrt(3)*x(index) + 1)*obj.radius)...
                            || (y(index) < -.5*obj.radius)
                        
                        uncertainty(index) = 1;
                    else
                        uncertainty(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
                    end
                end
                
            elseif strcmp(obj.shape,'circle') == true
                uncertainty = zeros(1,length(x));
                
                
                for index=1:length(x)
                    % conditions for outside circle
                    if (x(index)^2 + (y(index))^2)^.5 > obj.radius
                        uncertainty(index) = 1;
                    else
                        uncertainty(index) = obj.uncertaintyCalculate(x(index),...
                            y(index), z(index), t, tempMeas, D);
                    end
                end
                
            elseif strcmp(obj.shape,'sphere') == true
                uncertainty = zeros(1,length(x));
                
                
                for index=1:length(x)
                    % conditions for outside sphere
                    if (x(index)^2 + y(index)^2 + z(index)^2)^.5 > obj.radius
                        uncertainty(index) = 1;
                    else
                        uncertainty(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
                    end
                end
                
                
            elseif strcmp(obj.shape,'square') == true
                uncertainty = zeros(1,length(x));
                
                
                for index=1:length(x)
                    % conditions for outside square
                    if x(index) > obj.radius || x(index) < - obj.radius || y(index) > obj.radius || y(index) < -obj.radius
                        uncertainty(index) = 1;
                    else
                        uncertainty(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
                    end
                end
                
                
                
            else
                uncertainty = zeros(1,length(x));
                
                
                for index=1:length(x)
                    %conditions for outside search area
                    if inpolygon(x(index),y(index),obj.polygon(:,1),obj.polygon(:,2)) == 0
                        uncertainty(index) = 1;
                    else
                        uncertainty(index) = obj.uncertaintyCalculate(x(index), y(index), z(index), t, tempMeas, D);
                    end
                end
            end
        end
        
        function [ point, b ] = locationTest(obj, robot, i, j, k, theta, dt)
            % LOCATIONTEST tests a point to see how much cumulative uncertainty moving
            % to it would leave behind on the other gamma close points
            %
            % SYNOPSIS [ point, b ] = locationTest(obj, robot, i, j, k, theta, dt)
            %
            % INPUT obj: the object
            % robot: the robot whose motion is being evaluated
            % i,j,k: the x,y,z increments being added while evaluating
            % the point
            % theta: the heading of the robot
            % dt: the time step, as estimated by the previous time
            %
            % OUTPUT point: the point being considered
            % b: the sum of uncertainty at gamma-close points if there were
            % a measurements at F
            
            % generates a temporary matrix including the new test
            tempMeas = [obj.measurements; robot(1) + i, robot(2) + j, robot(3) + k,...
                robot(4) + dt];
            
            % completes the covariance based on the trial sensor
            D = inv(obj.finishCovariance(obj.D, tempMeas));
            
            % finds and returns the sum uncertainties at with the new
            % measurement
            b = 0;
            angle = (theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            u = zeros(length(angle));
            
            % checks the new uncertainty at each of the possible points,
            % and returns their sum
            A = obj.timeUncertaintyField(robot(1) + v,robot(2) + w,...
                robot(3) + u, tempMeas(end,4), tempMeas, D);
            b = b + sum(A);
            point = [robot(1) + i, robot(2) + j, robot(3) + k];
            
            
            
        end
        
        function [ covariances ] = finishCovariance(obj, covariance, tempMeas)
            % FINISHCOVARIANCE calculates the covariances of the new measurement with all
            % previous ones, and combines this with the previously found
            % matrix
            %
            % SYNOPSIS [ covariances ] = finishCovariance(obj, covariance, tempMeas)
            %
            % INPUT obj: the object
            % covariance: the matrix containing covariances of all but the
            % last measurements
            % tempMeas: the list of measurements
            %
            % OUTPUT covariances: the completed matrix of covariances
            
            B = zeros(length(tempMeas(:,1)), length(tempMeas(:,1)));
            i=length(tempMeas(:,1));2;
            % Find the covariance of the last row and column
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
            % combine with the original covariance
            covariances = covariance + B;
        end
        
        function [Ftemp,bestTemp] = compileLocationTest(obj, robots, theta, index, dt)
            % COMPILELOCATIONTEST This program allows for more
            % straightfoward running of the program if multiple robots are
            % being calculated simultaneously outside of a parfor loop
            %
            % SYNOPSIS [Ftemp,bestTemp] = compileLocationTest(obj, robots, theta, index, dt)
            %
            % INPUT obj: the object
            % robots: the robots whose motion is being evaluated
            % theta: the heading of the robot
            % index: the index of the spot among the potential goals being
            % considered
            % dt: the time step, as estimated by the previous time
            %
            % OUTPUT Ftemp: the points being considered by each robot
            % bestTemp: the sum of uncertainty at gamma-close points if there were
            % a measurements at Ftemp
            
            Ftemp = zeros(length(robots(:,1)),3);
            bestTemp = zeros(length(robots(:,1)),1);
            
            % take each possible goal for each robot, and check to see
            % which goal is the best
            for u=1:length(robots(:,1))
                angle=theta(u)+pi/(.5*obj.precision):pi/(.5*obj.precision):(2*pi+theta(u));
                i = obj.gamma * cos(angle(index));
                j = obj.gamma * sin(angle(index));
                k = 0;
                
                [Ftemp(u,:),bestTemp(u,1)] = obj.locationTest(robots(u,:), i, j, k, theta(u), dt);
            end
            
        end
        
        function [ commands ] = commandGen(obj, states, Goals)
            % COMMANDGEN generates commands for the Miabots based on their goal points
            % and the coefficients for the control law
            %
            % SYNOPSIS [ commands ] = commandGen(obj, states, Goals))
            %
            % INPUT obj: the object
            % states: the n_robots X 7 matrix of current robot positions
            % Goals: the points the each robot is trying to get to
            %
            % OUTPUT commands: the n_robots X 3 matrix telling which tells
            % the Miabots how to move
            
            % send robots along their current heading at start
            if obj.t < obj.firstStepTime
                for i=1:obj.n_robots
                    commands(i,:) = [obj.firstStepSpeed 0 0];
                end
            else
                % Get current states of the robot, x,y,z,heading, and
                % velocities
                
                x = states(:,1);
                y = states(:,2);
                z = states(:,3);
                v_x = states(:,4);
                v_y = states(:,5);
                theta = states(:,6);
                theta_dot = states(:,7);
                
                xgoal = Goals(:,1)+obj.origin(1);
                ygoal = Goals(:,2)+obj.origin(2);
                zgoal = Goals(:,3)+obj.origin(3);
                
                
                % angle that the current heading is displaced from desired
                % heading
                phi = wrapToPi(atan2(ygoal-y,xgoal-x)-theta);
                
                % if statement to determine control laws for angular
                % velocity
                for i=1:length(phi)
                    if (phi(i) <= pi/2) && (phi(i) >= -pi/2)
                        u_theta(i,1) = (obj.k2)*sin(phi(i));
                    else
                        u_theta(i,1) = -(obj.k2)*sin(phi(i));
                    end
                end
                r = ((xgoal-x).^2+(ygoal-y).^2).^.5; % distance to goal
                % position
                
                
                
                % control law for forward velocity
                u_x = ((obj.k1).*r.*cos(phi));
                
                u_z = (obj.k3).*(zgoal-z);
                
                % pass forward velocity and angular velocity to the command
                % matrix
                commands = [u_x u_theta u_z];
                
                
            end
        end
        
        function [ entropy ] = determineEntropy(obj, measurements,t,heat)
            % DETERMINEENTROPTY determines the information entropy of the area being searched
            % at a given time
            %
            % SYNOPSIS [ entropy ] = determineEntropy(obj, measurements,t)
            %
            % INPUT obj: the object
            % measurements: the matrix of past and current robot states
            % t: the time being considered
            %
            % OUTPUT entropy: the total information entropy of the survey
            % area
            
            % find the covariance matrix
            D = inv(obj.fieldGen(measurements));
            
            p=0;
            H = 0;
            % set up the area to be sampled, larger than the radius, since
            % some polygons will extend beyond it
            x = -1.5*obj.radius:.15*obj.radius:1.5*obj.radius;
            y = -1.5*obj.radius:.15*obj.radius:1.5*obj.radius;
            Htemp = zeros(1,length(x));
            pTemp = zeros(1,length(x));
            
            % sum the uncertainties within the region covered
            parfor i=1:length(x)
                for j=1:length(y)
                    if inpolygon(x(i), y(j), obj.polygon(:,1), ...
                            obj.polygon(:,2)) == 1
                        
                        Htemp(i) = Htemp(i) + obj.timeUncertaintyField(x(i),...
                            y(j), 0, t, measurements, D)
                        
                        pTemp(i) = pTemp(i)+1;
                    end
                end
                
            end
            H = H + sum(Htemp);
            p = p + sum(pTemp);
            
            if heat == true
                for i=1:length(x)
                    for j=1:length(x)
                        if Htemp(i,j) == 0
                            Htemp(i,j) = 1;
                        end
                    end
                end
                HeatMap(1 - Htemp);
            end
            
            % for discretized area, divide the sum by the total number of
            % points
            entropy = 1-H/p;
        end
    end
    
    methods
        % voronoi control law-specific methods
        
        function [ centroids ] = centroid( obj )
            % finds the centroid of each voronoi region surrounding a
            % sensor, weighted by uncertainty
            %
            % SYNOPSIS [ centroids ] = centroid( obj )
            %
            % INPUT obj: the object
            %
            % OUTPUT centroids: the coordinates of the centroids
            
            % finds the inverse of covariance, needed for the uncertainty
            obj.D = inv(obj.fieldGen(obj.measurements));
            
            % arrays for finding the center of mass of each region
            densitySums = zeros(length(obj.gridSize),length(obj.robots(:,1)));
            sumsx = zeros(length(obj.gridSize), length(obj.robots(:,1)));
            sumsy = zeros(length(obj.gridSize), length(obj.robots(:,1)));
            sumsz = zeros(length(obj.gridSize), length(obj.robots(:,1)));
            sensors = obj.robots;
            
            % check each point in the grid, and assign it to a voronoi region
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
        
        function [ M ] = certainty(obj, x, y, z, measurements)
            % CERTAINTY Calculates the net certainty at a
            % given point in time, used within uncertaintyField
            %
            % SYNOPSIS [ M ] = uncertainty(obj, x, y, z, measurements)
            %
            % INPUT obj: the object
            % x,y,z: the location of the point being considered
            % measurements: the list of past measurement locations
            %
            % OUTPUT M: the certainty at a point
            
            M = 0;
            
            % for speed, doesn't not compute for measurements far
            % away from each other spatially or temporally
            % follows equations 5 and 6 from the paper
            for i=1:length(measurements(:,1))
                
                if abs(((measurements(i,1)).^2 ...
                        + (measurements(i,2)).^2 + (measurements(i,3)).^2).^.5 - (x.^2 + y.^2 + z.^2).^.5) ...
                        < 4 * obj.sigma
                    for j=1:length(measurements(:,1))
                        
                        if abs(((measurements(j,1)).^2 ...
                                + (measurements(j,2)).^2 + (measurements(j,3)).^2).^.5 ...
                                - (x.^2 + y.^2)^.5) < 4 * obj.sigma
                            
                            % sums all components of certainty
                            M = M + (obj.spacetimeAverage*exp(-abs(((sqrt((x ...
                                - measurements(i,1)).^2 + (y - measurements(i,2)).^2 ...
                                + (z - measurements(i,3)).^2) ./ obj.sigma))) ...
                                - abs((obj.t - measurements(i,4))./ obj.tau)) ...
                                .* obj.D(i,j) .* exp(-abs(((sqrt((measurements(j,1)...
                                - x).^2 + (measurements(j,2) - y).^2 + (measurements(j,3)...
                                - z).^2)./ obj.sigma))) - abs(((measurements(j,4)) - obj.t)...
                                ./ obj.tau)));
                            
                        end
                        
                    end
                    
                end
            end
        end
        
        function [ A ] = uncertaintyField(obj, x, y, z)
            % UNCERTAINTYFIELD generates the uncertainty at a point in space at the current
            % time, used by the voronoi control law
            %
            % SYNOPSIS [ M ] = uncertainty(obj, x, y, z, measurements)
            %
            % INPUT obj: the object
            % x,y,z: the location of the point being considered
            %
            % OUTPUT A: the uncertainty at a point
            
            % conditions for outside an equilateral triangle
            measurements = obj.measurements;
            if strcmp(obj.shape,'triangle')==true
                if x > obj.radius*sqrt(3)/2 || x < -(obj.radius)*sqrt(3)/2 || y > obj.radius*(-sqrt(3)*x + 1) || y > obj.radius*(sqrt(3)*x + 1) || y < -(obj.radius)*.5
                    
                    M = obj.spacetimeAverage;
                else
                    M = obj.certainty(x,y,z,measurements);
                end
                A = obj.spacetimeAverage - M;
                
                % conditions for a square region of search
            elseif strcmp(obj.shape,'square')==true
                if x > obj.radius || x < -obj.radius || y > obj.radius || y < -obj.radius
                    
                    M = obj.spacetimeAverage;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = obj.spacetimeAverage - M;
                
                % conditions for a circular region of search
            elseif strcmp(obj.shape,'circle')==true
                if (x^2 + (y-.5)^2 > obj.radius^2)
                    
                    M = obj.spacetimeAverage;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = obj.spacetimeAverage - M;
                
            elseif strcmp(obj.shape,'sphere')==true
                if (x^2 + y^2 + z^2 > obj.radius^2)
                    
                    M = obj.spacetimeAverage;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = obj.spacetimeAverage - M;
                
                % conditions for a custom region of search
            elseif strcmp(obj.shape,'custom') == 1
                if inpolygon(x,y,obj.polygon(:,1),obj.polygon(:,2)) == 0
                    M = obj.spacetimeAverage;
                else
                    M = obj.uncertainty(x,y,z,measurements);
                end
                A = obj.spacetimeAverage - M;
            end
            
        end
        
        function [ waypoints ] = voronoi_control_law(obj, t, states)
            % control law for the voronoi based control law, which is the
            % standard to which the gradient control law is compared
            %
            % SYNOPSIS [ waypoints ] = voronoi_control_law(obj, t, states)
            %
            % INPUT obj: the object
            % t: the current time
            % states: the n_robots X 7 matrix of the robots' current states
            %
            % OUTPUT waypoints: the points for the robots to travel to
            
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
        
        
    end
    
end

