classdef streamedField < handle
    % This class runs gradient based search descriped in the symmetric 
    % coverage paper, using the Miabots class. The user must provide a 
    % runtime, a shape of the area to be searched, and initial positions.
    %
    % Summary provided below. More help can be found by typing doc field
    % or help field.method_name.
    %
    % SYNTAX
    %
    % S = streamedField(length(init(:,1)), shape, radius);
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
    %    robots: stores current positions of robots
    %
    %    origin: center of the survey area, which is treated as the origin
    %
    %    timeToDeleteSelf: number of time steps after which a robot deletes
    %    its own old positions
    %
    %    timeToDeleteOther: number of time steps after which a robot 
    %    deletes its record of the other robots' positions
    %
    %    selfMeasurements: stores a matrix of each robot's past locations
    %
    %    otherMeasurements: stores a matrix for each robot, consisting of
    %    the other robots' past locations
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
    %   commandGen: generates commands for the Miabots based on their goal
    %   points and the coefficients for the control law
    %
    %   determineEntropy: determines the information entropy of the area
    %   being searched at a given time
    %
    %   combineMeasurements: combines self and other measurements for each
    %   robot
    %
    %   twoRobotsCircle: sets the properties used for the demo of two
    %   robots in a circular survey area
    % 
    %   twoRobotsSquare(obj): sets the properties used for the demo of two
    %   robots in a square survey area 
    
    properties
        sigma = .2;            % time constant for spatial separation of measurements
        tau = .8;              % time constant for temporal separation of measurements
        mu = .1;               % uncertainty in measurements, a characteristic of the sensors
        gamma = .08;           % radius over which a gradient is determined for motion
        timeToDeleteSelf = 7;  % number of time steps after which a robot deletes its own old positions
        timeToDeleteOther = 2; % number of time steps after which a robot deletes its record of the other robots' positions
        runTime;               % how many seconds the Miabots will run for
        spacetimeAverage = 1;  % coefficient for field covariance
        n_robots;              % number of robots
        k1 = 1;                % coefficient for forward velocity in control law
        k2 = 1;                % coefficient for angular velocity in control law
        k3 = 1;                % coefficient for z velocity in control law
        radius = .5;           % distance to edge of survey area from origin
        origin = [0 -0.5 0];   % movable center which is treated as the origin
        shape = 'triangle'     % shape of the boundary area. Currently accepted are circle, square, triangle, and custom.
        precision = 6;         % number of spots considered for goal points
        t;                     % current time
        tPast = -.07;          % previous time
        D;                     % stores the covariance field for some versions of the control law
        polygon;               % vertices for a custom shape
        firstStepTime = .15    % time for which the robots move staight along heading
        firstStepSpeed = .2    % speed that the robots move at for the first step
        
        robots = zeros(0,4);             % stores the current location of the robots
        selfMeasurements = zeros(0,4,0); % stores a matrix of each robot's past locations
        otherMeasurements = zeros(0,4,0);% stores a matrix for each robot, consisting of the other robots' past locations
    end
    
    properties (GetAccess = private)
        selfCounter = 0;   % counter used to overwrite old data in self measurements
        otherCounter = 0;   % counter used to overwrite old data in other measurements
        
    end
    
    methods
        
        function obj = streamedField(n, shape, radius)
            % generates a new field object
            
            obj.n_robots = n;
            % initialize sensors
            for i=1:obj.n_robots
                obj.robots(i,:) = [0 0 0 0];
            end
            obj.otherCounter = zeros(1,n);
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
                
                % tracks own position
                obj.selfMeasurements(mod(obj.selfCounter,obj.timeToDeleteSelf)+1,:,i) = obj.robots(i,:);
                for j=1:obj.n_robots
                    if j~=i
                        % tracks the positions of the other robots
                        obj.otherMeasurements(mod(obj.otherCounter(j),(obj.n_robots-1)*obj.timeToDeleteOther)+1,:,j) = obj.robots(i,:);
                        obj.otherCounter(j) = obj.otherCounter(j) + 1;
                    end
                end
            end
            
            
            positions = obj.combineMeasurements();
            
            % calculates each robot individually, per the actual control
            % law
            
            parfor i=1:obj.n_robots
                covariance = zeros(length(positions(:,1,i)) + 1);
                covariance(1:end-1,1:end-1) = obj.fieldGen(positions(:,:,i));
                measurements = positions(:,:,i);
                
                Goals(i,:) = obj.bestDirection(obj.robots(i,:), states(i,6),measurements,covariance);
            end
            commands = obj.commandGen(states, Goals);
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
                    % equation to find covariance, number 4 in the paper
                    C(i,j) = exp(-abs(((sqrt((measurements(i,1)...
                        - measurements(j,1))^2 + (measurements(i,2)...
                        - measurements(j,2))^2 + (measurements(i,3)...
                        - measurements(j,3))^2)/ obj.sigma))) ...
                        - abs((measurements(i,4) - measurements(j,4))/ obj.tau));
                    
                end
            end
            
            % adds the uncertainty of the sensors to their variance
            covariances = C + obj.mu * eye(length(measurements(:,1)));
            
        end
        
        function [ goal ] = bestDirection(obj, robots, theta, measurements,...
                covariance)
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
            
            best = Inf; % tracks what direction
            % would bring the most certainty
            
            goal = zeros(1,3);
            % array of angles to be checked
            
            goaltemp = zeros(1, 3, obj.precision);
            bestTemp = zeros(1,obj.precision);
            
            dt = obj.t - obj.tPast; % we assume timesteps are equal
            % and use this for the future step
            
            % check each spot and determine their quality
            angle=theta+pi/(.5*obj.precision):pi/(.5*obj.precision):(2*pi+theta);
            for index=1:length(angle)
                i = obj.gamma * cos(angle(index));
                j = obj.gamma * sin(angle(index));
                k = 0;
                [goaltemp(1,:,index),bestTemp(1,index)]...
                    = obj.locationTest(robots, i, j, k, theta,...
                    dt, measurements, covariance);
                
            end
            
            
            % pick the best direction to move in
            for index=1:obj.precision
                
                if bestTemp(1,index) < best
                    % take the better of the two positions
                    best = bestTemp(1,index);
                    goal(1,:) = goaltemp(1,:,index);
                    
                    % if two spots tie, pick the first going
                    % counterclockwise from the current heading
                elseif bestTemp(1,index) == best
                    
                    % theta1 and theta2 are the angles from the heading
                    
                    theta1 = wrapTo2Pi(atan2(goaltemp(1,2,index),...
                        goaltemp(1,1,index)) - theta);
                    
                    
                    
                    theta2 = wrapTo2Pi(atan2(goal(1,2)-robots(1,2),...
                        goal(1,1)-robots(1,1)) - theta);
                    
                    if theta1 < theta2
                        goal(1,:) = goaltemp(1,:,index);
                        best(1) = bestTemp(1,index);
                        
                    end
                    
                end
                
            end
            
        end
        
        function [ Uncertainty ] = uncertaintyCalculate(obj, x, y, z, t, tempMeas, D)
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
                    M = M + (obj.spacetimeAverage*exp(-abs(((sqrt((x - tempMeas(i,1)).^2 + (y ...
                        - tempMeas(i,2)).^2 + (z - tempMeas(i,3)).^2)...
                        ./ obj.sigma))) - abs((t - tempMeas(i,4))...
                        ./ obj.tau)) .* D(i,j) .* exp(-abs(((sqrt((tempMeas(j,1)...
                        - x).^2 + (tempMeas(j,2) - y).^2 + (tempMeas(j,3) - z).^2)...
                        ./ obj.sigma))) - abs(((tempMeas(j,4)) - t)...
                        ./ obj.tau)));
                    
                end
                
                
            end
            
            
            Uncertainty = obj.spacetimeAverage - M;
        end
        
        function [ Uncertainty ] = timeUncertaintyField(obj, x, y, z, t, tempMeas, D)
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
            
            if strcmp(obj.shape,'triangle')==true
                Uncertainty = zeros(1,length(x));
                for index=1:length(x)
                    % conditions for outside triangle
                    if (x(index) > sqrt(3)/2*obj.radius) || (x(index) <...
                            -sqrt(3)/2)*obj.radius || (y(index) > ...
                            (-sqrt(3)*x(index) + 1)*obj.radius) ||...
                            (y(index) > (sqrt(3)*x(index) + 1)*obj.radius)...
                            || (y(index) < -.5*obj.radius)
                        
                        Uncertainty(index) = 1;
                    else
                        Uncertainty(index)...
                            = obj.uncertaintyCalculate(x(index), y(index),...
                            z(index), t, tempMeas, D);
                    end
                end
                % conditions for a circular region of search
            elseif strcmp(obj.shape,'circle') == true
                Uncertainty = zeros(1,length(x));
                for index=1:length(x)
                    %conditions for outside circle
                    if (x(index)^2 + (y(index))^2)^.5 > obj.radius
                        Uncertainty(index) = 1;
                    else
                        Uncertainty(index)...
                            = obj.uncertaintyCalculate(x(index), y(index),...
                            z(index), t, tempMeas, D);
                    end
                end
                
            elseif strcmp(obj.shape,'sphere') == true
                Uncertainty = zeros(1,length(x));
                for index=1:length(x)
                    %conditions for outside sphere
                    if (x(index)^2 + y(index)^2 + z(index)^2)^.5 > obj.radius
                        Uncertainty(index) = 1;
                    else
                        Uncertainty(index)...
                            = obj.uncertaintyCalculate(x(index), y(index),...
                            z(index), t, tempMeas, D);
                    end
                end
                
                % conditions for a square region of search
            elseif strcmp(obj.shape,'square') == true
                Uncertainty = zeros(1,length(x));
                for index=1:length(x)
                    %conditions for outside square
                    if x(index) > obj.radius || x(index) < - obj.radius ||...
                            y(index) > obj.radius || y(index) < -obj.radius
                        
                        Uncertainty(index) = 1;
                    else
                        Uncertainty(index)...
                            = obj.uncertaintyCalculate(x(index), y(index),...
                            z(index), t, tempMeas, D);
                    end
                end
                
                
                % conditions for a custom region of search
            elseif strcmp(obj.shape,'custom') == true
                Uncertainty = zeros(1,length(x));
                for index=1:length(x)
                    %conditions for outside sample area
                    if inpolygon(x(index), y(index), obj.polygon(:,1), ...
                            obj.polygon(:,2)) == 0
                        Uncertainty(index) = 1;
                    else
                        Uncertainty(index)...
                            = obj.uncertaintyCalculate(x(index), y(index),...
                            z(index), t, tempMeas, D);
                        
                    end
                end
            end
        end
        
        function [ F, b ] = locationTest(obj, robot, i, j, k, theta, dt,...
                measurements, covariance)
            % LOCATIONTEST tests a point to see how much cumulative uncertainty moving
            % to it would leave behind on the other gamma close points
            %
            % SYNOPSIS [ F, b ] = locationTest(obj, robot, i, j, k, theta, dt)
            %
            % INPUT obj: the object
            % robot: the robot whose motion is being evaluated
            % i,j,k: the x,y,z increments being added while evaluating
            % the point
            % theta: the heading of the robot
            % dt: the time step, as estimated by the previous time
            %
            % OUTPUT F: the point being considered
            % b: the sum of uncertainty at gamma-close points if there were
            % a measurements at F
            
            % generates a temporary matrix including the new test
            
            tempMeas = [measurements; robot(1) + i, robot(2) + j,...
                robot(3) + k, robot(4) + dt];
            
            % finds and returns the sum uncertainties at with the new
            % measurement
            b = 0;
            
            % completes the covariance based on the trial sensor
            D = inv(obj.finishCovariance(covariance, tempMeas));
            
            
            angle = (theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):...
                (2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            u = zeros(length(angle));
            
            % checks the new uncertainty at each of the possible points,
            % and returns their sum
            A = obj.timeUncertaintyField(robot(1) + v,robot(2) + w,...
                robot(3) + u, tempMeas(end,4), tempMeas, D);
            b = b + sum(A);
            F = [robot(1) + i, robot(2) + j, robot(3) + k];
            
            
            
        end
        
        function [ covariances ] = finishCovariance(obj, initialCovariance, tempMeas)
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
            
            % compute the last row and column of the covariance matrix
            B = zeros(length(tempMeas(:,1)), length(tempMeas(:,1)));
            i=length(tempMeas(:,1));
            for j=1:length(tempMeas(:,1))
                B(i,j) = exp(-abs(((sqrt((tempMeas(i,1) - tempMeas(j,1))^2 ...
                    + (tempMeas(i,2) - tempMeas(j,2))^2 + (tempMeas(i,3)...
                    - tempMeas(j,3))^2)/ obj.sigma))) ...
                    - abs((tempMeas(i,4) - tempMeas(j,4))/ obj.tau));
                
            end
            
            j=length(tempMeas(:,1));
            for i=1:length(tempMeas(:,1))
                B(i,j) = exp(-abs(((sqrt((tempMeas(i,1) - tempMeas(j,1))^2 ...
                    + (tempMeas(i,2) - tempMeas(j,2))^2 + (tempMeas(i,3)...
                    - tempMeas(j,3))^2)/ obj.sigma))) ...
                    - abs((tempMeas(i,4) - tempMeas(j,4))/ obj.tau));
                
            end
            
            % combine with the original matrix
            B(end,end) = B(end,end) + obj.mu;
            covariances = initialCovariance + B;
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
            x = -1.5*obj.radius:.1*obj.radius:1.5*obj.radius;
            y = -1.5*obj.radius:.1*obj.radius:1.5*obj.radius;
            Htemp = zeros(length(x));
            pTemp = zeros(1,length(x));
            
            % sum the uncertainties within the region covered
            parfor i=1:length(x)
                n = zeros(1,length(x));
                for j=1:length(x)
                    if inpolygon(x(i), y(j), obj.polygon(:,1), ...
                            obj.polygon(:,2)) == 1
                        
                        n(j) = obj.timeUncertaintyField(x(i),...
                            y(j), 0, t, measurements, D)
                        
                        pTemp(i) = pTemp(i)+1;
                        
                    end
                    Htemp(i,:) = n;
                end
                
            end
            H = sum(sum(Htemp));
            p = sum(pTemp);
            
            % draw heat maps of certainty if the users wants them
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
            entropy = 1-H/p;
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
        
        function [ positions ] = combineMeasurements(obj)
            % COMBINEMEASUREMENTS combines self and other measurements for
            % each robot
            %
            % SYNOPSIS [ positions ] = combineMeasurements(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT positions: the matrix of stored positions of robots
            
            obj.selfCounter = obj.selfCounter+1;
            selfMeasSize = length(obj.selfMeasurements(:,1,1));
            otherMeasSize = length(obj.otherMeasurements(:,1,1));
            positions = zeros(selfMeasSize+otherMeasSize,4,obj.n_robots);
            
            for i=1:obj.n_robots
                positions(:,:,i) = [obj.selfMeasurements(:,:,i); obj.otherMeasurements(:,:,i)];
            end
        end
        
        function [] = twoRobotsCircle(obj)
            % sets the properties used for the demo of two robots in a
            % circular survey area
            
            obj.sigma = .1;      % time constant for spatial separation of measurements
            obj.tau = .8;       % time constant for temporal separation of measurements
            obj.mu = .1;        % uncertainty in measurements, a characteristic of the sensors
            obj.gamma = .1;      % radius over which a gradient is determined for motion
            obj.timeToDeleteSelf = 7; % number of time steps after which a robot deletes its own old positions
            obj.timeToDeleteOther = 2; % number of time steps after which a robot deletes the other's old positions
            obj.k1 = 1;          % coefficient for forward velocity in control law
            obj.k2 = 1;          % coefficient for angular velocity in control law
            obj.k3 = 1;          % coefficient for z velocity in control law
            obj.origin = [0 -.50 0];% movable center which is treated as the origin
        end
        
        function [] = twoRobotsSquare(obj)
            % sets the properties used for the demo of two robots in a
            % square survey area
            
            obj.sigma = .1;        % time constant for spatial separation of measurements
            obj.tau = .8;          % time constant for temporal separation of measurements
            obj.mu = .1;          % uncertainty in measurements, a characteristic of the sensors
            obj.gamma = .08;      % radius over which a gradient is determined for motion
            obj.timeToDeleteSelf = 7; % number of time steps after which a robot deletes its own old positions
            obj.timeToDeleteOther = 2; % number of time steps after which a robot deletes the other's old positions
            
            obj.k1 = 1;          % coefficient for forward velocity in control law
            obj.k2 = 1;          % coefficient for angular velocity in control law
            obj.k3 = 1;          % coefficient for z velocity in control law
            obj.origin = [0 -.50 0];% movable center which is treated as the origin
        end
    end
    
end

