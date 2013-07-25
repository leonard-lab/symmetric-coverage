classdef streamedField < handle
    %Class to run voronoi and gradient based symmetric searchs
    
    properties
        sigma = .3;    % time constant for spatial separation of measurements
        tau = .8;       % time constant for temporal separation of measurements
        mu = .15;        % uncertainty in measurements, a characteristic of the sensors
        gamma = .1;    % radius over which a gradient is determined for motion
        timeToDeleteSelf = 15;
        timeToDeleteOther = 2;
        runTime;        % how many seconds the Miabots will run for
        n_robots;   % number of robots
        k1 = 2;         % coefficient for forward velocity in control law
        k2 = 1;         % coefficient for angular velocity in control law
        k3 = 1;         % coefficient for z velocity in control law
        % matrix of covariances between measurements
        radius = 1;    % distance to edge of survey area
        shape = 'triangle'
        % shape of the boundary area. Currently accepted are circle,
        % square, triangle, and custom.
        
        precision = 12; % number of spots considered for goal points
        t;              % current time
        tPast = -.04;          % previous time
        D;
        polygon;        % vertices for a custom shape
        q = 0;          % counter used in fast runspeed to determine leader
        measurements = zeros(0,4,0);
        robots = zeros(0,4);
        g = 0;
        h = 0;
        origin = [0 0 0];
        entropyList;
        storeMeas = cell(1,1);
        storeT = 0;
        selfMeasurements = zeros(0,4,0);
        otherMeasurements = zeros(0,4,0);
    end
    
    methods
        
        function obj = streamedField(n)
            % generates a new field object
            obj.n_robots = n;
            % initialize sensors
            for i=1:obj.n_robots
                obj.robots(i,:) = [0 0 0 0];
            end
            obj.h = zeros(1,n);
            obj.storeMeas{1} = zeros(0,4);
            
        end
        
        function [ commands ] = control_law(obj, t, states)
            % gradient control law which views gamma-close spots to a
            % sensor and directs a Miabot to the best location
            
            obj.t = t;
          
            % initialize the sensor objects to the current positions, and
            % record the current measurements
            for i=1:obj.n_robots
                obj.robots(i,:) = [states(i,1)-obj.origin(1) states(i,2)-obj.origin(2) states(i,3)-obj.origin(3) t];
         
                obj.selfMeasurements(mod(obj.g,obj.timeToDeleteSelf)+1,:,i) = obj.robots(i,:);
                for j=1:obj.n_robots
                    if j~=i
                        obj.otherMeasurements(mod(obj.h(j),(obj.n_robots-1)*obj.timeToDeleteOther)+1,:,j) = obj.robots(i,:);
                        obj.h(j) = obj.h(j) + 1;
                    end
                end
            end
            obj.g = obj.g+1;
            obj.measurements = zeros(length(obj.selfMeasurements(:,1,1))+length(obj.otherMeasurements(:,1,1)),4,obj.n_robots);
            
            for i=1:obj.n_robots
                obj.measurements(:,:,i) = [obj.selfMeasurements(:,:,i); obj.otherMeasurements(:,:,i)];
            end
            obj.D = zeros(length(obj.measurements(:,1)) + 1, length(obj.measurements(:,1)) + 1);
            obj.D(1:end-1,1:end-1) = obj.fieldGen(obj.measurements);
            
            
                % calculates each robot individually, per the actual control
                % law
                
                parfor i=1:obj.n_robots
                    covariance = zeros(length(obj.measurements(:,1,i)) + 1, length(obj.measurements(:,1,i)) + 1);
                    covariance(1:end-1,1:end-1) = obj.fieldGen(obj.measurements(:,:,i));
                    measurements = obj.measurements(:,:,i);
                   
                    Goals(i,:) = obj.bestDirection(obj.robots(i,:), states(i,6),measurements,covariance);
                    
                end
               commands = obj.commandGen(states, Goals);
            
    
            %states
            %commands
            obj.q = obj.q + 1;
            obj.tPast = obj.t;
            obj.storeMeas{obj.q+1} = [obj.storeMeas{obj.q}; obj.robots];
            %obj.entropyList = [obj.entropyList; obj.determineEntropy(obj.measurements, obj.t)];
            obj.storeT = [obj.storeT; obj.t];
        end
        
        function [ D ] = fieldGen(obj, measurements)
            % generates the covariance between two measurement points, and
            % returns a matrix of it
            
            C = zeros(length(measurements(:,1)), length(measurements(:,1)));
            for i=1:length(measurements(:,1))
                for j=1:length(measurements(:,1))
                    
                    C(i,j) = exp(-abs(((sqrt((measurements(i,1) - measurements(j,1))^2 ...
                        + (measurements(i,2) - measurements(j,2))^2 + (measurements(i,3) - measurements(j,3))^2)/ obj.sigma))) ...
                        - abs((measurements(i,4) - measurements(j,4))/ obj.tau));
                    
                end
            end
            
            % adds the uncertainty of the sensors to their variance
            D = C + obj.mu * eye(length(measurements(:,1)));
            
        end
        
        function [ F ] = bestDirection(obj, robots, theta, measurements, covariance)
            % used in the gradient control law, determines what direction
            % the robot should move in
            
            best = Inf(length(robots(:,1)),1); % tracks what direction would bring the most certainty
            
            F = zeros(length(robots(:,1)),3);
            % array of angles to be checked
            
            Ftemp = zeros(length(robots(:,1)), 3, obj.precision);
            bestTemp = zeros(length(robots(:,1)),obj.precision);
            dt = obj.t - obj.tPast; % we assume timesteps are roughly equal and use this for the future step
            
            % check each spot and determine their quality
            
            angle=theta+pi/(.5*obj.precision):pi/(.5*obj.precision):(2*pi+theta);
            for index=1:length(angle)
                i = obj.gamma * cos(angle(index));
                j = obj.gamma * sin(angle(index));
                k = 0;
                for u=1:length(robots(:,1))
                    [Ftemp(u,:,index),bestTemp(u,index)] = obj.locationTest(robots(u,:), i, j, k, theta, dt, measurements, covariance);
                end
                
            end
            

            % pick the best direction to move in
            for i=1:length(robots(:,1))
                for index=1:obj.precision
                 
                    if bestTemp(i,index) < best(i)
                        best(i) = bestTemp(i,index);
                        F(i,:) = Ftemp(i,:,index);
                        
                        % if two spots tie, pick the first going counterclockwise
                        % for the current heading
                    elseif bestTemp(i,index) == best(i)
                  
                        % theta1 and theta2 are the angles from the heading
                        
                        theta1 = wrapTo2Pi(atan2(Ftemp(i,2,index),Ftemp(i,1,index)) - theta(i));
                      
                        
                        
                        theta2 = wrapTo2Pi(atan2(F(i,2)-robots(i,2),F(i,1)-robots(i,1)) - theta(i));
                   
                    if theta1 < theta2
                        F(i,:) = Ftemp(i,:,index);
                        best(i) = bestTemp(i,index);
                        
                    end
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
                    
                    if (x(index) > sqrt(3)/2*obj.radius) || (x(index) < -sqrt(3)/2)*obj.radius || (y(index) > (-sqrt(3)*x(index) + 1)*obj.radius) || (y(index) > (sqrt(3)*x(index) + 1)*obj.radius) || (y(index) < -.5*obj.radius)
                        
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
                    
                    if (x(index)^2 + (y(index))^2)^.5 > obj.radius
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
        
        function [ k ] = lineSum(obj, robot, theta, tempMeas,covariance)
            % finds the sum of the uncertainty field over the region of
            % gamma close points to the gradient field
            
            k = 0;
            
            % completes the covariance based on the trial sensor
            D = inv(obj.finishCovariance(covariance, tempMeas));
            
            
            angle = (theta+pi/(.5*obj.precision)):pi/(.5*obj.precision):(2*pi+theta);
            v = obj.gamma * cos(angle);
            w = obj.gamma * sin(angle);
            u = zeros(length(angle));
            
            % checks the new uncertainty at each of the possible points,
            % and returns their sum
            A = obj.timeUncertaintyField(robot(1) + v,robot(2) + w, robot(3) + u, tempMeas(end,4), tempMeas, D);
            k = k + sum(A);
            
            
        end
        
        function [ F, b ] = locationTest(obj, robot, i, j, k, theta, dt, measurements, covariance)
            % tests a point to see how much cumulative uncertainty moving
            % to it would leave behind

            % generates a temporary matrix including the new test
            
            tempMeas = [measurements; robot(1) + i, robot(2) + j, robot(3) + k,...
                robot(4) + dt];
            
            % finds and returns the sum uncertainties at with the new
            % measurement
            b = obj.lineSum(robot, theta, tempMeas, covariance);
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
        
       
        function [ commands ] = commandGen(obj, states, Goals)
            % send robots along their current heading at start
            if obj.t < .15
                for i=1:obj.n_robots
                    commands(i,:) = [.2 0 0];
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
        
        function [ entropy ] = determineEntropy(obj, measurements,t)
            D = inv(obj.fieldGen(measurements));
            
            p=0;
            H = 0;
            x = -1.5:.1:1.5;
            y = -1.5:.1:1.5;
            Htemp = zeros(1,length(x));
            pTemp = zeros(1,length(x));
            parfor i=1:length(x)
                for j=1:length(y)
                    
                    if inpolygon(x(i),y(j),obj.polygon(:,1),obj.polygon(:,2)) == 1
                        Htemp(i) = Htemp(i) + obj.timeUncertaintyField(x(i), y(j), 0, t, measurements, D)
                        pTemp(i) = pTemp(i)+1;
                    end
                end

            end
             H = H + sum(Htemp);
             p = p + sum(pTemp);
            
            entropy = 1-H/p;
        end
    end
    
end

