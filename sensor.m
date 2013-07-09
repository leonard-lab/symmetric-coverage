classdef sensor < handle
    %Sensor class used to represent robot positions and measurements
    %   Detailed explanation goes here
    
    properties
        x; % x position in field
        y; % y position in field
        z; % z position in field
        t; % time position in field
    end
    
    methods
        function obj = sensor(x,y,z,t)
            % generates a new sensor object and given location and time
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.t = t;
            
        end
        
        function [meas] = addField(obj, meas)
            % used for testing purposes, produces a stationary sensor
            % measurement
            
            meas = obj.measure(meas);
            obj.t = obj.t + 1;
        end
        
        function [meas] = moveSensor(obj, meas)
            % used for testing purposes, produces a sensor which takes
            % measurements while moving in a circle
            
                meas = obj.measure(meas);
                obj.t = obj.t + 1;
                obj.x = obj.x + cos(pi * obj.t / 10);
                obj.y = obj.y + sin(pi * obj.t / 10);
            
        end
        
        function [meas] = goToCentroid(obj, meas, goal)
            % sends the sensor object to a given point
            
            obj.x = goal(1);
            obj.y = goal(2);
            obj.z = goal(3);
            obj.t = obj.t + .04;
            %meas = obj.measure(meas);

        end
        
        function [meas, measurements] = measure(obj, meas, measurements)
            % Takes a measurement at the given object's location in space
            % and time.
            temp = sensor(obj.x, obj.y, obj.z, obj.t);
            meas = [meas; temp];
            measurements = [measurements; obj.x obj.y, obj.z, obj.t];
        end
                
            
                        
    end
    
end

