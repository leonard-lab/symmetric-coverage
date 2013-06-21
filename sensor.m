classdef sensor < handle
    %Sensor class used to represent robot positions and measurements
    %   Detailed explanation goes here
    
    properties
        x; % x position in field
        y; % y position in field
        t; % time position in field
    end
    
    methods
        function obj = sensor(x,y,t)
            obj.x = x;
            obj.y = y;
            obj.t = t;
            
        end
        
        function [meas] = addField(obj, meas)
            meas = obj.measure(meas);
            obj.t = obj.t + 1;
        end
        
        function [meas] = moveSensor(obj, meas)
                meas = obj.measure(meas);
                obj.t = obj.t + 1;
                obj.x = obj.x + cos(pi * obj.t / 10);
                obj.y = obj.y + sin(pi * obj.t / 10);
            
        end
        
        function [meas] = goToCentroid(obj, meas, goal)
            obj.x = goal(1);
            obj.y = goal(2);
            obj.t = obj.t + .04;
            %meas = obj.measure(meas);

        end
        
        function [meas] = measure(obj, meas)
            temp = sensor(obj.x, obj.y, obj.t);
            meas = [meas; temp];
        end
                
            
                        
    end
    
end

