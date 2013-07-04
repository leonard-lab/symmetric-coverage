% this script will run the voronoi based control law
clear all
S = field();
%close all
S.runTime = 1;
if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open 2 % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);
noise = [0.00 0.00 0 0.00];
init = [sqrt(3)/20 -.05 0 0; -sqrt(3)/20 -.05 0 pi/3; 0 .1 0 5*pi/3];% -sqrt(3)/20 .05 0 0; sqrt(3)/20 .05 0 pi/3; 0 -.1 0 5*pi/3];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', true, 'sim_noise', noise);
m.start
%A = zeros(16,20);
%for i=1:16
%    for j=1:20
%        A(j,i) = S.timeErrorField(i/10 - .6,j/10 - .5,S.runTime)
%    end
%end
%HeatMap(A);
        
%{
D = field();
D.runTime = 3;
D.t = 0;
theta = [0 0 0];
            A = zeros(floor(D.runTime/.04),2);
            B = zeros(floor(D.runTime/.04),2);
            C = zeros(floor(D.runTime/.04),2);
            %D = zeros(floor(runtime/.04),2);
            while D.t < D.runTime
                D.sensors = [D.a; D.b; D.c; D.d];
                A(floor(D.t/.04) + 1,1) = D.a.x;
                A(floor(D.t/.04) + 1,2) = D.a.y;
                B(floor(D.t/.04) + 1,1) = D.b.x;
                B(floor(D.t/.04) + 1,2) = D.b.y;
                C(floor(D.t/.04) + 1,1) = D.c.x;
                C(floor(D.t/.04) + 1,2) = D.c.y;
                %D(floor(obj.t/.04) + 1,1) = obj.d.x;
                %D(floor(obj.t/.04) + 1,2) = obj.d.y;
                rot1 = [cos(-2*pi/3) sin(-2*pi/3); -sin(-2*pi/3) cos(-2*pi/3)];
                rot2 = [cos(2*pi/3) sin(2*pi/3); -sin(2*pi/3) cos(2*pi/3)];
                F = D.bestDirection(D.sensors(1), theta(1));
                G = F*rot1;
                H = F * rot2;
                D.meas = D.a.goToCentroid(D.meas, F);
                D.meas = D.b.goToCentroid(D.meas, G);
                D.meas = D.c.goToCentroid(D.meas, H);
                %obj.meas = obj.d.goToCentroid(obj.meas, obj.bestDirection(obj.sensors(4), theta(4)));
                D.meas = D.a.measure(D.meas);
                D.meas = D.b.measure(D.meas);
                D.meas = D.c.measure(D.meas);
                %obj.meas = obj.d.measure(obj.meas);
                D.t = D.t + .04
                D.remove();
                
            end
%}
% plots the resulting path of the two robots against the ideal

figure
plot(m.get_history(1,'x'), m.get_history(1,'y'),...
    m.get_history(2,'x'), m.get_history(2,'y'), m.get_history(3,'x'), m.get_history(3,'y'));%, m.get_history(4,'x'), m.get_history(4,'y'),...
    %m.get_history(5,'x'), m.get_history(5,'y'), m.get_history(6,'x'), m.get_history(6,'y')); %, A(:,1), A(:,2), B(:,1), B(:,2), C(:,1), C(:,2));
xlabel('X-position');
ylabel('Y-position');
line([-.866 0], [-.5 1]);
line([.866 0], [-.5 1]);
line([-.866 .866], [-.5 -.5]);
%plot(circle(0,0,.75));
legend('Robot 1', 'Robot 2');
axis([-.9 .9 -.5 1]);




figure
plot(m.get_history(1,'state_times'), m.get_history(1,'x'),...
m.get_history(2,'state_times'), m.get_history(2,'x'), m.get_history(3,'state_times'), m.get_history(3,'x'));%, m.get_history(4,'state_times'), m.get_history(4,'x'),...
%m.get_history(5,'state_times'), m.get_history(5,'x'), m.get_history(6,'state_times'), m.get_history(6,'x'));
xlabel('time');
ylabel('X-Position');

figure
plot(m.get_history(1,'state_times'), m.get_history(1,'y'),...
m.get_history(2,'state_times'), m.get_history(2,'y'), m.get_history(3,'state_times'), m.get_history(3,'y'));%, m.get_history(4,'state_times'), m.get_history(4,'y'),...
%m.get_history(5,'state_times'), m.get_history(5,'y'), m.get_history(6,'state_times'), m.get_history(6,'y'));
xlabel('time');
ylabel('Y-Position');

w = m.get_history(1,'commands');
figure
plot(m.get_history(1,'state_times'), w(:,2));
xlabel('time');
ylabel('aungular');




