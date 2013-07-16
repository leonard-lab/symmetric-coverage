% this script will run the voronoi based control law
clear all
%init = [sqrt(3)/20 -.05 0 0; sqrt(3)/20 .05 0 pi/3; 0 .1 0 2*pi/3; -sqrt(3)/20 .05 0 pi; -sqrt(3)/20 -.05 0 4*pi/3; 0 -.1 0 5*pi/3];
%init = [0 .5 0 0; 0 -.5 0 pi];
init = [sqrt(3)/20 -.05 0 0; -sqrt(3)/20 -.05 0 -2*pi/3; 0 .1 0 2*pi/3];
%init = [0 .5 0 .3; .5 0 0 3*pi/2 + .3; 0 -.5 0 pi+.3; -.5 0 0 pi/2 + .3];
%init = .5 .* [ 0.5000 0 0 0; 0.3830 -0.3214 0 2*pi/9; 0.0868 -0.4924 0 4*pi/9; 
%    -0.2500 -0.4330 0 6*pi/9; -0.4698 -0.1710 0 8*pi/9; -0.4698 0.1710 0 10*pi/9
%    -0.2500 0.4330 0 12*pi/9; 0.0868 0.4924 0 14*pi/9; 0.3830 0.3214 0 16*pi/9];
S = field(length(init(:,1)));
%close all
S.shape = 'sphere';
%S.polygon = [1 1; -1 1; -1 -1; 1 -1; 1 1];
%S.polygon = S.radius * [1.5 .5*sqrt(3); 0 sqrt(3); -1.5 .5*sqrt(3); -1.5 -.5*sqrt(3); 0 -sqrt(3); 1.5 -.5*sqrt(3); 1.5 .5*sqrt(3)];
%S.polygon = [0 1; 1/sqrt(12) .5; sqrt(3)/2 .5; sqrt(3)/3 0; sqrt(3)/2 -.5;
%    1/sqrt(12) -.5; 0 -1; -1/sqrt(12) -.5; -sqrt(3)/2 -.5; -sqrt(3)/3 0;
%    -sqrt(3)/2 .5; -1/sqrt(12) .5; 0 1];
S.runspeed = 'slow';
S.runTime = 2;
if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open 2 % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.voronoi_control_law(t,x);
noise = [0.001 0.001 0 0.0005];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'waypoint', S.runTime,...
    'sim', true, 'Ts', 0.075);
m.start

%%

%m.shutdown()
%%

figure
col=hsv(S.n_robots);
for i=1:S.n_robots
m.shutdown()
    hold on
    plot3(m.get_history(i,'x'), m.get_history(i,'y'), m.get_history(i,'z'), 'color', col(i,:));
end
xlabel('X-position');
ylabel('Y-position');
zlabel('Z-position');
if strcmp(S.shape,'triangle') == true
    line([-.866 0], [-.5 1]);
    line([.866 0], [-.5 1]);
    line([-.866 .866], [-.5 -.5]);
end
if strcmp(S.shape,'circle') == true
    hold on
    angle=0:0.01:2*pi;
    x=S.radius*cos(angle);
    y=S.radius*sin(angle);
    plot3(x,y);

    
end

if strcmp(S.shape,'sphere') == true
    hold on
    angle=0:0.1:2*pi;
    x=S.radius*cos(angle);
    y=S.radius*sin(angle);
    plot3(x,y,zeros(length(x)));
    hold on
    plot3(zeros(length(x)),x,y);
    hold on
    plot3(y,zeros(length(x)),x);
end
if strcmp(S.shape,'square') == true
    line([-S.radius -S.radius], [-S.radius S.radius]);
    line([-S.radius S.radius], [-S.radius -S.radius]);
    line([S.radius S.radius], [-S.radius S.radius]);
    line([S.radius -S.radius], [S.radius S.radius]);
end
if strcmp(S.shape,'custom') == true
    hold on
    plot(S.polygon(:,1),S.polygon(:,2));
end
legend('Robot 1', 'Robot 2');



figure
for i=1:S.n_robots
    hold on
    plot(m.get_history(i,'state_times'), m.get_history(i,'x'), 'color', col(i,:));
end
xlabel('time');
ylabel('X-Position');

figure
for i=1:S.n_robots
    hold on
    plot(m.get_history(i,'state_times'), m.get_history(i,'y'), 'color', col(i,:));
end
xlabel('time');
ylabel('Y-Position');

figure
for i=1:S.n_robots
    hold on
    plot(m.get_history(i,'state_times'), m.get_history(i,'z'), 'color', col(i,:));
end
xlabel('time');
ylabel('Z-Position');

w = m.get_history(1,'commands');
d = m.get_history(1,'state_times');
figure
plot(d(2:length(d)), w(:,2));
xlabel('time');
ylabel('angular');