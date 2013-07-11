% this script will run the voronoi based control law
clear all
%init = [sqrt(3)/20 -.05 0 0; sqrt(3)/20 .05 0 pi/3; 0 .1 0 2*pi/3; -sqrt(3)/20 .05 0 pi; -sqrt(3)/20 -.05 0 4*pi/3; 0 -.1 0 5*pi/3];
%init = [0 .5 0 0; 0 -.5 0 pi];
init = [sqrt(3)/20 -.05 0 0; 0 .1 0 2*pi/3; -sqrt(3)/20 -.05 0 4*pi/3];
S = field();
%close all
S.shape = 'circle';
%S.polygon = [1 1; -1 1; -1 -1; 1 -1; 1 1];
S.polygon = [1.5 .5*sqrt(3); 0 sqrt(3); -1.5 .5*sqrt(3); -1.5 -.5*sqrt(3); 0 -sqrt(3); 1.5 -.5*sqrt(3); 1.5 .5*sqrt(3)];
%S.runspeed = 'slow';
S.runTime = 30;
if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open 2 % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);
noise = [0.002 0.002 0 0.001];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', true);
m.start


figure
col=hsv(S.n_robots);
for i=1:S.n_robots
    hold on
    plot(m.get_history(i,'x'), m.get_history(i,'y'), 'color', col(i,:));
end
xlabel('X-position');
ylabel('Y-position');
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
    plot(x,y);
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
axis square;



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

w = m.get_history(1,'commands');
d = m.get_history(1,'state_times');
figure
plot(d(2:length(d)), w(:,2));
xlabel('time');
ylabel('angular');



