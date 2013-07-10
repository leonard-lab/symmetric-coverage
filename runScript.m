% this script will run the voronoi based control law
clear all
%init = [sqrt(3)/20 -.05 0 0; 0 .1 0 2*pi/3; -sqrt(3)/20 -.05 0 4*pi/3];% -sqrt(3)/20 .05 0 0; sqrt(3)/20 .05 0 pi/3; 0 -.1 0 5*pi/3];
%init = [0 .5 0 0; 0 -.5 0 pi];
init = [sqrt(3)/20 -.05 0 0; 0 .1 0 2*pi/3; -sqrt(3)/20 -.05 0 4*pi/3];
S = field();
%close all
S.shape = 'custom';
%S.polygon = [1 1; -1 1; -1 -1; 1 -1; 1 1];
S.polygon = [1.5 .5*sqrt(3); 0 sqrt(3); -1.5 .5*sqrt(3); -1.5 -.5*sqrt(3); 0 -sqrt(3); 1.5 -.5*sqrt(3); 1.5 .5*sqrt(3)];
%S.runspeed = 'slow';
S.runTime = 50;
if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open 2 % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);
noise = [0.0005 0.0005 0 0.0003];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', true, 'sim_noise', noise);
m.start


figure
plot(m.get_history(1,'x'), m.get_history(1,'y'),...
    m.get_history(2,'x'), m.get_history(2,'y'), m.get_history(3,'x'), m.get_history(3,'y'));%, m.get_history(4,'x'), m.get_history(4,'y'),...
%m.get_history(5,'x'), m.get_history(5,'y'), m.get_history(6,'x'), m.get_history(6,'y')); %, A(:,1), A(:,2), B(:,1), B(:,2), C(:,1), C(:,2));
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
d = m.get_history(1,'state_times');
figure
plot(d(2:length(d)), w(:,2));
xlabel('time');
ylabel('angular');



