% this script will run the voronoi based control law
clear all

% select initial conditions for the robots, some examples are given here
%init = [sqrt(3)/20 -.05 0 0; sqrt(3)/20 .05 0 pi/3; 0 .1 0 2*pi/3; -sqrt(3)/20 .05 0 pi; -sqrt(3)/20 -.05 0 4*pi/3; 0 -.1 0 5*pi/3];
%init = [0 .5 0 0; 0 -.5 0 pi];
%init = [sqrt(3)/20 -.05 0 0; -sqrt(3)/20 -.05 0 -2*pi/3; 0 .1 0 2*pi/3];
%init = [0 .5 0 .3; .5 0 0 -pi/2 + .3; 0 -.5 0 pi+.3; -.5 0 0 pi/2 + .3];
%init = .5 .* [ 0.5000 0 0 0; 0.3830 -0.3214 0 2*pi/9; 0.0868 -0.4924 0 4*pi/9; 
%    -0.2500 -0.4330 0 6*pi/9; -0.4698 -0.1710 0 8*pi/9; -0.4698 0.1710 0 10*pi/9
%    -0.2500 0.4330 0 12*pi/9; 0.0868 0.4924 0 14*pi/9; 0.3830 0.3214 0 16*pi/9];
a = transpose(0:8);
b = zeros(length(a),1);
init = [.25*cos(2*a*pi/9) -.25*sin(2*a*pi/9) b -2*a*pi/9]

% initialize the field object
S = field(length(init(:,1)));
%close all

% can adjust shape of survey area, default is triangular, with sphere,
% circle, square, and custom being other options
S.shape = 'custom';

% if shape is 'custom' polygon represents the vertices of the shape
%S.polygon = [1 1; -1 1; -1 -1; 1 -1; 1 1];
%S.polygon = S.radius * [1.5 .5*sqrt(3); 0 sqrt(3); -1.5 .5*sqrt(3); -1.5 -.5*sqrt(3); 0 -sqrt(3); 1.5 -.5*sqrt(3); 1.5 .5*sqrt(3)];
S.polygon = [0 1; 1/sqrt(12) .5; sqrt(3)/2 .5; sqrt(3)/3 0; sqrt(3)/2 -.5;
    1/sqrt(12) -.5; 0 -1; -1/sqrt(12) -.5; -sqrt(3)/2 -.5; -sqrt(3)/3 0;
    -sqrt(3)/2 .5; -1/sqrt(12) .5; 0 1];

% selects speed of the run, 'slow' computes each robot individually, but is
% susceptible to noise, 'fast' alternates leader robots to speed up the
% program, at the possible expense of accuracy, 'average_fast' runs
% similarly to fast, but uses the average of each rotated position,
% 'average_slow' runs at the slow speed, but sends robots to the average of
% their goal points to protect against noise and jitteriness
S.runspeed = 'slow';
S.runTime = 10;

if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);
noise = [0.000 0.000 0 0.000];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', true, 'Ts', 0.075, 'Sim_noise', noise);
m.start

%%

%m.shutdown()
%%

figure
col=hsv(S.n_robots);

% plots the position within the shape, and the shape itself
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
%legend('Robot 1', 'Robot 2');
%axis square;


% plots x vs t, y vs t, z vs t, and theta vs t
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
%{
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
%}
%%
%{
% generates a heatmap to show certainty at the end of the run
S.measurements
S.D = inv(S.fieldGen());
t = S.runTime;
B=zeros(41,41);
x = -1:.05:1;
y = -1:.05:1;
parfor i=1:length(x)
    Btemp = zeros(41,41);
    for j=1:length(y)
        Btemp(j,i) = S.timeUncertaintyField(x(i), y(j), 0, S.runTime, S.measurements, S.D);
    end
    B = B + Btemp;
end
HeatMap(1 - B);
%}
      



