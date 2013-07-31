% this script will run the gradient based control law
clear all

shape = 'circle';
% can adjust shape of survey area, default is triangular, with sphere,
% circle, square, and custom being other options

radius = .5; % even if custom, try to set to roughly the radius of the shape

runspeed = 'slow';
% selects speed of the run, 'slow' computes each robot individually, but is
% susceptible to noise, 'fast' alternates leader robots to speed up the
% program, at the possible expense of accuracy, 'average_fast' runs
% similarly to fast, but uses the average of each rotated position,
% 'average_slow' runs at the slow speed, but sends robots to the average of
% their goal points to protect against noise and jitteriness

runTime = 30; % time to run Miabots for
sim = true; % whether to run a simulation or run actual Miabots through ROS
noise = [0.0005 0.0005 0 0.0005]; % simulated noise for when sim = true
% select initial conditions for the robots, some examples are given here
% two robot setup:
%init = [0 -.25 0 -.5; 0 -.75 0 pi-.5];

% three robot setup:
init = [sqrt(3)/20 -.55 0 0; -sqrt(3)/20 -.55 0 -2*pi/3; 0 -.4 0 2*pi/3];

%four robot setup:
%init = [0 .5 0 .3; .5 0 0 -pi/2 + .3; 0 -.5 0 pi+.3; -.5 0 0 pi/2 + .3];

% setup of 9 robots for hexagram described in paper
%a = transpose(0:8);
%b = zeros(length(a),1);
%init = [.25*cos(2*a*pi/9) -.25*sin(2*a*pi/9) b -2*a*pi/9];

% initialize the field object
S = streamedField(length(init(:,1)), shape, radius);

%S.runspeed = runspeed;


% if shape is 'custom' polygon represents the vertices of the shape
%S.polygon = [1 1; -1 1; -1 -1; 1 -1; 1 1];
%S.polygon = S.radius * [1.5 .5*sqrt(3); 0 sqrt(3); -1.5 .5*sqrt(3); -1.5 -.5*sqrt(3); 0 -sqrt(3); 1.5 -.5*sqrt(3); 1.5 .5*sqrt(3)];

%S.polygon = 2.*[0 1; 1/sqrt(12) .5; sqrt(3)/2 .5; sqrt(3)/3 0; sqrt(3)/2 -.5;
%    1/sqrt(12) -.5; 0 -1; -1/sqrt(12) -.5; -sqrt(3)/2 -.5; -sqrt(3)/3 0;
%    -sqrt(3)/2 .5; -1/sqrt(12) .5; 0 1];

S.runTime = runTime;

if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);

% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', sim, 'Sim_noise', noise);
m.start

%%

m.shutdown()


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
    line((S.radius .*[-.866 0]) + [S.origin(1) S.origin(1)], (S.radius .*[-.5 1]) + [S.origin(2) S.origin(2)]);
    line((S.radius .*[.866 0]) + [S.origin(1) S.origin(1)], (S.radius .*[-.5 1]) + [S.origin(2) S.origin(2)]);
    line((S.radius .*[-.866 .866]) + [S.origin(1) S.origin(1)], (S.radius .*[-.5 -.5]) + [S.origin(2) S.origin(2)]);
end
if strcmp(S.shape,'circle') == true
    hold on
    angle=0:0.01:2*pi;
    x=S.radius*cos(angle);
    y=S.radius*sin(angle);
    plot(x+S.origin(1),y+S.origin(2));
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
    line([-S.radius -S.radius]+[S.origin(1) S.origin(1)], [-S.radius S.radius]+[S.origin(2) S.origin(2)]);
    line([-S.radius S.radius]+[S.origin(1) S.origin(1)], [-S.radius -S.radius]+[S.origin(2) S.origin(2)]);
    line([S.radius S.radius]+[S.origin(1) S.origin(1)], [-S.radius S.radius]+[S.origin(2) S.origin(2)]);
    line([S.radius -S.radius]+[S.origin(1) S.origin(1)], [S.radius S.radius]+[S.origin(2) S.origin(2)]);
end

if strcmp(S.shape,'custom') == true
    hold on
    plot(S.polygon(:,1),S.polygon(:,2));
end
%legend('Robot 1', 'Robot 2');
%axis square;

%{
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
% generate a graph of the information entropy of the area being surveyed as
% a function of time

t = m.get_history(1,'state_times');

% take the state history
for i=1:S.n_robots
    X(i,:) = m.get_history(i,'x') - S.origin(1);
    Y(i,:) = m.get_history(i,'y') - S.origin(2);
    Z(i,:) = m.get_history(i,'z') - S.origin(3);
end
K = zeros(0,4);
entropyList = 0;
for i=1:length(t)
    for k=1:S.n_robots
        K = [K; X(k,i) Y(k,i) Z(k,i) t(i)];
    end
    
    meas = zeros(0,4);
    % truncate state history
    for j=0:length(K(:,1))-1
        meas(mod(j,60)+1,:) = K(j+1,:);
    end
    
    entropyList = [entropyList; S.determineEntropy(meas, t(i),false)];
    
end
n = entropyList;
figure
plot([0 t], n);
xlabel('time');
ylabel('entropic information');
%}

