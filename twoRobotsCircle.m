% this script will run the gradient based control law for two robots moving
% in a circle 
clear all

% select initial conditions for the robots, some examples are given here
% two robot setup:
init = [0 -.25 0 -.5; 0 -.75 0 pi-.5];

shape = 'circle';
radius = .5;
% initialize the field object
S = field(length(init(:,1)), shape, radius); % CONSIDER ADDING SHAPE, POLYGON, RUNSPEED
S.twoRobotsCircle();

%S.runspeed = 'fast';
% selects speed of the run, 'slow' computes each robot individually, but is
% susceptible to noise, 'fast' alternates leader robots to speed up the
% program, at the possible expense of accuracy, 'average_fast' runs
% similarly to fast, but uses the average of each rotated position,
% 'average_slow' runs at the slow speed, but sends robots to the average of
% their goal points to protect against noise and jitteriness
S.runTime = 30;

if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);
noise = [0.00 0.00 0 0.000];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', true, 'Sim_noise', noise);
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
    hold on
    angle=0:0.01:2*pi;
    x=S.radius*cos(angle);
    y=S.radius*sin(angle);
    plot(x+S.origin(1),y+S.origin(2));


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
%{
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

entropyList = [entropyList; S.determineEntropy(meas, t(i))];

end
n = entropyList;
figure
plot([0 t], n);
xlabel('time');
ylabel('entropic information');
%}