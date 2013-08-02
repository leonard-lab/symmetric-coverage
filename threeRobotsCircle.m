% this script will run the voronoi based control law
clear all

% three robot setup:
init = [sqrt(3)/20 -.45 0 .3; -sqrt(3)/20 -.45 0 -2*pi/3+.3; 0 -.3 0 2*pi/3+.3];

% can adjust shape of survey area, default is triangular, with sphere,
% circle, square, and custom being other options
shape = 'circle';
radius = .4;

% initialize the field object
S = streamedField(length(init(:,1)), shape, radius); % CONSIDER ADDING SHAPE, POLYGON, RUNSPEED

S.sigma = .2;        % time constant for spatial separation of measurements
S.tau = .3;          % time constant for temporal separation of measurements
S.mu = .1;          % uncertainty in measurements, a characteristic of the sensors
S.gamma = .04;      % radius over which a gradient is determined for motion
S.timeToDeleteSelf = 3; % number of time steps after which a robot deletes its own old positions
S.timeToDeleteOther = 1; % number of time steps after which a robot deletes the other's old positions
S.k1 = 1;          % coefficient for forward velocity in control law
S.k2 = 1;          % coefficient for angular velocity in control law
S.k3 = 1;          % coefficient for z velocity in control law
S.origin = [0 -0.4 0];% movable center which is treated as the origin

%S.runspeed = 'fast'; % used by field, but not streamedField
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
noise = [0.000 0.000 0 0.000];
% calls new Miabot object that actuates robot motion
m = Miabots(init, control_law, 'velocity', S.runTime,...
    'sim', true);
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
figure
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
        meas(mod(j,40)+1,:) = K(j+1,:);
    end

entropyList = [entropyList; S.determineEntropy(meas, t(i),false)];

end
n = entropyList;

plot([0 t], n);
xlabel('time');
ylabel('entropic information');
%}
%%
% records a string of all inputted variables, and generates a csv file
% containing the data from the run
p1 = transpose(m.get_history(1,'state_times'));
M = zeros(length(p1), 9, S.n_robots);
for i=1:S.n_robots
    
    p2 = transpose(m.get_history(i,'x'));
    p3 = transpose(m.get_history(i,'y'));
    p4 = transpose(m.get_history(i,'z'));
    p5 = transpose(m.get_history(i,'vx'));
    p6 = transpose(m.get_history(i,'vz'));
    p7 = transpose(m.get_history(i,'theta'));
    p8 = transpose(m.get_history(i,'theta_dot'));
    p9 = n(2:length(n));
M(:,:,i) = [p1 p2 p3 p4 p5 p6 p7 p8 p9];
end
J = S.settings();
csvwrite('output.csv',M,1,0);