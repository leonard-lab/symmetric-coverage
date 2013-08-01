% this script will run the gradient based control law for two robots in a
% square
clear all

% select initial conditions for the robots, some examples are given here
% two robot setup:
init = [0 -.25 0 -.5; 0 -.75 0 pi-.5];

shape = 'square';
radius = 1;
% initialize the field object
S = field(length(init(:,1)), shape, radius); % CONSIDER ADDING SHAPE, POLYGON, RUNSPEED

S.runspeed = 'fast';
% selects speed of the run, 'slow' computes each robot individually, but is
% susceptible to noise, 'fast' alternates leader robots to speed up the
% program, at the possible expense of accuracy, 'average_fast' runs
% similarly to fast, but uses the average of each rotated position,
% 'average_slow' runs at the slow speed, but sends robots to the average of
% their goal points to protect against noise and jitteriness
S.runTime = 30;
S.sigma = .25;
S.tau = 2.5;
S.mu = .1;
S.gamma = .2;
S.timeToDelete = 40;
S.k1 = 3;
S.k2 = 1;
S.k3 = 1;
S.origin = [0 -.50 0];
if matlabpool('size') == 0 % checking to see if my pool is already open
    matlabpool open % can do more on computer with more cores
end

% call control law for robot motion
control_law = @(t,x) S.control_law(t,x);
noise = [0.002 0.002 0 0.001];
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

line([-S.radius -S.radius]+[S.origin(1) S.origin(1)], [-S.radius S.radius]+[S.origin(2) S.origin(2)]);
line([-S.radius S.radius]+[S.origin(1) S.origin(1)], [-S.radius -S.radius]+[S.origin(2) S.origin(2)]);
line([S.radius S.radius]+[S.origin(1) S.origin(1)], [-S.radius S.radius]+[S.origin(2) S.origin(2)]);
line([S.radius -S.radius]+[S.origin(1) S.origin(1)], [S.radius S.radius]+[S.origin(2) S.origin(2)]);


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
        meas(mod(j,80)+1,:) = K(j+1,:);
    end
    
    entropyList = [entropyList; S.determineEntropy(meas, t(i),false)];
    
end
n = entropyList;
figure
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
J = strcat('runTime = ',num2str(S.runTime),'; sigma = ',num2str(S.sigma),...
     '; tau = ',num2str(S.tau),'; mu = ', num2str(S.mu),'; gamma = ',num2str(S.gamma),...
     ': timeToDelete = ', num2str(S.timeToDelete),'; k1 = ',num2str(S.k1),'; k2 = ',...
      num2str(S.k2),'; k3 = ', num2str(S.k3),'; origin = ',num2str(S.origin),...
      '; space-time average = ', num2str(S.spacetimeAverage),'; first step',...
      num2str(S.firstStepTime),'; first step speed = ',...
      num2str(S.firstStepSpeed), 'precision = ', num2str(S.precision));

csvwrite('output.csv',M,1,0);
