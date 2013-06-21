% this script will run the voronoi based control law
clear all
S = field();
close all
S.runTime = 10;
            
% call control law for robot motion
control_law = @(t,x) S.voronoi_control_law(t,x); 
            
% calls new Miabot object that actuates robot motion
m = Miabots([.1 .5 0 0; -.3 .3 0 0; .8 .7 0 0; .9 0 0 0], control_law, 'velocity', S.runTime,...
    'sim', true);
m.start
            
% plots the resulting path of the two robots against the ideal
figure
plot(m.get_history(1,'x'), m.get_history(1,'y'),...
m.get_history(2,'x'), m.get_history(2,'y'), m.get_history(3,'x'), m.get_history(3,'y'), m.get_history(4,'x'), m.get_history(4,'y'));
xlabel('X-position');
ylabel('Y-position');
legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4');
axis([-1 1 -1 1]);
figure
plot(m.get_history(1,'state_times'), m.get_history(1,'x'),...
m.get_history(2,'state_times'), m.get_history(2,'x'), m.get_history(3,'state_times'), m.get_history(3,'x'), m.get_history(4,'state_times'), m.get_history(4,'x'));

figure
plot(m.get_history(1,'state_times'), m.get_history(1,'y'),...
m.get_history(2,'state_times'), m.get_history(2,'y'), m.get_history(3,'state_times'), m.get_history(3,'y'), m.get_history(4,'state_times'), m.get_history(4,'y'));
            

