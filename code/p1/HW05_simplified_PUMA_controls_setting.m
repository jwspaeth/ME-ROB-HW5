%% Simplified PUMA class statics
% run this file before runnig

clear

la=1;
lb=0.2;
lc=1;

%external force and moment at the tip (world frame)
external_force=[0;0;-1];
external_moment=[0;1;0];

%initial angles
theta1=45/360*2*pi;
theta2=-60/360*2*pi;
theta3=120/360*2*pi;


%joint torque setting
tau1=0;
tau2=0;
tau3=0;

%xedyedzedsim=[0.0 0.5 0.2 0.0];

tf=10; % simulation end time

N=70; %number of points

figure(2)
axis([0 1 0 1])
grid
hold on
[xed,yed]=ginput(N);
tt=tf/N:tf/N:tf;
%end point data for simscape
xedyedzedsim=[tt' xed yed zeros(length(tt),1)];
disp(size(xedyedzedsim))
plot(xed,yed)
%axis([-1 1 -1 1])
