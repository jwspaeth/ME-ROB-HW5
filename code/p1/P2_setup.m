%% Simplified PUMA class statics
% run this file before running

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

tf=10; % simulation end time

samples_per_segment = 1000;
path = generate_p2_path(samples_per_segment);
disp(size(path))
path_size = size(path);
N = path_size(1); % Number of points
tt=tf/N:tf/N:tf;
% tt=0:tf/(n_samples-1):tf;

%end point data for simscape
xed = path(:,2);
xed = xed/200 / 2;
xed = xed + 0.2;
yed = -1 * path(:,1);
yed = yed + abs(mean(yed));
yed = yed / 20 / 6;
yed = yed + 0.4;
xedyedzedsim=[tt' xed yed zeros(length(tt),1)];

% Gains
% Ka = 0;
% Kv = 250;
% ev limit = [0.5, -0.5];
% Kp = 8000;
% Ki = 1000;


%initial angles
% Use IK to predict initial angle of timestep 0
position = [xed(1); yed(1); 0];
theta = InverseKinematics(position);
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);

% plot(xed,yed)