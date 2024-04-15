%% Simplified PUMA class statics
% run this file before running

clear

la=1;
lb=0.2;
lc=1;

%external force and moment at the tip (world frame)
external_force=[0;0;-1];
external_moment=[0;1;0];

%joint torque setting
tau1=0;
tau2=0;
tau3=0;

tf=5; % simulation end time

%% Generate path
radius = 0.1;
origin1 = [0.1, 0];
origin2 = [0.3, 0];
origin3 = [0.5, 0];
samples_per_segment = 100;
path = generate_p1_path(radius, origin1, origin2, origin3, samples_per_segment);
% path = generate_test_path(radius, origin1, samples_per_segment);
path_size = size(path);
N = path_size(1); % Number of points

tt=tf/N:tf/N:tf;
% tt=0:tf/(n_samples-1):tf;
%end point data for simscape
xed = path(:,1);
yed = path(:,2) + 0.5;
xedyedzedsim=[tt' xed yed zeros(length(tt),1)];

%initial angles
% Use IK to predict initial angle of timestep 0
% theta1=45/360*2*pi;
% theta2=-60/360*2*pi;
% theta3=120/360*2*pi;

% theta1=0;
% theta2=0;
% theta3=0;

position = [xed(1); yed(1); 0];
theta = InverseKinematics(position);
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);

%% Set gains
% Ka = 1;
% % kv = 16384;
% kv = 2000000;
% Kv = [
%     kv, 0, 0;
%     0, kv, 0;
%     0, 0, 100;
% ];
% % kp = 256;
% kp = 0;
% kz = 0; % 17;
% Kp = [
%     2000, 0, 0;
%     0, 2000, 0;
%     0, 0, 741.85
% ];
Ka = 1;
% kv = 100000;
kv = 160;
Kv = [
    kv, 0, 0;
    0, kv, 0;
    0, 0, -kv;
];
kp = 8000;
Kp = [
    kp, 0, 0;
    0, kp, 0;
    0, 0, -kp;
];

%% Forward Kinematics
% x = cos(theta1) * (la * cos(theta2) + lc * cos(theta2 + theta3))
% y = sin(theta1) * (la * cos(theta1) + lc * cos(theta2 + theta3))
% z = la * sin(theta1) + lc * sin(theta2 + theta3)

% %% Jacobian
% J = [
%     -sin(theta1) * (la * cos(theta2) + lc * cos(theta2 + theta3)), -cos(theta1) * (la * sin(theta2) + lc * sin(theta2 + theta3)), -cos(theta1) * lc * sin(theta2 + theta3);
%     cos(theta1) * (la * cos(theta2) + lc * cos(theta2 + theta3)), -cos(theta1) * (la * sin(theta2) + lc * sin(theta2 + theta3)), -cos(theta1) * lc * sin(theta2 + theta3);
%     la * cos(theta1) + lc * cos(theta2 + theta3), lc * cos(theta2 + theta3), lc * cos(theta2 + theta3);
% ];
% J_inv = inv(J);

total = xed + yed;
% disp(total)
% disp(max(total))

% plot(xed,yed)