function coriolis_term = CalculateCoriolisTerm(kinematics)
%CALCULATECORIOLISTERM Summary of this function goes here
%   Detailed explanation goes here
ml_2 = 18.75;
ml_3 = 18.75;
mj_2 = 3.14159;
mj_3 = 7.53982;
l_2 = 0.2;
l_3 = 1;
joint_angles = kinematics(1:3);
q2 = joint_angles(2);
q3 = joint_angles(3);
joint_velocities = kinematics(4:6);
dq1 = joint_velocities(1);
dq2 = joint_velocities(2);
dq3 = joint_velocities(3);

%% All terms
% coriolis_term = [
%     -(dq1*(4*l_2^2*mj_2*sin(2*q2)*dq2 + 4*l_2^2*mj_3*sin(2*q2)*dq2 + l_2^2*ml_2*sin(2*q2)*dq2 + 4*l_2^2*ml_3*sin(2*q2)*dq2 + 4*l_3^2*mj_3*sin(2*q2 + 2*q3)*dq2 + 4*l_3^2*mj_3*sin(2*q2 + 2*q3)*dq3 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq2 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq3 + 8*l_2*l_3*mj_3*dq2*sin(2*q2 + q3) + 4*l_2*l_3*mj_3*dq3*sin(2*q2 + q3) + 4*l_2*l_3*ml_3*dq2*sin(2*q2 + q3) + 2*l_2*l_3*ml_3*dq3*sin(2*q2 + q3) + 4*l_2*l_3*mj_3*sin(q3)*dq3 + 2*l_2*l_3*ml_3*sin(q3)*dq3))/4;
%     (l_3^2*mj_3*sin(2*q2 + 2*q3)*dq1^2)/2 + (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2^2*mj_2*sin(2*q2)*dq1^2)/2 + (l_2^2*mj_3*sin(2*q2)*dq1^2)/2 + (l_2^2*ml_2*sin(2*q2)*dq1^2)/8 + (l_2^2*ml_3*sin(2*q2)*dq1^2)/2 - l_2*l_3*mj_3*sin(q3)*dq3^2 - (l_2*l_3*ml_3*sin(q3)*dq3^2)/2 + l_2*l_3*mj_3*dq1^2*sin(2*q2 + q3) + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/2 - 2*l_2*l_3*mj_3*sin(q3)*dq2*dq3 - l_2*l_3*ml_3*sin(q3)*dq2*dq3;
%     (l_3^2*mj_3*sin(2*q2 + 2*q3)*dq1^2)/2 + (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2*l_3*mj_3*sin(q3)*dq1^2)/2 + l_2*l_3*mj_3*sin(q3)*dq2^2 + (l_2*l_3*ml_3*sin(q3)*dq1^2)/4 + (l_2*l_3*ml_3*sin(q3)*dq2^2)/2 + (l_2*l_3*mj_3*dq1^2*sin(2*q2 + q3))/2 + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/4;
% ];

%% Only links
% coriolis_term = [
%     -(dq1*(l_2^2*ml_2*sin(2*q2)*dq2 + 4*l_2^2*ml_3*sin(2*q2)*dq2 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq2 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq3 + 4*l_2*l_3*ml_3*dq2*sin(2*q2 + q3) + 2*l_2*l_3*ml_3*dq3*sin(2*q2 + q3) + 2*l_2*l_3*ml_3*sin(q3)*dq3))/4;
%     (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2^2*ml_2*sin(2*q2)*dq1^2)/8 + (l_2^2*ml_3*sin(2*q2)*dq1^2)/2 - (l_2*l_3*ml_3*sin(q3)*dq3^2)/2 + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/2 - l_2*l_3*ml_3*sin(q3)*dq2*dq3;
%     (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2*l_3*ml_3*sin(q3)*dq1^2)/4 + (l_2*l_3*ml_3*sin(q3)*dq2^2)/2 + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/4;
% ];

%% Only links improved inertia
%  coriolis_term = [
%      -(dq1*(l_2^2*ml_2*sin(2*q2)*dq2 + 4*l_2^2*ml_3*sin(2*q2)*dq2 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq2 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq3 + 4*l_2*l_3*ml_3*dq2*sin(2*q2 + q3) + 2*l_2*l_3*ml_3*dq3*sin(2*q2 + q3) + 2*l_2*l_3*ml_3*sin(q3)*dq3))/4;
%      (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2^2*ml_2*sin(2*q2)*dq1^2)/8 + (l_2^2*ml_3*sin(2*q2)*dq1^2)/2 - (l_2*l_3*ml_3*sin(q3)*dq3^2)/2 + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/2 - l_2*l_3*ml_3*sin(q3)*dq2*dq3;
%      (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2*l_3*ml_3*sin(q3)*dq1^2)/4 + (l_2*l_3*ml_3*sin(q3)*dq2^2)/2 + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/4;
% ];

%% Full with improved inertia
coriolis_term = [
    -(dq1*(4*l_2^2*mj_2*sin(2*q2)*dq2 + 4*l_2^2*mj_3*sin(2*q2)*dq2 + l_2^2*ml_2*sin(2*q2)*dq2 + 4*l_2^2*ml_3*sin(2*q2)*dq2 + 4*l_3^2*mj_3*sin(2*q2 + 2*q3)*dq2 + 4*l_3^2*mj_3*sin(2*q2 + 2*q3)*dq3 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq2 + l_3^2*ml_3*sin(2*q2 + 2*q3)*dq3 + 8*l_2*l_3*mj_3*dq2*sin(2*q2 + q3) + 4*l_2*l_3*mj_3*dq3*sin(2*q2 + q3) + 4*l_2*l_3*ml_3*dq2*sin(2*q2 + q3) + 2*l_2*l_3*ml_3*dq3*sin(2*q2 + q3) + 4*l_2*l_3*mj_3*sin(q3)*dq3 + 2*l_2*l_3*ml_3*sin(q3)*dq3))/4;
    (l_3^2*mj_3*sin(2*q2 + 2*q3)*dq1^2)/2 + (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2^2*mj_2*sin(2*q2)*dq1^2)/2 + (l_2^2*mj_3*sin(2*q2)*dq1^2)/2 + (l_2^2*ml_2*sin(2*q2)*dq1^2)/8 + (l_2^2*ml_3*sin(2*q2)*dq1^2)/2 - l_2*l_3*mj_3*sin(q3)*dq3^2 - (l_2*l_3*ml_3*sin(q3)*dq3^2)/2 + l_2*l_3*mj_3*dq1^2*sin(2*q2 + q3) + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/2 - 2*l_2*l_3*mj_3*sin(q3)*dq2*dq3 - l_2*l_3*ml_3*sin(q3)*dq2*dq3;
    (l_3^2*mj_3*sin(2*q2 + 2*q3)*dq1^2)/2 + (l_3^2*ml_3*sin(2*q2 + 2*q3)*dq1^2)/8 + (l_2*l_3*mj_3*sin(q3)*dq1^2)/2 + l_2*l_3*mj_3*sin(q3)*dq2^2 + (l_2*l_3*ml_3*sin(q3)*dq1^2)/4 + (l_2*l_3*ml_3*sin(q3)*dq2^2)/2 + (l_2*l_3*mj_3*dq1^2*sin(2*q2 + q3))/2 + (l_2*l_3*ml_3*dq1^2*sin(2*q2 + q3))/4;
];

end

