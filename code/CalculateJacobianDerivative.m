function dJ = CalculateJacobianDerivative(kinematics)
    %CALCULATEJACOBIAN Summary of this function goes here
    %   Detailed explanation goes here
    l1 = 1;
    l2 = 0.2;
    l3 = 1;
    joint_angles = kinematics(1:3);
    theta1 = joint_angles(1);
    theta2 = joint_angles(2);
    theta3 = joint_angles(3);
    joint_velocities = kinematics(4:6);
    dtheta1 = joint_velocities(1);
    dtheta2 = joint_velocities(2);
    dtheta3 = joint_velocities(3);
    % J = [
    %     -sin(theta1) * (l1 * cos(theta2) + l3 * cos(theta2 + theta3)), -cos(theta1) * (l1 * sin(theta2) + l3 * sin(theta2 + theta3)), -cos(theta1) * l3 * sin(theta2 + theta3);
    %     cos(theta1) * (l1 * cos(theta2) + l3 * cos(theta2 + theta3)), -cos(theta1) * (l1 * sin(theta2) + l3 * sin(theta2 + theta3)), -cos(theta1) * l3 * sin(theta2 + theta3);
    %     l1 * cos(theta1) + l3 * cos(theta2 + theta3), l3 * cos(theta2 + theta3), l3 * cos(theta2 + theta3);
    % ];
    dJ = zeros(3,3);
    dJ(1,1) = -cos(theta1)*l1*cos(theta2)*dtheta1 + sin(theta1)*l1*sin(theta2)*dtheta2 - cos(theta1)*l3*cos(theta2+theta3)*dtheta1 + sin(theta1)*l3*sin(theta2 + theta3)*(dtheta1 + dtheta2) + l2*sin(theta1)*dtheta1;
    dJ(2,1) = -sin(theta1)*l1*cos(theta2)*dtheta1 - cos(theta1)*l1*sin(theta2)*dtheta2 - sin(theta1)*l3*cos(theta2+theta3)*dtheta1 - cos(theta1)*l3*sin(theta2 + theta3)*(dtheta1 + dtheta2) - l2*cos(theta1)*dtheta1;
    dJ(3,1) = 0;
    dJ(1,2) = sin(theta1)*l1*sin(theta2)*dtheta1 - cos(theta1)*l1*cos(theta2)*dtheta2 + sin(theta1)*l3*sin(theta2+theta3)*dtheta1 - cos(theta1)*l3*cos(theta2+theta3)*(dtheta2+dtheta3);
    dJ(2,2) = -cos(theta1)*l1*sin(theta2)*dtheta1 - sin(theta1)*l1*cos(theta2)*dtheta2 - cos(theta1)*l3*sin(theta2+theta3)*dtheta1 - sin(theta1)*l3*cos(theta2+theta3)*(dtheta2+dtheta3);
    dJ(3,2) = -l1*sin(theta2)*dtheta2 - l3*sin(theta2+theta3)*(dtheta2+dtheta3);
    dJ(1,3) = sin(theta1)*l3*sin(theta2+theta3)*dtheta1 - cos(theta1)*l3*cos(theta2+theta3)*(dtheta2+dtheta3);
    dJ(2,3) = -cos(theta1)*l3*sin(theta2+theta3)*dtheta1 - sin(theta1)*l3*cos(theta2+theta3)*(dtheta2+dtheta3);
    dJ(3,3) = -l3 * sin(theta2 + theta3)*(dtheta1 + dtheta2);
end

