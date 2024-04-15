function J = CalculateJacobian(joint_angles)
    %CALCULATEJACOBIAN Summary of this function goes here
    %   Detailed explanation goes here
    l1 = 1;
    l2 = 0.2;
    l3 = 1;
    theta1 = joint_angles(1);
    theta2 = joint_angles(2);
    theta3 = joint_angles(3);
    % J = [
    %     -sin(theta1) * (l1 * cos(theta2) + l3 * cos(theta2 + theta3)), -cos(theta1) * (l1 * sin(theta2) + l3 * sin(theta2 + theta3)), -cos(theta1) * l3 * sin(theta2 + theta3);
    %     cos(theta1) * (l1 * cos(theta2) + l3 * cos(theta2 + theta3)), -cos(theta1) * (l1 * sin(theta2) + l3 * sin(theta2 + theta3)), -cos(theta1) * l3 * sin(theta2 + theta3);
    %     l1 * cos(theta1) + l3 * cos(theta2 + theta3), l3 * cos(theta2 + theta3), l3 * cos(theta2 + theta3);
    % ];
    J = zeros(3,3);
    J(1,1) = -sin(theta1) * (l1*cos(theta2) + l3*cos(theta2 + theta3)) - cos(theta1)*l2;
    J(2,1) = cos(theta1) * (l1*cos(theta2) + l3*cos(theta2 + theta3)) - sin(theta1)*l2;
    J(3,1) = 0;
    J(1,2) = -cos(theta1) * (l1*sin(theta2) + l3*sin(theta2 + theta3));
    J(2,2) = -sin(theta1) * (l1*sin(theta2) + l3*sin(theta2 + theta3));
    J(3,2) = l1*cos(theta2) + l3*cos(theta2 + theta3);
    J(1,3) = -cos(theta1)*l3*sin(theta2 + theta3);
    J(2,3) = -sin(theta1)*l3*sin(theta2 + theta3);
    J(3,3) = l3 * cos(theta2 + theta3);
end

