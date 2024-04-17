function gravity_term = CalculateGravityTerm(joint_angles)
    %CALCULATEGRAVITYTERM Summary of this function goes here
    %   Detailed explanation goes here
    l2 = 1;
    l3 = 1;
    m2 = 18.75;
    m3 = 18.75;
    mj2 = 7.53982;
    mj3 = 4.18879;
    g = 9.8;
    theta2 = joint_angles(2);
    theta3 = joint_angles(3);
    gravity_term = [
        0;
        1/2*l2*cos(theta2)*m2*g + l2*cos(theta2)*m3*g + 1/2*l3*cos(theta2+theta3)*m3*g + l2*cos(theta2)*mj2*g + l2*cos(theta2)*mj3*g + l3*cos(theta2+theta3)*mj3*g;
        1/2*l3*cos(theta2+theta3)*m3*g + l3*cos(theta2+theta3)*mj3*g;
    ];
    gravity_term = -gravity_term;
end

