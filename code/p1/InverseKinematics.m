function theta=InverseKinematics(position)
    %% Define set parameters
    la = 1;
    lb = 0.2;
    lc = 1;

    %% Calculate IK
    x = position(1);
    y = position(2);
    z = position(3);

    sign1 = 1;
    sign2 = 1;
    theta1 = atan2(-x, y) + sign1*atan2(sqrt(x^2+y^2-lb^2), lb);
    
    arg1 = sqrt((x^2+y^2+z^2-lb^2+la^2+lc^2)^2 - 2*((x^2 + y^2+z^2-lb^2)^2 + la^4 + lc^4));
    arg2 = x^2 + y^2 + z^2 - lb^2 - la^2 - lc^2;
    theta3 = sign2*atan2(arg1, arg2);
    
    % theta2 = atan2(-z, sqrt(x^2 + y^2 -lb^2)) - atan2(arg1, x^2+y^2+z^2-lb^2+la^2-lc^2)
    arg3 = -z*(lc*cos(theta3)+la) - (cos(theta1)*x + sin(theta1)*y)*lc*sin(theta3);
    arg4 = (cos(theta1)*x + sin(theta1)*y)*(lc*cos(theta3) + la) - z*lc*sin(theta3);
    theta2 = atan2(arg3, arg4);

    theta = [theta1; theta2; theta3];
end