function [loss, w] = CalculateLoss(q, p_r, k)
    %LOSS Summary of this function goes here
    %   Detailed explanation goes here
    
    %% Calculate position error
    a1 = 0.2;
    a2 = 0.15;
    a3 = 0.12;
    p = [
        a1*cos(q(1)) + a2*cos(q(1)+q(2)) + a3*cos(q(1)+q(2)+q(3));
        a1*sin(q(1)) + a2*sin(q(1)+q(2)) + a3*sin(q(1)+q(2)+q(3))
    ];
    p_error = sqrt(sum((p - p_r).^2));
    
    %% Calculate manipulability error
    J = [
        -a1*sin(q(1)) - a2*sin(q(1)+q(2)) - a3*sin(q(1)+q(2)+q(3)), -a2*sin(q(1)+q(2)) - a3*sin(q(1)+q(2)+q(3)), - a3*sin(q(1)+q(2)+q(3));
        a1*cos(q(1)) + a2*cos(q(1)+q(2)) + a3*cos(q(1)+q(2)+q(3)), a2*cos(q(1)+q(2)) + a3*cos(q(1)+q(2)+q(3)), a3*cos(q(1)+q(2)+q(3));
    ];
    w = sqrt(det(J*J.'));
    
    %% Calculate final loss
    loss = p_error - w*k;
end

