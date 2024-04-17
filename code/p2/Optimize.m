function [best_q, best_w] = Optimize(q_ranges, p_r,k,n_samples)
    %OPTIMIZE Summary of this function goes here
    %   Detailed explanation goes here

    % Create sample grid
    q1 = q_ranges(1,1):q_ranges(1,2)/(n_samples-1):q_ranges(1,2);
    q1 = q1.';
    q2 = q_ranges(2,1):q_ranges(2,2)/(n_samples-1):q_ranges(2,2);
    q2 = q2.';
    q3 = q_ranges(3,1):q_ranges(3,2)/(n_samples-1):q_ranges(3,2);
    q3 = q3.';
    qs = zeros(n_samples^3, 3);
    count = 1;
    for i = 1:n_samples
        for j = 1:n_samples
            for l = 1:n_samples
                qs(count,:) = [
                    q1(i), q2(j), q3(l);
                ];
                count = count + 1;
            end
        end
    end

    % Evaluate sample grid and keep best (lowest) loss
    best_loss = 1000000;
    best_w = 0;
    best_q = qs(1);
    total = n_samples^3;
    for i = 1:total
        q = qs(i,:);
        [loss, w] = CalculateLoss(q, p_r, k);
        if loss < best_loss
            best_loss = loss;
            best_w = w;
            best_q = q;
        end
    end

end

