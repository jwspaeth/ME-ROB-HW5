function path=generate_line_path(p0, pf, n_samples)
    % Assume that t0, tf are 0, 1.
    % Assume that p0, pf in polar are 0, 2pi
    % Step 1: Generate in polar coordinates
    tf = 1;
    % tt=tf/n_samples:tf/n_samples:tf;
    tt=0:tf/(n_samples-1):tf;
    path = [];
    for i = 1:n_samples
        cart_coord = (pf-p0)*tt(1,i) + p0;
        path = [path; cart_coord];
    end