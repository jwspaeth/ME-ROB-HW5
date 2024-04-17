function path=generate_circle_path(radius, origin, n_samples)
    % Assume that t0, tf are 0, 1.
    % Assume that p0, pf in polar are 0, 2pi
    % Step 1: Generate in polar coordinates
    tf = 1;
    % tt=tf/n_samples:tf/n_samples:tf;
    tt=0:tf/(n_samples-1):tf;
    polar_path = [];
    for i = 1:n_samples
        polar_coord = (2*pi)*tt(1,i);
        polar_path = [polar_path, polar_coord];
    end

    % Step 2: Convert to Cartesian coordinates
    path = [];
    for i = 1:n_samples
        cart_coord = [radius*cos(polar_path(i)), radius*sin(polar_path(i))];
        cart_coord = cart_coord + origin;
        path = [path; cart_coord];
    end