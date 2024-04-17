function path=generate_p1_path(radius, origin1, origin2, origin3, samples_per_segment)
    % Step 1: Obtain 3 circles
    circle1 = generate_circle_path(radius, origin1, samples_per_segment);
    circle2 = generate_circle_path(radius, origin2, samples_per_segment);
    circle3 = generate_circle_path(radius, origin3, samples_per_segment);

    % Step 2: Interpolate between circle beginnings
    p0 = circle1(end,:);
    pf = circle2(1,:);
    line1 = generate_line_path(p0, pf, samples_per_segment/2);
    p0 = circle2(end,:);
    pf = circle3(1,:);
    line2 = generate_line_path(p0, pf, samples_per_segment/2);

    % Step 3: Join 3 circles and connecting line segments
    path = [circle1; line1; circle2; line2; circle3];
