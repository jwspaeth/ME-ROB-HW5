function path=generate_test_path(radius, origin1, samples_per_segment)
    % Obtain 3 circles
    circle1 = generate_circle_path(radius, origin1, samples_per_segment);

    % Join 1 circle
    path = circle1;
