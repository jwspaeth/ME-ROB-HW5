function path=generate_p2_path(samples_per_segment)
    % Step 1: Create an Image of the String "GATECH"
    figure('visible', 'off', 'Position', [0 0 300 100]);  % Adjusted figure size for longer text
    text('String', 'GATECH', 'FontSize', 32, 'Units', 'Normalized', 'Position', [0.5 0.5], 'HorizontalAlignment', 'Center', 'FontName', 'Arial');
    F = getframe(gca);
    img = frame2im(F);
    close;
    
    % Step 2: Convert to Grayscale and Binarize
    grayImg = rgb2gray(img);
    binaryImg = imbinarize(grayImg);
    
    % Step 3: Edge Detection and Thinning
    edges = edge(binaryImg, 'Canny');
    thinnedEdges = bwmorph(edges, 'thin', Inf);
    
    % Step 4: Extract the Paths
    [B, ~] = bwboundaries(thinnedEdges, 'noholes');
    
    % Step 5: Join all paths
    path = [];
    for k = 2:length(B)
        boundary = B{k};
        % Interpolate to desired samples per segment
        b_size = size(boundary);
        N = b_size(1);
        ttq = 1:N/samples_per_segment:N;
        ttq = ttq.';
        x = boundary(:,1);
        x = smooth(x);
        xq = interp1(x, ttq);
        y = boundary(:,2);
        y = smooth(y);
        yq = interp1(y, ttq);
        boundary = [xq, yq];
        if k > 2
            line = generate_line_path(prev, boundary(1,:), samples_per_segment/2);
            path = [path; line];
        end
        path = [path; boundary];
        prev = boundary(end,:);
    end
    path(:,1) = smooth(path(:,1));
    path(:,2) = smooth(path(:,2));
