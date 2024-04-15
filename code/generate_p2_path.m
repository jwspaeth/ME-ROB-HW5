function path=generate_p2_path()
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
        path = [path; boundary];
    end
