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

% Step 5: Visualization - Plot all paths
figure;
hold on;
% for k = 2:length(B)
%     boundary = B{k};
%     plot(boundary(:,2), boundary(:,1), 'LineWidth', 2);
% end
plot(path(:,2), path(:,1), 'LineWidth', 2);
hold off;
axis equal;  % Maintain aspect ratio
axis ij;  % Match the image coordinate system
title('Paths for "GATECH"');
xlabel('X');
ylabel('Y');
