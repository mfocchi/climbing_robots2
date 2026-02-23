function [X, Y, Z] = generateRockWallMap(Lz, Ly, gridSize,wallDepth, maxRidgeDepth,   seed, debug)

if nargin == 2
    gridSize = 100;        % Size of height map
    wallDepth = 2;
    maxRidgeDepth = 0.5;
    debug = true
     rng(47);
end

if nargin ==3
     
    wallDepth = 2;
    maxRidgeDepth = 0.5;
    debug = false
     rng(47);
end


if nargin  ==4
    maxRidgeDepth = 0.5;
    debug = false
     rng(47);
end

if nargin  ==5
    debug = false
     rng(47);
end

if nargin ==6
    debug = false
    rng(seed);
end
if nargin >6
     rng(seed);
end

 
%1) Fractal noise / multi-scale Perlin-like structures for roughness.
frequencies = [1, 2, 4];
weights = [0.5, 0.25, 0.15, 0.1];
% Use convolution to smooth noise (low-pass filter)
smoothKernel = fspecial('gaussian', [15, 15], 3);
for i = 1:length(frequencies)
    scale = frequencies(i);
    % rand(N) returns an N-by-N matrix
    noise = rand(floor(gridSize/scale));
    noise_resized = imresize(noise, [gridSize gridSize], 'bilinear');
    X =  weights(i) * imfilter(noise_resized, smoothKernel, 'replicate');
end
% Normalize and % Scale to realistic wall height (e.g., meters)
X = X - min(X(:));
X =X / max(X(:)) * wallDepth;

% 
%2)  Ridges / dihedrals with directional high gradients.
% Create diagonal ridge (simulating a dihedral)
% [x, y] = meshgrid(1:gridSize, 1:gridSize);
% angle = pi/4;  % Dihedral angle
% ridge = abs(sin(angle)*x + cos(angle)*y - gridSize/2) < 5;
% Z(ridge) = Z(ridge) + 2.6;  % Raise ridge

% Vertical ridge
X(round(gridSize*0.3):15 +round(gridSize*0.3),:) =X(round(gridSize*0.3):15 + round(gridSize*0.3), :) + maxRidgeDepth;

%3) pillars
numPillars = 10;
for i = 1:numPillars
    %randi(100) in MATLAB gives a uniform integer from 1 to 100, inclusive.
    cz = randi(gridSize);
    cy = randi(gridSize);
    [Z, Y] = meshgrid(1:gridSize, 1:gridSize);
    radius = randi([5, 10]);
    bulge = exp(-((Z-cz).^2 + (Y-cy).^2)/(2*(radius^2)));
    X = X + bulge * wallDepth;  % bump height
end

if debug

    figure;
    surf(X);
    shading interp;
    colormap(gray);
    
    axis equal;
    title('Generated Rock Wall Height Map');
    xlabel('Z');
    ylabel('Y');
    zlabel('Height (m)');

end

% Create physical grid in meters
z = linspace(Lz, 0, gridSize);
y = linspace(0, Ly, gridSize);
[Z, Y] = meshgrid(z, y);  % X, Y are in meters


end