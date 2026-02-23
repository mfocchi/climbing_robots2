function [X, Y, Z] = generateCostMap(Lz, Ly, gridSize, gaussian_center, max_cost)


% Create physical grid in meters
[Z, Y] = meshgrid(1:gridSize, 1:gridSize);
z = linspace(Lz, 0, gridSize);
y = linspace(0, Ly, gridSize);
[Z, Y] = meshgrid(z, y);  % X, Y are in meters

radius = 0.25;
bulge = exp(-((Z-gaussian_center(3)).^2 + (Y-gaussian_center(2)).^2)/(2*(radius^2)));
X = max_cost -bulge*max_cost;  % bump height


end