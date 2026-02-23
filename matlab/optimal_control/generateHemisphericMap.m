function [X, Y, Z] = generateHemisphericMap(Lz, Ly, cz, cy, radius, grid_size, x_offset)

    % Default arguments (optional behavior similar to Python)
    if nargin < 3, cz = -10; end
    if nargin < 4, cy = 2.5; end
    if nargin < 5, radius = 3; end
    if nargin < 6, grid_size = 100; end
    if nargin < 7, x_offset = 0.01; end

    % Initialize X
    X = zeros(grid_size, grid_size);

    % Create grid
    z = linspace(Lz, 0, grid_size);
    y = linspace(0, Ly, grid_size);
    [Z, Y] = meshgrid(z, y);

    % Compute squared distance from center
    dist2 = (Z - cz).^2 + (Y - cy).^2;

    % Create mask for hemisphere
    mask = dist2 <= radius^2;

    % Compute hemisphere
    hemisphere = zeros(size(X));
    hemisphere(mask) = sqrt(radius^2 - dist2(mask));

    % Add hemisphere to X
    X = X + hemisphere;

    % Add offset
    X = X + x_offset;

    % Assertion
    assert(x_offset ~= 0, ...
        'hemisphere X offset should not be 0 otherwise the dynamics becomes singular');

end