
function F=createInterpolant(value, mesh_y, mesh_z)

  
   % --- Robust griddedInterpolant builder (meshgrid inputs)
mz = mesh_z;
my = mesh_y;
V  = value;

% Basic checks
assert(ismatrix(mz) && ismatrix(my) && ismatrix(V), 'All inputs must be 2D matrices.');

if ~isequal(size(mz), size(my))
    error('mesh_z and mesh_y must have identical sizes (meshgrid outputs).');
end
[ny_mg, nz_mg] = size(mz);

% Recover compact vectors from meshgrid
tol = 1e-12;
is_z_rows_identical = all(all(abs(mz - repmat(mz(1,:), ny_mg, 1)) < tol));
is_y_cols_identical = all(all(abs(my - repmat(my(:,1), 1, nz_mg)) < tol));
if ~(is_z_rows_identical && is_y_cols_identical)
    error('meshgrid structure unexpected: mesh_z rows or mesh_y columns are not consistent.');
end
z_vec = mz(1,:);   % 1 x nz
y_vec = my(:,1);   % ny x 1

% Detect common user mistake: cost_x equals one of the mesh matrices
if isequal(size(V), size(mz)) && ( all(abs(V(:)-mz(:))<tol) || all(abs(V(:)-my(:))<tol) )
    error(['params.cost_x appears to be identical to one of the mesh matrices (mesh_z or mesh_y). ' ...
           'params.cost_x must be the matrix of cost values evaluated on the grid, i.e. ' ...
           'cost_x = f(mesh_z, mesh_y). Please compute the cost matrix before calling this function. ' ...
           'Example: params.cost_x = yourCostFun(mesh_z, mesh_y);']);
end

% Ensure V has orientation [numel(y_vec), numel(z_vec)]
if isequal(size(V), [numel(y_vec), numel(z_vec)])
    Vgrid = V;
elseif isequal(size(V), [numel(z_vec), numel(y_vec)])
    Vgrid = V.'; % transpose if user provided swapped dims
    warning('Transposed params.cost_x to match [ny,nz] orientation.');
else
    error('size(params.cost_x) mismatch. Expected [%d,%d] or [%d,%d], got [%d,%d].', ...
          numel(y_vec), numel(z_vec), numel(z_vec), numel(y_vec), size(V,1), size(V,2));
end

% Build griddedInterpolant with linear extrapolation
F = griddedInterpolant({y_vec, z_vec}, Vgrid, 'linear', 'nearest'); 

end