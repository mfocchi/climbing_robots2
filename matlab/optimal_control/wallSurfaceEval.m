function val = wallSurfaceEval(z_query, y_query,params, Fmesh)
    
    if nargin <4
        Fmesh=createInterpolant(params.mesh_x, params.mesh_y, params.mesh_z);
    end
    % In theory I should use this that extrapolates the mesh out of domain
    % but works worse for some reason
    %better implemetation that extrapolates last value out of domain
    wallSurfaceFcn = @(z_query, y_query) Fmesh(y_query, z_query);

    % this extrapolates out of the domain, the gradient is not defined 
    %wallSurfaceFcn = @(z_query, y_query) interp2(params.mesh_z, params.mesh_y,  params.mesh_x, z_query, y_query, 'linear', 0);
    val = wallSurfaceFcn(z_query, y_query);
end