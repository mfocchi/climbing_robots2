function val = evalCost(z_query, y_query, params, Fcost)
  %this interpolates to 0 out of the domain which makes gradient cost not smooth!
    % costEvalFcn = @(z_query, y_query) interp2(params.mesh_z, params.mesh_y,  params.cost_x, z_query, y_query, 'linear',0);  
    % val = costEvalFcn(z_query, y_query);


%better implemetation that extrapolates last value out of domain
costEvalFcn = @(z_query, y_query) Fcost(y_query, z_query);

% Example eval
val = costEvalFcn(z_query, y_query);

end