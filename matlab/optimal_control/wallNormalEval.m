function normal = wallNormalEval(z_query, y_query, params)
    
%1) compute gradients 
%Now, you need to remember that meshgrid variex x as the SECOND index. I used meshgrid in this because we tend to think of X as the horizontal coordinate, Y as the vertical. But we will need to be careful, as the horizontal coordinate of an array is the second one.
% to compute  the first derivative in the x direction, you can use central differences 
% df/dx = (F(y, x + dx) - F(y , x - dx))/(2*dx)
% df/dy = (F(y + dy, x) - F(y - dy , x))/(2*dy)
% We can achieve that using a convolution, as you tried. The convolution kernel for the x and y gradients would be
% kernelx = [-1 0 1]/(2*dx);
% kernely = [-1;0;1]/(2*dy);
% Be careful, as edge effects will cause problems. conv2 will insert zeros for those terms that fall off the edge of the world. So I would strongly recommend the use of 'valid'.
% pdx = conv2(Z,kernelx,'valid');
% pdy = conv2(Z,kernely,'valid');

% in my casee I have Z in place of X
dz = params.mesh_z(1,2) -  params.mesh_z(1,1);  
dy = params.mesh_y(2,1) -  params.mesh_y(1,1);  
% df/dz = (F(y, z + dz) - F(y , z - dz))/(2*dz);
% df/dy = (F(y + dy, z) - F(y - dy , z))/(2*dy)
kernelz = [-1 0 1]/(2*dz);
kernely = [1;0;-1]/(2*dy);
dx_dz = conv2(params.mesh_x,kernelz,'valid');  
dx_dy = conv2(params.mesh_x,kernely,'valid'); 
% pad properly edges
dx_dz = [conv2(params.mesh_x(:,1:2),[-1 1]/dz,'valid'), dx_dz, conv2(params.mesh_x(:,[end-1,end]),[-1 1]/dz,'valid')];
dx_dy = [conv2(params.mesh_x(1:2,:),[-1; 1]/dy,'valid'); dx_dy; conv2(params.mesh_x([end-1,end],:),[-1; 1]/dy,'valid')];

%surf(dx_dz,params.mesh_y,params.mesh_z)
%define interpolators of gradients
Fz = @(z_query, y_query) interp2(params.mesh_z, params.mesh_y, dx_dz, z_query, y_query, 'linear', 0);
Fy = @(z_query, y_query) interp2(params.mesh_z, params.mesh_y, dx_dy, z_query, y_query, 'linear', 0);


%2) compute normal 
% Query
dz_val = Fz(z_query, y_query);
dy_val = Fy(z_query, y_query);

% Compute normal
normal = [1 -dy_val -dz_val];
normal = normal / norm(normal);
end