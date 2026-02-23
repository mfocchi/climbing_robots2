close all

 wallDepth = 1; %how              
gridSize = 100;
maxRidgeDepth = 0.5;
seed= 47;

[params.mesh_x , params.mesh_y, params.mesh_z] = generateRockWallMap(-20, 5, gridSize, wallDepth,maxRidgeDepth, seed, false);
Fmesh=createInterpolant(params.mesh_x, params.mesh_y, params.mesh_z);

[Z, Y] = meshgrid(1:300, 1:300);
z = linspace(-40, 0, 300);
y = linspace(0, 40, 300);


[Z, Y] = meshgrid(z, y);  % X, Y are in meters

wall_x = wallSurfaceEval(Z, Y,params, Fmesh)
figure
h1=surf(wall_x, Y,Z, 'FaceAlpha', 0.5);

grid on;

xlabel('X');
ylabel('Y');
zlabel('Z');
view(147,8.6);
xlim([0 10])
ylim([0 40])
zlim([-40 0])