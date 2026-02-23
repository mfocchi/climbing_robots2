function  plot_curve(solution, solution_constr, p0,landing_patch_center,  mu, plot_energy, color_input, full_update, params)

%make sure is column vector
p0 = p0(:);
landing_patch_center = landing_patch_center(:);
if full_update
    % anchor1  line
    plot3([params.p_a1(1) p0(1)],[params.p_a1(2) p0(2)],[params.p_a1(3) p0(3)],'k--');   hold on ;
    %anchor 1
    plot3(params.p_a1(1),params.p_a1(2),params.p_a1(3),'Marker','*','Color','k','MarkerSize',10);
    % anchor2  line
    plot3([params.p_a2(1) p0(1)],[params.p_a2(2) p0(2)],[params.p_a2(3) p0(3)],'k--');   hold on ;
    %anchor 1
    plot3(params.p_a2(1),params.p_a2(2),params.p_a2(3),'Marker','*','Color','k','MarkerSize',10);
        
    
    %initial
    plot3(p0(1), p0(2), p0(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
    
    % Min-max axis
    min_x = min(min(solution.p(1,:)), landing_patch_center(1))-3 ;
    max_x = max(max(solution.p(1,:)), landing_patch_center(1))+3 ;
    min_y = min(min(solution.p(2,:)), landing_patch_center(2))-3 ;
    max_y = params.p_a2(2);

    min_z = min(min(p0(3)), min(landing_patch_center(3)))-4;
    max_z = 2;
    
    
    hold on ; grid on ; axis equal
    set(gca,'CameraPosition',[10   35   10])
    set(gca,'XLim',[min_x max_x])
    set(gca,'YLim',[min_y max_y])
    set(gca,'ZLim',[min_z max_z])
    
    
    

    % % half cone
    % cone_center = p0';
    % cone_size = 3;
    % pcone1 = cone_center +[0 0 max_z];
    % pcone2 = cone_center +[mu*(cone_size) cone_size max_z];
    % pcone3 = cone_center +[mu*(cone_size) cone_size min_z];
    % pcone4 = cone_center +[0 0  min_z];
    % Xw = [pcone1(1) pcone2(1) pcone3(1) pcone4(1)];
    % Yw = [pcone1(2) pcone2(2) pcone3(2) pcone4(2)];
    % Zw = [pcone1(3) pcone2(3) pcone3(3) pcone4(3)];
    % h(4) = fill3(Xw, Yw, Zw, 'r', 'FaceAlpha',.3  );
    % 
    % % other half cone
    % pcone1 = cone_center +[0 0 max_z];
    % pcone2 = cone_center +[mu*(cone_size) -cone_size max_z];
    % pcone3 = cone_center +[mu*(cone_size) -cone_size min_z];
    % pcone4 = cone_center +[0 0  min_z];
    % Xw = [pcone1(1) pcone2(1) pcone3(1) pcone4(1)];
    % Yw = [pcone1(2) pcone2(2) pcone3(2) pcone4(2)];
    % Zw = [pcone1(3) pcone2(3) pcone3(3) pcone4(3)];
    % h(5) = fill3(Xw, Yw, Zw, 'r', 'FaceAlpha',.3  );
    
    
end

%plot patch
plot_patch(landing_patch_center,  params);


% actual traj
plot3(solution.p(1,:), solution.p(2,:), solution.p(3,:) ,'Color',color_input ) ;

% discrete traj
plot3(solution_constr.p(1,:), solution_constr.p(2,:), solution_constr.p(3,:) ,'o', 'Color', color_input ) ;

% landing_patch_center 
plot3(landing_patch_center(1), landing_patch_center(2), landing_patch_center(3), 'Marker', '.', 'Color','g', 'MarkerSize',40) ;

%leg inpulse
force_scale = 0.2;
arrow3d_points(p0,p0 + solution.Fleg*force_scale,'color','r');grid on;hold on;

if strcmp(params.obstacle_avoidance,'mesh')
    h=surf(params.mesh_x, params.mesh_y, params.mesh_z);
    %plot normal at p0 (to properly visualize it as perpendicular to
    %surface you need to set axis equal!
    arrow3d_points(p0,p0 + params.contact_normal(:)*2,'color','k');grid on;hold on;
    %surfnorm(params.mesh_x, params.mesh_y, params.mesh_z); % plots the     %normals (negative)
    % I plot the opposite of the cost to visualize it in the map
    h1=surf(params.cost_x/5, params.cost_y, params.cost_z, 'FaceAlpha', 0.5);

end    

grid on;

xlabel('X');
ylabel('Y');
zlabel('Z');
view(147,8.6);

if (plot_energy)
    figure
    plot(solution.time, solution.energy.Etot)
    title('Plotting the Energy')
    grid on
    xlabel('time');
    ylabel('Energy');
end

end