function  plot_curve(solution, solution_constr, p0,pf,  mu, plot_energy, color_input, full_update, params)

%make sure is column vector
p0 = p0(:);
pf = pf(:);
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
    min_x = min(min(solution.p(1,:)), pf(1))-3 ;
    max_x = max(max(solution.p(1,:)), pf(1))+3 ;
    min_y = min(min(solution.p(2,:)), pf(2))-3 ;
    max_y = max(max(solution.p(2,:)), pf(2)) +3 ;

    min_z = min(min(p0(3)), min(pf(3)))-4;
    max_z = 2;
    
    
    hold on ; grid on ; axis equal
    set(gca,'CameraPosition',[10   35   10])
    set(gca,'XLim',[min_x max_x])
    set(gca,'YLim',[min_y max_y])
    set(gca,'ZLim',[min_z max_z])
    
    
    %     drawing a wall at X = 0
    p1 = [0 min_y min_z];
    p2 = [0 max_y min_z];
    p3 = [0 max_y max_z];
    p4 = [0 min_y max_z];
    Xw = [p1(1) p2(1) p3(1) p4(1)];
    Yw = [p1(2) p2(2) p3(2) p4(2)];
    Zw = [p1(3) p2(3) p3(3) p4(3)];
    h(3) = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  );

    % half cone
    cone_center = p0';
    cone_size = 3;
    pcone1 = cone_center +[0 0 max_z];
    pcone2 = cone_center +[mu*(cone_size) cone_size max_z];
    pcone3 = cone_center +[mu*(cone_size) cone_size min_z];
    pcone4 = cone_center +[0 0  min_z];
    Xw = [pcone1(1) pcone2(1) pcone3(1) pcone4(1)];
    Yw = [pcone1(2) pcone2(2) pcone3(2) pcone4(2)];
    Zw = [pcone1(3) pcone2(3) pcone3(3) pcone4(3)];
    h(4) = fill3(Xw, Yw, Zw, 'r', 'FaceAlpha',.3  );
    
    % other half cone
    pcone1 = cone_center +[0 0 max_z];
    pcone2 = cone_center +[mu*(cone_size) -cone_size max_z];
    pcone3 = cone_center +[mu*(cone_size) -cone_size min_z];
    pcone4 = cone_center +[0 0  min_z];
    Xw = [pcone1(1) pcone2(1) pcone3(1) pcone4(1)];
    Yw = [pcone1(2) pcone2(2) pcone3(2) pcone4(2)];
    Zw = [pcone1(3) pcone2(3) pcone3(3) pcone4(3)];
    h(5) = fill3(Xw, Yw, Zw, 'r', 'FaceAlpha',.3  );
    
    
end

% actual traj
plot3(solution.p(1,:), solution.p(2,:), solution.p(3,:) ,'Color',color_input ) ;

% discrete traj
plot3(solution_constr.p(1,:), solution_constr.p(2,:), solution_constr.p(3,:) ,'o', 'Color', color_input ) ;

% final point
plot3(pf(1), pf(2), pf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;

%leg inpulse
force_scale = 0.2;
arrow3d_points(p0,p0 + solution.Fleg*force_scale,'color','r');grid on;hold on;

if params.obstacle_avoidance
     plot_ellipsoid(params.obstacle_location,params.obstacle_size(1), params.obstacle_size(2), params.obstacle_size(3),  min_z, max_z, min_y,max_y);
end

grid on;

xlabel('X');
ylabel('Y');
zlabel('Z');
view(33,63);

if (plot_energy)
    figure
    plot(solution.time, solution.energy.Etot)
    title('Plotting the Energy')
    grid on
    xlabel('time');
    ylabel('Energy');
end

end