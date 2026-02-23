
close all 
clear all
clc

anchor_distance = 10;
p_anchor1 = [0;0;0]
p_anchor2 = [0;anchor_distance;0]

% x = [fl1, fl2, fr1, fr2]
baseline = 0.8;
wall_clearance = 0.4;

mu = 0.8
g_vec = [0;0;-9.81]
mass = 5
force_scale =0.1
min_feet_force = 0;
% margin out of the vertical
margin = 0.31;
steps=  0.5*anchor_distance + [0, 3, 0.5*anchor_distance+ margin]
figure

for i=1:length(steps)
    % base pos
    p_base = [wall_clearance; steps(i); -5];

    %old model base frame orientation
    % [theta, phi, l] = computePolarVariables(p_base);
    % wRb =   [cos(phi)*sin(theta), -sin(phi), cos(phi)*cos(theta) 
    %          sin(phi)*sin(theta),  cos(phi), cos(theta)*sin(phi) 
    %          -cos(theta),         0,          sin(theta)        ]

    [psi] = computeTheta(p_base);
    %new model base frame (psi is associated to the rope plane so we have
    %a rotation of (pi/2 -psi) about the y axis
    wRb =[ cos(pi/2-psi), 0, sin(pi/2-psi),
               0, 1,          0, 
            -sin(pi/2-psi), 0, cos(pi/2-psi)]
        
    pf1 = [0; -baseline/2; -wall_clearance];
    pf2 = [0; baseline/2; -wall_clearance];

    % hoist positions
    w_pf1 = wRb*pf1 +p_base;
    w_pf2 = wRb*pf2 +p_base; 

    % line of actions of the anchor forces (univ vectors)
    w_pr1  = (p_base-p_anchor1)/norm(p_base-p_anchor1);
    w_pr2  = (p_base-p_anchor2)/norm(p_base-p_anchor2);

 
    % moment equilibrium  constraints about base origin(3)
    J = [cross_mx(w_pf1 - p_base) cross_mx(w_pf2 - p_base) zeros(3,1) zeros(3,1)] ;
    beq2 = zeros(3,1);

    % stack equality constgraints
    Aeq = [Aeq1;Aeq2];
    beq = [beq1;beq2];

    %friction cone matrix
    %contact frame
    n = [1;0;0];   
    t1 = cross(n, [0;1;0]);
    t2 = cross( [0;0;1], n);
    
    %friction cone pyramid
    F = [(t1-mu*n)';
         (-t1-mu*n)';
         (t2-mu*n)';
         (-t2-mu*n)'];

    % inequalities friction cones constraints + min normal force + 
    % unilaterality of ropes  fr1 < 0 
    % forces
    C = blkdiag([F ;-n'],[F; -n'], 1,1);
    d = [zeros(4,1);-min_feet_force; zeros(4,1);-min_feet_force;0;0]

    % only unilateral on feet and rope used for optim
%     C = blkdiag(-n',-n', 1,1);
%     d = [-min_feet_force; -min_feet_force;0;0]

    [x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, C, d, Aeq, beq);
    %[x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, [], [], Aeq, beq);
    fl1 = x(1:3)
    fl2 = x(4:6)
    fr1 = x(7)
    fr2 = x(8)

    if EXITFLAG == -2

        disp('no solution found')
    end

    margins = C*x - d;
    S_n1 = [zeros(1,4) 1 zeros(1,5) 0 0] ;
    S_n2 = [zeros(1,5) zeros(1,4) 1 0 0] ;
    S_F1 = blkdiag(eye(4), zeros(1+5+1+1,1+5+1+1 ));
    S_F2 = blkdiag(zeros(5,5), eye(4),zeros(1+1+1,1+1+1) );

    normal1_viol = S_n1*margins > 0;
    normal2_viol = S_n2*margins > 0;
    F1_viol = any(S_F1*margins  > 0);
    F2_viol = any(S_F2*margins > 0);
    if normal1_viol 
        disp('normal 1 violation')
    end
    if normal2_viol 
        disp('normal 2 violation')
    end
    if F1_viol 
        disp('friction 1 violation')
    end
    if F2_viol 
        disp('friction 2 violation')
    end


    
    
    %plot
    % fl1 = [1; 0;0];
    % fl2 = [1; 0;0];
    % fr1 = 1;
    % fr2 = 1;

    min_z = -7;
    max_z = 1;
    min_y = p_anchor1(2)-2;
    max_y = p_anchor2(2)+2;
    
    % for paper 
    %clf(gcf)
    set(0, 'DefaultAxesBox', 'on');
    set(0, 'DefaultTextFontSize', 30);
    set(0, 'DefaultAxesFontSize', 30);
    set(0, 'DefaultUicontrolFontSize', 30);
    
    %     drawing a wall at X = 0
    p1 = [0 min_y min_z];
    p2 = [0 max_y min_z];
    p3 = [0 max_y max_z];
    p4 = [0 min_y max_z];
    Xw = [p1(1) p2(1) p3(1) p4(1)];
    Yw = [p1(2) p2(2) p3(2) p4(2)];
    Zw = [p1(3) p2(3) p3(3) p4(3)];
    h = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  ); 
    fill3([0.001,0.001,0.001,0.001] , [p_anchor1(2)-margin,p_anchor1(2)-margin , p_anchor2(2)+margin,  p_anchor2(2)+margin], [-7,0, 0, -7],'r', 'FaceAlpha',.5, 'EdgeColor','none'  );



    % plot world
    plot3(0,0,0,'.k', 'MarkerSize',40);grid on;hold on;
    % plot anchors
    plot3(p_anchor1(1),p_anchor1(2),p_anchor1(3),'.m', 'MarkerSize',60);grid on;hold on;
    plot3(p_anchor2(1),p_anchor2(2),p_anchor2(3),'.y', 'MarkerSize',60);grid on;hold on;
    % plot feet pos
    plot3(w_pf1(1), w_pf1(2),w_pf1(3),'.b', 'MarkerSize',40);hold on;
    plot3(w_pf2(1), w_pf2(2),w_pf2(3),'.b', 'MarkerSize',40);hold on;
    % plot base pos
    plot3(p_base(1), p_base(2),p_base(3),'.g', 'MarkerSize',40);hold on;
    xlabel('$X$','interpreter', 'latex');
    ylabel('$Y$','interpreter', 'latex');
    zlabel('$Z$','interpreter', 'latex');
    
    %plot rope forces
    arrow3d_points(p_base,p_base+w_pr1 * fr1*force_scale,'color','m');grid on;hold on;
    arrow3d_points(p_base,p_base+w_pr2 * fr2*force_scale,'color','y');grid on;hold on;
    %plot feet forces
    if normal1_viol || F1_viol
        arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','k');grid on;hold on;
    else
        arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','b');grid on;hold on;
    end

    if normal2_viol || F2_viol
       arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','k');grid on;hold on;

    else
       arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','b');grid on;hold on;
    end


    %plot world reference frame
    Tt = [eye(3), [0;0;0];
        zeros(1,3) 1];
    tt = hgtransform('Matrix', Tt);
    ht = triad('Parent',tt, 'linewidth', 6);


    %plot base reference frame
    Tt = [wRb, p_base;
        zeros(1,3) 1];
    tt = hgtransform('Matrix', Tt);
    %ht = triad('Parent',tt, 'linewidth', 6);


    %fundamental to see perpedicuolaity
    xlim([0, 2])
    ylim([min_y,max_y])
    zlim([min_z,max_z])
    axis equal
    view(60,30)
    pause(1)

    
end


%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [25 20]);
set(gcf, 'PaperPosition', [0 0 25 20]);
print(gcf, '-dpdf',strcat('static_analysis_3 points.pdf'),'-painters')
