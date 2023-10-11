clear all
clc
close all

global delta_duration  Fleg  Fr1 Fr2  time  optim_time OPTIM  p_a1 p_a2 b   g m

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

OPTIM = true;
%possible settings
test_type='normal'; 
%test_type='obstacle_avoidance';  
%test_type='landing_test';

if OPTIM %inputs from optim
    if strcmp(test_type, 'obstacle_avoidance')
        load ('tests/test_matlab2obstacle.mat');
        m = 5.08;   % Mass [kg]
    elseif strcmp(test_type, 'landing_test')  
        load ('tests/test_matlab2landingClearance.mat');
        m = 15.07 ;   % Mass [kg]    
    else    	
        load ('tests/test_matlab2.mat');
        m = 5.08;   % Mass [kg]
    end           
            
    %load ('test_matlab_cpp.mat');    
    %load ('test_matlab2landingClearance.mat');
    Tf = solution.Tf; 
    dt = 0.001;
    time = [0:dt:Tf];
    Fleg = solution.Fleg;
    optim_time = solution.time;
    Fr1 = solution.Fr_l; %resample creates oscillations!
    Fr2 = solution.Fr_r;
    delta_duration = solution.T_th;
    force_scaling = 100;
    p0 = solution.p(:,1);
    
else %fixed inputs
    Tf = 4;
    dt = 0.001;
    time =[0:dt:Tf];
    delta_duration = 0.05;
    %inputs
    Fleg = [200;0;0];
    Fr1 = ones(1, length(time)) * -40;
    Fr2 = ones(1, length(time)) * -30;
    force_scaling = 20;
    %jump params
    p0 = [0.5; 2.5; -6]; % there is singularity for px = 0!
    m = 5.08;   % Mass [kg]
end     

%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
b = anchor_distance;
p_a1 = [0;0;0];
p_a2 = [0;anchor_distance;0];
g = 9.81;

%compute initial state from jump param
x0 = computeStateFromCartesian(p0);

%%Simulation:
% define the stop function event
Opt    = odeset('Events', @stopFun);


% % %1 - Solve differential equations with variable step solver
[time_sim, x] = ode45(@(time_sim,x) diffEq(time_sim, x, @Fr1Fun,@Fr2Fun, @FlegFun), time, x0, Opt); 
for i=1:length(x)    
    [X(i), Y(i), Z(i)] = forwardKin(x(i,1), x(i,2), x(i,3));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D plot Animation
figure(3)

title('Matlab Animation - simplified model');
xlabel('X ') ; ylabel('Y ') ; zlabel('Z ');

axis equal; hold on;
%anchor 1    
h(1) = plot3(p_a1(1),p_a1(2),p_a1(3),'.m', 'MarkerSize',40);grid on;hold on;
%anchor2    
h(2) = plot3(p_a2(1),p_a2(2),p_a2(3),'.y', 'MarkerSize',40);grid on;hold on;

%initial leg impulse (red)
h(9) = plot3([p0(1) p0(1)+Fleg(1)/force_scaling],[p0(2) p0(2)+Fleg(2)/force_scaling],[p0(3) p0(3)+Fleg(3)/force_scaling],'r','Linewidth',4);

% rope forces
Fr1_vec =  (p0 - p_a1)/norm(p0 - p_a1)*Fr1(1);
Fr2_vec =  (p0 - p_a2)/norm(p0 - p_a2)*Fr2(1);
%Fr1 (red ) (forces are positive if they accelerate the mass
h(10) = plot3([p0(1) p0(1)+Fr1_vec(1)/force_scaling],[p0(2) p0(2)+Fr1_vec(2)/force_scaling],[p0(3) p0(3)+Fr1_vec(3)/force_scaling],'r','Linewidth',4);
%Fr2 (red ) 
h(11) = plot3([p0(1) p0(1)+Fr2_vec(1)/force_scaling],[p0(2) p0(2)+Fr2_vec(2)/force_scaling],[p0(3) p0(3)+Fr2_vec(3)/force_scaling],'r','Linewidth',4);

%plot world reference frame
Tt = [eye(3), [0;0;0];
    zeros(1,3) 1];
tt = hgtransform('Matrix', Tt);
h(12) = triad('Parent',tt, 'linewidth', 6);

%  plot rope lines (black)
h(4) = plot3([p_a1(1) p0(1)],[p_a1(2) p0(2)],[p_a1(3) p0(3)],'k-');
h(5) = plot3([p_a2(1) p0(1)],[p_a2(2) p0(2)],[p_a2(3) p0(3)],'k-');

min_x= 0;
max_x= 2*max(X);
min_z = min(Z)-2;
max_z = 1;
min_y = -2;
max_y = min_y+anchor_distance+4;

%  drawing a wall at X = 0 
p1 = [0 min_y min_z];
p2 = [0 max_y min_z];
p3 = [0 max_y max_z];
p4 = [0 min_y max_z]; 
Xw = [p1(1) p2(1) p3(1) p4(1)];
Yw = [p1(2) p2(2) p3(2) p4(2)];
Zw = [p1(3) p2(3) p3(3) p4(3)];  
h(6) = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.2  );

set(gca,'XLim',[min_x max_x])
set(gca,'YLim',[min_y max_y])
set(gca,'ZLim',[min_z max_z]) 
set(gca,'fontsize',30)

h(7) = animatedline('color','g', 'linewidth',3);
h(8) = animatedline('color','b', 'linewidth',3);

%Pendulum sphere (red)
% h(7) = animatedline('Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);

if strcmp(test_type, 'obstacle_avoidance')   
    h(9) = plot_ellipsoid([-0.5, 2.5,-6],1.5, 1.5, 0.866,  min_z, max_z, min_y,max_y);
end
view(60,27);

% Loop for animation
for i = 1:length(X)    
    %Pendulum trajectory position of the pendulum green during push blue
    %during flight
    if time_sim(i)<=delta_duration
        addpoints(h(7), X(i),Y(i),Z(i)); 

    else 
        addpoints(h(8)  ,X(i),Y(i),Z(i));
       
    end
    drawnow limitrate;
    pause(0.001);
 
end

% plot target point (red)
h(13) = plot3(X(end),Y(end), Z(end),'.r', 'MarkerSize',40);
axis equal

% OTHER PLOTS
if OPTIM
    figure(1)
    subplot(3,1,1)
    plot(time_sim, X,'b'); hold on;grid on;
    plot(solution.time, solution.p(1,:),'ro');
    ylabel('X')
    legend('sim', 'opt')

    subplot(3,1,2)
    plot(time_sim, Y,'b'); hold on;grid on;
    plot(solution.time, solution.p(2,:),'ro');
    ylabel('Y')

    subplot(3,1,3)
    plot(time_sim, Z,'b'); hold on;grid on;
    plot(solution.time, solution.p(3,:),'ro');
    ylabel('Z')

    % states
    figure(2)
    subplot(3,1,1)
    plot(time_sim, x(:,1),'b'); hold on;grid on;
    plot(solution.time, solution.psi,'ro');
    ylabel('psi')
    legend('sim', 'opt')

    subplot(3,1,2)
    plot(time_sim, x(:,2),'b'); hold on;grid on;
    plot(solution.time, solution.l1,'ro');
    ylabel('l1')

    subplot(3,1,3)
    plot(time_sim, x(:,3),'b'); hold on;grid on;
    plot(solution.time, solution.l2,'ro');
    ylabel('l2')
    
    figure(5)
    plot(optim_time, Fr1,'r-o'); hold on; grid on;
    plot(optim_time, Fr2,'b-o'); hold on; grid on;
end

matlab_final_point = [X(end);Y(end);Z(end)];
gazebo_final_point =[-0.00298  1.55479 -2.21499];
if OPTIM
    fprintf('Optim final point [%3.4f, %3.4f, %3.4f] \n', solution.achieved_target)
    fprintf('error norm[%3.4f %3.4f %3.4f] \n',matlab_final_point - solution.achieved_target)
end
fprintf('Matlab final point [%3.4f, %3.4f, %3.4f] \n', matlab_final_point)
fprintf('Gazebo final point [%3.4f, %3.4f, %3.4f] \n', gazebo_final_point)
fprintf('Touchdown at s t [%3.4f] \n', time_sim(end))

% this is needed because the intergration time t might be different from
% the discretization of the input Fr
function [fr1] = Fr1Fun(t)
global   Fr1 time optim_time OPTIM

   if OPTIM
       t_ = optim_time;
   else 
       t_ = time;
   end
   idx = min(find(t_>t))-1;
   fr1 = Fr1(idx);
end

% this is needed because the intergration time t might be different from
% the discretization of the input Fr
function [fr2] = Fr2Fun(t)
global   Fr2 time optim_time OPTIM

   if OPTIM
       t_ = optim_time;
   else 
       t_ = time;
   end
   idx = min(find(t_>t))-1;
   fr2 = Fr2(idx);
end

function [fleg] = FlegFun(t)
global   Fleg  delta_duration 

    if (t <= delta_duration)
        fleg = Fleg(:);
    else
        fleg = [0;0;0];
    end
  
end



function [value, isterminal, direction] = stopFun(t, x )
         global OPTIM 
        
        
         
        if OPTIM
            %value =  optim_time(end) - t;
            value =1; % stops at the end of time
        else
            [px, py, pz] = forwardKin(x(1), x(2), x(3));
            value = px;%stop when gets to wall
        end
 
        direction  = -1;
        isterminal = 1;   % Stop the integration
       
end

function [dxdt] = diffEq(t,x, Fr1, Fr2 ,Fleg)

    global  b g m     
 

    %Retrieving states
    psi = x(1);
    l1 = x(2);
    l2 = x(3);
    psid = x(4);
    l1d = x(5);
    l2d = x(6);
    
    [px, py, pz]  = forwardKin(psi, l1, l2);  
    pz2b = pz*2*b;
    px2b = px*2*b;
    px_l1 = px/l1;%
    n_pz_l1 =  -pz/l1;%
    px_l1_sinpsi = px/l1/sin(psi);%
    py2b = py*2*b;%
    
    % mass equation and rope constraints 
    A_dyn = [l1*n_pz_l1,   px_l1 - (l1*sin(psi)*(py2b/(b^2*l1) - py2b^2/(2*b^2*l1^3)))/(2*px_l1_sinpsi),  (l2*py2b*sin(psi))/(2*b^2*l1*px_l1_sinpsi),
                      0,                                                                           l1/b,                                       -l2/b,
               l1*px_l1, (l1*cos(psi)*(py2b/(b^2*l1) - py2b^2/(2*b^2*l1^3)))/(2*px_l1_sinpsi) - n_pz_l1, -(l2*py2b*cos(psi))/(2*b^2*l1*px_l1_sinpsi)];
    

    b_dyn =  [2*l1d*n_pz_l1*psid - l1*psid^2*px_l1 - (sin(psi)*(4*l1^4*l1d^2 - 8*l1^3*l2*l1d*l2d + 4*l1^2*l2^2*l2d^2 - 6*l1^2*l1d^2*py2b - 2*l1^2*l2d^2*py2b + 8*l1*l2*l1d*l2d*py2b + 3*l1d^2*py2b^2))/(4*b^2*l1^3*px_l1_sinpsi) - (py2b^2*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2)^2)/(16*b^4*l1^5*px_l1_sinpsi^3) + (psid*py2b*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^2*px_l1_sinpsi) + (l1d*py2b*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^3*px_l1_sinpsi),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               (l1d^2 - l2d^2)/b,
               l1*n_pz_l1*psid^2 + 2*l1d*psid*px_l1 + (cos(psi)*(4*l1^4*l1d^2 - 8*l1^3*l2*l1d*l2d + 4*l1^2*l2^2*l2d^2 - 6*l1^2*l1d^2*py2b - 2*l1^2*l2d^2*py2b + 8*l1*l2*l1d*l2d*py2b + 3*l1d^2*py2b^2))/(4*b^2*l1^3*px_l1_sinpsi) + (py2b^2*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2)^2)/(16*b^4*l1^5*px_l1_sinpsi^3) - (l1d*py2b*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^3*px_l1_sinpsi) + (psid*py2b*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^2*px_l1_sinpsi)];
 
    

    J =  computeJacobian([px, py, pz]);
    Ftot = [m*[0;0;-g] + J*[Fr1(t);Fr2(t)] + Fleg(t)]; 


    y = inv(A_dyn)*(inv(m)*Ftot - b_dyn);
    dxdt = [psid; l1d; l2d;  y];


end

