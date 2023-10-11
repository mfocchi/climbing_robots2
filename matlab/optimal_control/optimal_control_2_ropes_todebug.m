clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


%possible settings
test_type='normal' ;
%test_type='obstacle_avoidance' ;
%test_type='landing_test'; 


if strcmp(test_type, 'obstacle_avoidance')
    %jump params
    % INITIAL POINT
    p0 = [0.5; 0.5; -6]; % there is singularity for px = 0!
    %FINAL TARGET
    pf= [0.5; 4.5;-6];
    Fleg_max = 300;
    Fr_max = 90; % Fr is negative
    params.m = 5.08;   % Mass [kg]
    params.jump_clearance = 1;
    params.obstacle_avoidance  = true;
elseif  strcmp(test_type, 'landing_test')
    %jump params
    % INITIAL POINT
    p0 = [0.5; 2.5; -6]; % there is singularity for px = 0!
    %FINAL TARGET
    pf= [0.5; 4;-4];
    params.m = 15.07;
    Fleg_max = 600;
    Fr_max = 300;
    params.jump_clearance = 1.;
    params.obstacle_avoidance  = false;
else %normal
    %jump params
    % INITIAL POINT
    p0 = [0.5; 2.5; -6]; % there is singularity for px = 0!
    %FINAL TARGET
    pf= [0.5; 4;-4];
    Fleg_max = 300;
    Fr_max = 90; % Fr is negative
    params.jump_clearance = 1;
    params.m = 5.08;   % Mass [kg]
    params.obstacle_avoidance  = false;
end
 
params.obstacle_location = [-0.5; 3;-7.5];
%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
params.num_params = 4;   

%accurate
params.int_method = 'rk4';
params.N_dyn = 30; %dynamic constraints (number of knowts in the discretization) 
params.FRICTION_CONE = 1;
params.int_steps = 5.; %0 means normal intergation

%faster
% params.int_method = 'eul';
% params.int_steps = cast(5,"int64"); %0 means normal intergation
% params.N_dyn = 30; %dynamic constraints (number of knowts in the discretization) 
% params.FRICTION_CONE = 1;


params.contact_normal =[1;0;0];
params.b = anchor_distance;
params.p_a1 = [0;0;0];
params.p_a2 = [0;anchor_distance;0];
params.g = 9.81;
params.w1=1; % diff Fr1/2 smothing
params.w2=0; %hoist work
params.w3=0; %(not used)
params.w4=0;% %(not used)
params.w5=0; %  %(not used0 ekinf (important! energy has much higher values!)
params.w6=0;%  %(not used)
params.contact_normal =[1;0;0];
params.T_th =  0.05;
mu = 0.8;

constr_tolerance = 1e-3;

dt=0.001; % only to evaluate solution

%compute initial state from jump param
x0 = computeStateFromCartesian(params, p0);

%pendulum period
T_pend = 2*pi*sqrt(x0(2)/params.g)/4; % half period TODO replace with linearized x0(2) = l10
  
Fr_l0 = 0*ones(1,params.N_dyn);
Fr_r0 = 0*ones(1,params.N_dyn);
x0 = [  Fleg_max,  Fleg_max,  Fleg_max,        T_pend,  Fr_l0,                               Fr_r0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
lb = [  -Fleg_max,   -Fleg_max, -Fleg_max          0.01, -Fr_max*ones(1,params.N_dyn), -Fr_max*ones(1,params.N_dyn)];
ub = [  Fleg_max,    Fleg_max, Fleg_max,           inf,  0*ones(1,params.N_dyn),            0*ones(1,params.N_dyn)];


options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
'MaxFunctionEvaluations', 10000, 'ConstraintTolerance',constr_tolerance);

tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, params), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, Fleg_max, Fr_max, mu, params) , options);
toc
% evaluate constraint violation 
[c ceq, num_constr, solution_constr] = constraints(x, p0,  pf,  Fleg_max, Fr_max, mu, params);
solution = eval_solution(x, dt,  p0, pf, params) ;
solution.cost = final_cost;
solution.T_th = params.T_th;
problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);
% EXITFLAG ==1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
% EXITFLAG == 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.
%EXITFLAG == 0 max number of iterations

if problem_solved
    plot_curve( solution,solution_constr, p0, pf, mu,  false, 'r', true, params);
else 
    fprintf(2,"Problem didnt converge!\n")
    plot_curve( solution,solution_constr, p0, pf, mu,  false, 'k', true, params);
end

 
fprintf('Fleg:  %f %f %f \n\n',solution.Fleg(1), solution.Fleg(2), solution.Fleg(3))
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_kin_energy:  %f\n\n',solution.Ekinf)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution_constr.final_error_discrete)


DEBUG = true;

if (DEBUG)
    eval_constraints(c, num_constr, constr_tolerance)  
    figure
    ylabel('Fr-X')
    plot(solution.time,0*ones(size(solution.Fr_l)),'k'); hold on; grid on;
    plot(solution.time,-Fr_max*ones(size(solution.Fr_l)),'k');
    plot(solution.time,solution.Fr_l,'r');
    plot(solution.time,solution.Fr_r,'b');
    legend({'min','max','Frl','Frr'});
    
        
%     othing 
%     figure
%     subplot(3,1,1)
%     plot(solution.time, solution.psi,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.psi,'ob');
%     ylabel('psi')
% 
%     subplot(3,1,2)
%     plot(solution.time, solution.l1,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.l1,'ob');
%     ylabel('phi')
%     
%     subplot(3,1,3)
%     plot(solution.time, solution.l2,'r'); hold on; grid on;
%     plot(solution_constr.time, solution_constr.l2,'ob');
%     ylabel('l')
%     
%     figure
%     subplot(3,1,1)
%     plot(solution.time, solution.psid,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.psid,'ob');
%     ylabel('thetad')
% 
%     subplot(3,1,2)
%     plot(solution.time, solution.l1d,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.l1d,'ob');
%     ylabel('phid')
%     
%     subplot(3,1,3)
%     plot(solution.time, solution.l2d,'r'); hold on; grid on;
%     plot(solution_constr.time, solution_constr.l2d,'ob');
%     ylabel('ld')
        
    
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution_constr.time, solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution_constr.time, solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution_constr.time, solution_constr.p(3,:),'ob') ; hold on;
    ylabel('Z')
       
    
end

disp('Fleg')
solution.Fleg
solution.Tf
disp('target')
solution.achieved_target


% Fleg 27 iterazioni
%   120.3777
%   -37.6587
%   -65.7212
% 
% Tf    1.2175
% 
% target
%     0.5197
%     3.9967
% %    -4.0008
% cost:  -180.000000
% final_kin_energy:  84.553207
% final_error_real:  0.020000
% final_error_discrete:  0.020000
% max_integration_error:  0.000000


if strcmp(test_type, 'obstacle_avoidance')
    save('../simulation/compact_model/tests/test_matlab2obstacle.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
elseif strcmp(test_type, 'landing_test')  
    save('../simulation/compact_model/tests/test_matlab2landingClearance.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
else
    save('../simulation/compact_model/tests/test_matlab2.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');    
end

