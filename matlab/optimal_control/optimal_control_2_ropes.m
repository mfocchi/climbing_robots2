clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


USEGENCODE = true;
COPYTOLOCOSIM = false;


%possible settings
test_type='normal' ;
%test_type='obstacle_avoidance' ;
%test_type='landing_test'; 


if strcmp(test_type, 'obstacle_avoidance')
    %jump params
    % INITIAL POINT
    p0 = [0.5, 0.5, -6]; % there is singularity for px = 0!
    %FINAL TARGET
    pf= [0.5, 4.5, -6];
    %intermediate jump
    pf(2)= p0(2)+ (pf(2)-p0(2))/2;
    pf(1) = 1.5;

    Fleg_max = 300;
    Fr_max = 90; % Fr is negative

    % the order of params matters for code generation
    params.jump_clearance = 1; % ensure at least this detachment from wall
    params.m = 5.08;   % Mass [kg]
    params.obstacle_avoidance  = true;

elseif  strcmp(test_type, 'landing_test')
    %jump params
    % INITIAL POINT
    p0 = [0.5, 2.5, -6]; % there is singularity for px = 0!
    %FINAL TARGET
    pf= [0.5, 4,-4];
  
    Fleg_max = 600;
    Fr_max = 300;

    % the order of params matters for code generation
    params.jump_clearance = 1.; % ensure at least this detachment from wall
    params.m = 15.07; 
    params.obstacle_avoidance  = false;

else %normal
    %jump params
    % INITIAL POINT
    p0 = [0.5, 2.5, -6]; % there is singularity for px = 0!
    %FINAL TARGET
    pf= [0.5, 4,-4];
    Fleg_max = 300;
    Fr_max = 90; % Fr is negative

    % the order of params matters for code generation
    params.jump_clearance = 1; % ensure at least this detachment from wall
    params.m = 5.08;   % Mass [kg]
    params.obstacle_avoidance  = false;
end
 
params.obstacle_location = [-0.5; 2.5; -6];
params.obstacle_size = [1.5; 1.5; 0.866];

%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
params.num_params = 4;   
params.int_method = 'rk4';
params.N_dyn = 30; %dynamic constraints (number of knowts in the discretization) 
params.FRICTION_CONE = 1;
params.int_steps = 5.; %0 means normal intergation
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

%gen code (run if you did some change in the cost)
if ~isfile('optimize_cpp_mex.mexa64')
    disp('Generating C++ code');
    % generates the cpp code
    % run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], 0, 0, 0, coder.cstructname(params, 'param') } -nargout 1 -report
    if COPYTOLOCOSIM
        disp("copying to locosim")  
        copyfile codegen/mex/optimize_cpp/ ~/trento_lab_home/ros_ws/src/locosim/robot_control/base_controllers/climbingrobot_controller/codegen/mex/optimize_cpp
        copyfile optimize_cpp_mex.mexa64 ~/trento_lab_home/ros_ws/src/locosim/robot_control/base_controllers/climbingrobot_controller/codegen/
    end
end


mpc_fun   = 'optimize_cpp';
if USEGENCODE  
    mpc_fun=append(mpc_fun,'_mex' );
end
mpc_fun_handler = str2func(mpc_fun);
solution = mpc_fun_handler(p0,  pf, Fleg_max, Fr_max, mu, params);

switch solution.problem_solved
    case 1 
        fprintf(2,"Problem converged!\n")
        plot_curve( solution,solution.solution_constr, p0, pf, mu,  false, 'r', true, params);
    case -2  
        fprintf(2,"Problem didnt converge!\n")
        plot_curve( solution,solution.solution_constr, p0, pf, mu,  false, 'k', true, params);
    case 2 
        fprintf(2,"semidefinite solution (should modify the cost)\n")
        plot_curve( solution,solution.solution_constr, p0, pf, mu,  false, 'r', true, params);

    case 0 
        fprintf(2,"Max number of feval exceeded (10000)\n")
end
 
fprintf('Fleg:  %f %f %f \n\n',solution.Fleg(1), solution.Fleg(2), solution.Fleg(3))
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_kin_energy:  %f\n\n',solution.Ekinf)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution.solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution.solution_constr.final_error_discrete)


DEBUG = false;

if (DEBUG)
    eval_constraints(solution.c, solution.num_constr, solution.constr_tolerance)  
    figure
    ylabel('Fr-X')
    plot(solution.time,0*ones(size(solution.Fr_l)),'k'); hold on; grid on;
    plot(solution.time,-Fr_max*ones(size(solution.Fr_l)),'k');
    plot(solution.time,solution.Fr_l,'r');
    plot(solution.time,solution.Fr_r,'b');
    legend({'min','max','Frl','Frr'});
    
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution.solution_constr.time, solution.solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution.solution_constr.time, solution.solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution.solution_constr.time, solution.solution_constr.p(3,:),'ob') ; hold on;
    ylabel('Z')
       

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
   
    
end

fprintf('Leg impulse force: %f %f %f\n\n', solution.Fleg);
fprintf('Jump Duration: %f\n\n', solution.Tf);
fprintf('Landing Target: %f %f %f\n\n', solution.achieved_target);

[impulse_work , hoist_work, hoist_work_fine] = computeJumpEnergyConsumption(solution,params);
Energy_consumed = impulse_work+hoist_work_fine;
fprintf('Energy_consumed [J]: %f \n\n', Energy_consumed);



