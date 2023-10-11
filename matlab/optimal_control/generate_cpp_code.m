clear all; close all

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

%jump parameters
% INITIAL POINT
p0 = [ 0.5, 2.5  , -6]; % there is singularity for px = 0!
%FINAL TARGET
pf= [0.5, 4,-4];

% obstacle avoidance (remember to set params.obstacle_avoidance = true )
% p0 = [0.5, 0.5, -6]; 
% pf= [0.5, 4.5,-6];
params.jump_clearance = 1;

% normal test
mass = 5.08; 
Fleg_max = 300;
Fr_max = 90; % Fr is negative

% %landing test
% mass =  15.07; 
% Fleg_max =  600;
% Fr_max = 300; % Fr is negative

mu = 0.8;
params.m = mass;   % Mass [kg]
params.obstacle_avoidance  = false;
params.obstacle_location = [-0.5; 2.5;-6];
anchor_distance = 5;
params.num_params = 4.;   
params.int_method = 'rk4'; % Eul
params.N_dyn = 30.; %dynamic constraints (number of knowts in the discretization) 
params.FRICTION_CONE = 1;
params.int_steps = 5.; %0 means normal integration
params.contact_normal =[1;0;0];
params.b = anchor_distance;
params.p_a1 = [0;0;0];
params.p_a2 = [0;anchor_distance;0];
params.g = 9.81;
params.w1= 1; %  smoothing
params.w2= 0.; %hoist work
params.w3= 0; % (not used)
params.w4= 0; % (not used)
params.w5= 0; % (not used)
params.w6= 0; % (not used)
params.T_th =  0.05;

% generates the cpp code
% cfg = coder.config('mex');
% cfg.IntegrityChecks = false;
% cfg.SaturateOnIntegerOverflow = false;
% codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], 0, 0, 0, coder.cstructname(params, 'param') } -nargout 1 -report

%run normal code
% solution1 = optimize_cpp(p0,  pf, Fleg_max, Fr_max, mu, params) 
% solution1.Tf
% solution1.achieved_target
% plot_solution(solution1, p0, pf, Fleg_max, Fr_max, mu, params) 

%run generated code
solution = optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, params);
plot_solution(solution, p0, pf, Fleg_max, Fr_max, mu, params); 


switch solution.problem_solved
    case 1 
        fprintf(1,"Problem converged!\n")
    case -2  
        fprintf(2,"Problem didnt converge!\n")
    case 2 
        fprintf(1,"semidefinite solution (should modify the cost)\n")
    case 0 
        fprintf(2,"Max number of feval exceeded (10000)\n")
end


 % [impulse_work , hoist_work, hoist_work_fine] = computeJumpEnergyConsumption(solution ,params)
 % E = impulse_work+hoist_work_fine;

%this is to save the result for simulation in matlab
%save('../simulation/compact_model/tests/test_matlab2obstacle.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');

%save('../simulation/compact_model/tests/test_matlab2_cpp.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');

%copyfile codegen ~/trento_lab_home/ros_ws/src/climbing_robots2/robot_control/base_controllers/
%copyfile optimize_cpp_mex.mexa64 ~/trento_lab_home/ros_ws/src/trento_lab_framework/climbing_robots2/robot_control/base_controllers/codegen/