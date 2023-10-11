clc; clear all;

addpath('../');

% normal test
load('test_matlab2.mat')
Fr_max = 100; % Fr is negative (max variation) WE DO NOT CONSTRAINT THE MAX
mass =5.08;

% landing test
%load('test_matlab2landingClearance.mat')
%Fr_max = 150; % Fr is negative (max variation)
%mass = 15.07;

constr_tolerance = 1e-3;
dt=0.001; % only to evaluate solution
N_dyn = length(solution.time);
mpc_N = cast(0.4*length(solution.time), "int64");

%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
params.int_method = 'rk4';
params.int_steps = 5.; %0 means normal intergation
params.contact_normal =[1;0;0];
params.b = anchor_distance;
params.p_a1 = [0;0;0];
params.p_a2 = [0;anchor_distance;0];
params.g = 9.81;
params.m = mass;   % Mass [kg]
params.w1 =1; % tracking
params.w2= 0.000001; % smooth term 
params.mpc_dt = solution.Tf / (N_dyn-1);

samples = length(solution.time) - mpc_N+1;
start_mpc = 4;
actual_t = solution.time(start_mpc);
actual_state = [solution.psi(:,start_mpc);solution.l1(:,start_mpc);solution.l2(:,start_mpc); solution.psid(:,start_mpc); solution.l1d(:,start_mpc); solution.l2d(:,start_mpc)];

%perturbed state
[act_p] = computePositionVelocity(params, actual_state(1), actual_state(2), actual_state(3));
act_p =  act_p + [0.;0.1; 0.0];
%overwrite the position part of the state
act_state_pos_noise = computeStateFromCartesian(params, act_p);
actual_state(1:3) = act_state_pos_noise(1:3);

ref_com = solution.p(:,start_mpc:start_mpc+mpc_N-1);      
Fr_l0 = solution.Fr_l(:,start_mpc:start_mpc+mpc_N-1);
Fr_r0 = solution.Fr_r(:,start_mpc:start_mpc+mpc_N-1);

%generate c++ code with python bindings
cfg = coder.config('mex');
cfg.IntegrityChecks = false;
cfg.SaturateOnIntegerOverflow = false;
coder.cstructname(params, 'param')
%codegen -config cfg  optimize_cpp_mpc -args { zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, zeros(1,1,'int64'),coder.cstructname(params, 'param') } -nargout 3 -report 
%codegen -config cfg  optimize_cpp_mpc_no_constraints -args { zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, zeros(1,1,'int64'),coder.cstructname(params, 'param') } -nargout 3 -report 
%codegen -config cfg  optimize_cpp_mpc_propellers -args { zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, zeros(1,1,'int64'),coder.cstructname(params, 'param') } -nargout 3 -report 

[x, EXITFLAG, final_cost] =  optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params);% zeros(1,mpc_N), zeros(1,mpc_N));

copyfile codegen ~/trento_lab_home/ros_ws/src/climbing_robots2/robot_control/base_controllers/
copyfile optimize_cpp_mpc_mex.mexa64 ~/ros_ws/src/trento_lab_framework/climbing_robots2/robot_control/base_controllers/codegen/
copyfile optimize_cpp_mpc_no_constraints_mex.mexa64 ~/trento_lab_home/ros_ws/src/climbing_robots2/robot_control/base_controllers/codegen/
copyfile optimize_cpp_mpc_propellers_mex.mexa64 ~/trento_lab_home/ros_ws/src/climbing_robots2/robot_control/base_controllers/codegen/