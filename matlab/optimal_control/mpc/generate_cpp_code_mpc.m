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
%accurate
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

%perturb state
[act_p] = computePositionVelocity(params, actual_state(1), actual_state(2), actual_state(3));
% deterninistic
act_p =  act_p + [0.;0.1; 0.0];
%overwrite the position part of the state
act_state_pos_noise = computeStateFromCartesian(params, act_p);
actual_state(1:3) = act_state_pos_noise(1:3);

ref_com = solution.p(:,start_mpc:start_mpc+mpc_N-1);      
Fr_l0 = solution.Fr_l(:,start_mpc:start_mpc+mpc_N-1);
Fr_r0 = solution.Fr_r(:,start_mpc:start_mpc+mpc_N-1);

% sanity check
%[x, EXITFLAG, final_cost] = optimize_cpp_mpc(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params);
%Solution for Matlab 2020b: -50.0000  -50.0000  -41.5962   50.0000   50.0000   50.0000   14.5843   -6.8346  -13.1779   -9.7423   -3.2233         0   50.0000   50.0000   40.0472  -50.0000  -50.0000  -49.5350  -14.7668
% 6.6934   13.2390    9.8314    3.2192         0


% %generate c++ code with python bindings
cfg = coder.config('mex');
cfg.IntegrityChecks = false;
cfg.SaturateOnIntegerOverflow = false;
coder.cstructname(params, 'param')
% If var is an entry-point (top-level) function input argument, place coder.cstructname at the beginning of the function, before any control flow statements
%https://it.mathworks.com/help/simulink/slref/coder.cstructname.html
%https://it.mathworks.com/help/coder/ug/primary-function-input-specification.html
%codegen -config cfg  optimize_cpp_mpc -args { zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, zeros(1,1,'int64'),coder.cstructname(params, 'param') } -nargout 3 -report 
%codegen -config cfg  optimize_cpp_mpc_no_constraints -args { zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, zeros(1,1,'int64'),coder.cstructname(params, 'param') } -nargout 3 -report 
%codegen -config cfg  optimize_cpp_mpc_propellers -args { zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, zeros(1,1,'int64'),coder.cstructname(params, 'param') } -nargout 3 -report 



% Driver script to show codegen, SWIG, and Python (does not work)
%cfg = coder.config('dll');
%cfg.PostCodeGenCommand = 'pythonhook';
%codegen -config cfg  optimize_cpp_mpc -args {zeros(6,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,  0, 0 } -nargout 3 -report -o optimize_cpp_mpc.so
%copyfile codegen/dll/optimize_cpp_mpc/optimize_cpp_mpc.so optimize_cpp_mpcPython/
% check python
%disp(checkedSystem('python3 optimize_cpp_mpcMain.py'));


% check performances in matlab
for i=1:10
    tic
    [x, EXITFLAG, final_cost] =  optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params);% zeros(1,mpc_N), zeros(1,mpc_N));
    toc
end
%optimize_cpp_mpc_mex unit test
% -49.4214  -22.9852    0.2963    3.2071    6.9494   11.0391   11.5331   12.9415   19.0685   24.4259   25.7462   25.8806   34.2098   35.0223   16.7722   -2.6196  -15.0529  -19.4382
%-19.0498  -16.6146  -14.7696  -13.5718  -12.7079  -12.2642

system('python3 test_mpc_mex.py');

copyfile codegen ~/trento_lab_home/ros_ws/src/trento_lab_framework/locosim/robot_control/base_controllers/climbingrobot_controller/
copyfile optimize_cpp_mpc_mex.mexa64 ~/trento_lab_home/ros_ws/src/trento_lab_framework/locosim/robot_control/base_controllers/climbingrobot_controller/codegen/
copyfile optimize_cpp_mpc_no_constraints_mex.mexa64 ~/trento_lab_home/ros_ws/src/trento_lab_framework/locosim/robot_control/base_controllers/climbingrobot_controller/codegen/
copyfile optimize_cpp_mpc_propellers_mex.mexa64 ~/trento_lab_home/ros_ws/src/trento_lab_framework/locosim/robot_control/base_controllers/climbingrobot_controller/codegen/