clc; clear all;

load('test_matlab2.mat')

addpath('../')
DEBUG_DYNAMICS = false;
DEBUG_MPC_MACHINERY = false;
DISTURBED_VARIALES = 'state' % 'cartesian' %TODO
USEGENCODE = true % need to run gen_cpp_code_mpc
PROPELLERS = true;

Fr_max = 100; % Fr is negative (max variation)

dt=0.001; % only to evaluate solution
N_dyn = length(solution.time);
mpc_N = cast(0.6*length(solution.time), "int64");

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
params.m = 5.08;   % Mass [kg]
params.w1 =1; % tracking
params.w2= 0.000001; % smooth term 
params.mpc_dt = solution.Tf / (N_dyn-1);

samples = length(solution.time) - mpc_N+1;
start_mpc = 3;
actual_t = solution.time(start_mpc);
actual_state = [solution.psi(:,start_mpc);solution.l1(:,start_mpc);solution.l2(:,start_mpc); solution.psid(:,start_mpc); solution.l1d(:,start_mpc); solution.l2d(:,start_mpc)];


if DEBUG_DYNAMICS
   % 1 - sanity check: on traj consider after the application of T_th!
   % (sample > 2)
    pos = eval_pos_vel_mpc(actual_state,  actual_t, solution.Fr_l(start_mpc:end), solution.Fr_r(start_mpc:end),zeros(1,mpc_N),zeros(1,mpc_N), mpc_N, params)
    solution.p(:,3:samples)
end
delta_Fr_l = zeros(1,mpc_N);
delta_Fr_r = zeros(1,mpc_N);
extra_forces= zeros(1,mpc_N);

mpc_fun   = 'optimize_cpp_mpc'
if PROPELLERS
    mpc_fun=append(mpc_fun,'_propellers')
end
%Optimization
if USEGENCODE
    mpc_fun=append(mpc_fun,'_mex' )
end
mpc_fun_handler = str2func(mpc_fun);


for i=start_mpc:samples
    ref_com = solution.p(:,i:i+mpc_N-1);      
    Fr_l0 = solution.Fr_l(:,i:i+mpc_N-1);
    Fr_r0 = solution.Fr_r(:,i:i+mpc_N-1);
    fprintf("Iteration #: %d\n", i)
    
    if ~DEBUG_DYNAMICS
        
        %debug mpc machinery
        if ~DEBUG_MPC_MACHINERY
            
             %ADD NOISE ON position   
             %compute position relative to actualstate      
             % [act_p] = computePositionVelocity(params, actual_state(1), actual_state(2), actual_state(3));
             % 
             % %adding noise only on position!
             % %1 - random noise 
             % %act_p =  act_p + [0.0*rand(1);0.2*rand(1); 0.0*rand(1)];
             % % 2 deterninistic noise (only errors on Y and Z are corrected because of
             % % underactuation!)
             % act_p =  act_p + [0.;0.1; 0.0];
             % 
             % %overwrite the position part of the state after adding noise
             % act_state_pos_noise = computeStateFromCartesian(params, act_p);
             % actual_state(1:3) = act_state_pos_noise(1:3);

             %ADD NOISE ON state
             actual_state(1:3) = actual_state(1:3) +[0.05;0.1; 0.0]; %make the disturbance suvh that the robot is pulling
             
             
             %%%%%%%%%%%%%%%%%%%%%%%
            
             tic
             [x, EXITFLAG, final_cost] = mpc_fun_handler(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params);% bootstrap  delta_Fr_l, delta_Fr_r);
              toc
             delta_Fr_l = x(1:mpc_N);
             delta_Fr_r = x(mpc_N+1:2*mpc_N);
             if PROPELLERS
                 extra_forces = x(2*mpc_N+1:3*mpc_N);
             end
        end
        
        % predict new state
        [mpc_states, t] = computeRollout(actual_state, actual_t,params.mpc_dt, mpc_N, Fr_l0 + delta_Fr_l, Fr_r0 + delta_Fr_r,[0;0;0],params.int_method,params.int_steps, params, extra_forces);
 
            
        % predict new traj
        [mpc_p, mpc_pd, mpc_time] = eval_pos_vel_mpc(actual_state,  actual_t, Fr_l0, Fr_r0,delta_Fr_l ,delta_Fr_r, mpc_N, params, extra_forces);

        %update dynamics (this emulates the real dynamics with noise)
        [actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, params.mpc_dt/(params.int_steps-1), params.int_steps, (Fr_l0(1) + delta_Fr_l(1))*ones(1,params.int_steps),  (Fr_r0(1) + delta_Fr_r(1))*ones(1,params.int_steps),[0;0;0], params.int_method, params, extra_forces); 


        %actual_com = computePositionVelocity(actual_state(1), actual_state(2), actual_state(3));   

        %plot
        clf(gcf)
        set(0, 'DefaultAxesBox', 'on');
        set(0, 'DefaultTextFontSize', 30);
        set(0, 'DefaultAxesFontSize', 30);
        set(0, 'DefaultUicontrolFontSize', 30);    
        %plot cartesian 
        % subplot(3,1,1)    
        % plot(solution.time(start_mpc:end), solution.p(1,start_mpc:end), 'ro-'); grid on;hold on;
        % plot(mpc_time, mpc_p(1,:), 'bo-'); grid on;hold on;
        % ylabel('X')
        % subplot(3,1,2)       
        % plot(solution.time(start_mpc:end), solution.p(2,start_mpc:end), 'ro-'); grid on;hold on;
        % plot(mpc_time, mpc_p(2,:), 'bo-'); grid on;hold on;
        % ylabel('Y')
        % subplot(3,1,3)   
        % plot(solution.time(start_mpc:end), solution.p(3,start_mpc:end), 'ro-'); grid on;hold on;
        % plot(mpc_time, mpc_p(3,:), 'bo-'); grid on;hold on;  ylabel('Z')

        % plot states
        subplot(3,2,1)    
        plot(solution.time(start_mpc:end), solution.psi(start_mpc:end), 'ro-'); grid on;hold on;
        plot(mpc_time, mpc_states(1,:), 'bo-'); grid on;hold on; ylabel('psi')
        
        subplot(3,2,3)       
        plot(solution.time(start_mpc:end), solution.l1(start_mpc:end), 'ro-'); grid on;hold on;
        plot(mpc_time, mpc_states(2,:), 'bo-'); grid on;hold on; ylabel('l1')
        
        subplot(3,2,5)   
        plot(solution.time(start_mpc:end), solution.l2(start_mpc:end), 'ro-'); grid on;hold on;
        plot(mpc_time, mpc_states(3,:), 'bo-'); grid on;hold on;  ylabel('l2')
        
        subplot(3,2,2)           
        plot(mpc_time, delta_Fr_l, 'go-'); grid on;hold on;  ylabel('deltaFrl'); grid on;hold on;  
        plot(mpc_time, Fr_l0, 'ko-'); grid on;hold on;  ylabel('deltaFrl'); grid on;hold on;
        xlim([min(mpc_time), max(mpc_time)]);
        legend({'deltaFrl', 'Fr_l0'})
   
        subplot(3,2,4)   
        plot(mpc_time, delta_Fr_r, 'bo-'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;
        plot(mpc_time, Fr_r0, 'ko-'); grid on;hold on;  ylabel('deltaFrl'); grid on;hold on;
        xlim([min(mpc_time), max(mpc_time)]);
        legend({'deltaFrr', 'Fr_r0'})
        
        subplot(3,2,6)
        plot(mpc_time, Fr_l0 + delta_Fr_l, 'go-'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;
        plot(mpc_time, Fr_r0 + delta_Fr_r, 'bo-'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;       
        plot(mpc_time, ones(1,length(mpc_time))*(-Fr_max), 'r'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;
        plot(mpc_time, ones(1,length(mpc_time))*0., 'r'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;
        
        string_legend = {'Fl + deltaFl','Fr + deltaFr','min Fr', 'maxFr'};
        
        if PROPELLERS
            plot(mpc_time, extra_forces, 'ko-'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;
            string_legend{5} = 'propeller';
        end
        legend(string_legend);
        
        pause(0.3)

    else

        % sanity check : uncomment one of the two
         ref_com(:,1) % need to consider the next sample
         actual_com = computePositionVelocity(params, actual_state(1), actual_state(2), actual_state(3))

         % 2 - sanity check with  for loop and integrate dynamics
         [actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, params.mpc_dt/(params.int_steps-1), params.int_steps, (Fr_l0(1) )*ones(1,params.int_steps),  (Fr_r0(1) )*ones(1,params.int_steps),[0;0;0], params.int_method, params); % keep Fr constant   
         % 3 - sanity check with for loop and computeRollout
         % [state12, time12] = computeRollout(actual_state, actual_t,params.mpc_dt, 2, Fr_l0(1) , Fr_r0(1)  ,[0;0;0],params.int_method,params.int_steps, params);  
         % actual_state = state12(:,2);   
    end
     
end

