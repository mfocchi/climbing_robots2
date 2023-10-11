function [p,pd,t] = eval_pos_vel_mpc( state0,  actual_t,  Fr_l0, Fr_r0,delta_Fr_l, delta_Fr_r, mpc_N, params, propeller_force)
    %init values for cpp
    p = zeros(3, mpc_N);
    pd = zeros(3, mpc_N);
    t = zeros(1, mpc_N);

    if  (length(Fr_l0) < mpc_N) || (length(Fr_r0) < mpc_N)
        disp('eval_pos_mpc:wrong input length: input should be longer than mpc_N')
        return
    end
    
    % check vectors are row and extract first mpc_N elements
    Fr_l0_mpc = Fr_l0(1:mpc_N);
    Fr_r0_mpc = Fr_r0(1:mpc_N);
 
    if nargin < 9
        % single shooting
        [states, t] = computeRollout(state0, actual_t,params.mpc_dt, mpc_N, Fr_l0_mpc + delta_Fr_l, Fr_r0_mpc + delta_Fr_r,[0;0;0],params.int_method,params.int_steps, params);
 
    else
          % single shooting
       [states, t] = computeRollout(state0, actual_t,params.mpc_dt, mpc_N, Fr_l0_mpc + delta_Fr_l, Fr_r0_mpc + delta_Fr_r,[0;0;0],params.int_method,params.int_steps, params, propeller_force);
    end
    
    psi = states(1,:);
    l1 = states(2,:);
    l2 = states(3,:);
    psid = states(4,:);
    l1d = states(5,:);
    l2d = states(6,:); 
    [p, pd ]= computePositionVelocity(params, psi, l1, l2, psid,l1d, l2d);
   
end