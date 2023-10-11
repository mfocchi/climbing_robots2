function cost = cost_mpc_propellers(x, state0,  actual_t, ref_com, Fr_l0, Fr_r0,mpc_N, params)

    % init for cpp
    ref_com_mpc = zeros(3,mpc_N);
    cost=0;
    
  
    
    if (length(ref_com) < mpc_N) 
        disp('cost_mpc:wrong ref_com input length: ref_com should be longer than mpc_N')
        ref_com
        return
    end
    
    delta_Fr_l = x(1:mpc_N); 
    delta_Fr_r = x(mpc_N+1:2*mpc_N); 
    propeller_forces = x(2*mpc_N+1:3*mpc_N);

    
    % compute actual state
    ref_com_mpc = ref_com(:, 1:mpc_N);             
    [p, pd, t] = eval_pos_vel_mpc(state0,  actual_t,  Fr_l0, Fr_r0,delta_Fr_l, delta_Fr_r, mpc_N, params, propeller_forces);
       
    %p has mpc_N +1 elements 
    % cartesian track
    norm_order = 2;
    row_column_wise =1; %1 column /2 row wise
    tracking_cart= sum (vecnorm(ref_com_mpc - p, norm_order, row_column_wise).^2); 
    terminal_cost = norm(ref_com_mpc(:,mpc_N) - p(:,mpc_N));
    % %state tracking (is worse than cart tracking)
    % tracking_state = 0.;
    % for i=1:mpc_N
    %     state_ref = computeStateFromCartesian(params, ref_com_mpc(:,i));
    %     state = computeStateFromCartesian(params, p(:,i));
    %     tracking_state = tracking_state + norm(state_ref(2:3) -state(2:3));%consider only l1 l2
    % end

    % smoothnes: minimize jerky control action
    smooth = sum(diff(delta_Fr_l).^2)+ sum(diff(delta_Fr_r).^2) +  sum(diff(propeller_forces).^2);  % this creates tracking errors sum(delta_Fr_l.^2) + sum(delta_Fr_r.^2);
         
    cost =  params.w1* tracking_cart +params.w2 *smooth;% + params.w3* terminal_cost; % creates issues in gazebo
end