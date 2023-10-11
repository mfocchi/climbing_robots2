

function [x, EXITFLAG, final_cost] = optimize_cpp_mpc_propellers(actual_state, actual_t, ref_com, Fr_l0, Fr_r0,  Fr_max, mpc_N, params)% , delta_Fr_l0, delta_Fr_r0)  

        constr_tolerance = 1e-3;
        % bootstrap
%         if nargin <9 
%             delta_Fr_l0 = 0*ones(1,mpc_N);
%             delta_Fr_r0 = 0*ones(1,mpc_N);
%         end
        % init to zeros
        delta_Fr_l0 = 0*ones(1,mpc_N);
        delta_Fr_r0 = 0*ones(1,mpc_N);
        propeller_force0 = 0*ones(1,mpc_N);
        propeller_force_max = 100; %TODO generalize
        x0 = [   delta_Fr_l0,                    delta_Fr_r0, propeller_force0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
        lb = [  -Fr_max*ones(1,mpc_N), -Fr_max*ones(1,mpc_N), -propeller_force_max*ones(1,mpc_N)];
        ub = [   Fr_max*ones(1,mpc_N),  Fr_max*ones(1,mpc_N), propeller_force_max*ones(1,mpc_N)];
        options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
        'MaxFunctionEvaluations', 5000, 'ConstraintTolerance', constr_tolerance);

        % use constraint on force
        [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost_mpc_propellers(x, actual_state, actual_t, ref_com, Fr_l0, Fr_r0, mpc_N,params),  x0,[],[],[],[],lb,ub, @(x) constraints_mpc(x, Fr_max, Fr_l0, Fr_r0, mpc_N), options);
               
  
end
