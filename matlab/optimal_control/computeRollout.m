function [states_rough, t_rough] = computeRollout(x0, t0, dt_dyn, N_dyn, Fr_l, Fr_r, Fleg, int_method, int_steps, params, extra_forces)

    %init
    states_rough = zeros(6, N_dyn);
    t_rough = zeros(1, N_dyn);

    if nargin <11
        extra_forces = zeros(1, N_dyn);
    end     
          

    if  (int_steps ==0)
        [~,~,states_rough, t_rough] = integrate_dynamics(x0, 0,dt_dyn, N_dyn, Fr_l, Fr_r,Fleg,int_method, params, extra_forces);
    else
        dt_step = dt_dyn/cast(int_steps-1, "double");
        for i=1:N_dyn              
            if (i>=2)     
              [states_rough(:,i), t_rough(i)] = integrate_dynamics(states_rough(:,i-1), t_rough(i-1), dt_step, int_steps, ...
                                                Fr_l(i-1)*ones(1,int_steps), Fr_r(i-1)*ones(1,int_steps), Fleg, int_method, params, extra_forces(i-1)*ones(1,int_steps)); % keep Fr constant           
            else
              states_rough(:,i) = x0;
              t_rough(i) = t0;      
            end    
        end
    end




end