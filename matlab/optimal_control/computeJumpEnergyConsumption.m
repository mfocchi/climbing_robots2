function [impulse_work , hoist_work, hoist_work_fine] = computeJumpEnergyConsumption(solution, params)

    % this function is not used, is implemented just as a reference for python
    % we compute energy consumption in python
    dt_dyn = solution.Tf / (params.N_dyn-1); 
    %%Energy consumption
    J_TO_Wh = 0.000277 %maps joule to Wh
    impulse_end_idx = max(find(solution.time<=solution.T_th));
    % the work is the kinetic energy at the end of the thrusting
    impulse_work =   solution.Ekin(impulse_end_idx); %params.m/2*solution.pd(:,impulse_end_idx)'*solution.pd(:,impulse_end_idx);
    %for the hoist work we integrathe the ppowet on a rough grid
    hoist_work = 0;
    for i=1:length(solution.time)
        hoist_work = hoist_work + (abs(solution.Fr_l(i).*solution.l1d(i)) + abs(solution.Fr_r(i).*solution.l2d(i)))* dt_dyn;  %assume the motor is not regenreating
    end
    
    
    % more precise
    dt = solution.time_fine(2)-solution.time_fine(1);
    hoist_work_fine = 0;
    for i=1:length(solution.time_fine)
        hoist_work_fine = hoist_work_fine + (abs(solution.Fr_l_fine(i).*solution.l1d_fine(i)) + abs(solution.Fr_r_fine(i).*solution.l2d_fine(i)))* dt;  %assume the motor is not regenreating
    end
    
%     impulse_workWh = J_TO_Wh*impulse_work
%     hoist_workWh=J_TO_Wh*hoist_work

end

