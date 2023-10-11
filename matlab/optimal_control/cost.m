function cost = cost(x, p0,  pf, params)

    Fleg = [ x(1); x(2); x(3)];
    Tf = x(4);
    Fr_l = x(params.num_params+1:params.num_params+params.N_dyn); 
    Fr_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn); 

    % check they are column vectors
    p0 = p0(:);
    pf = pf(:);
    % variable intergration step
    dt_dyn = Tf / (params.N_dyn-1); 
    

    % single shooting
    state0 =  computeStateFromCartesian(params,p0);
    [states, t] = computeRollout(state0, 0,dt_dyn, params.N_dyn, Fr_l, Fr_r,Fleg,params.int_method,params.int_steps,params);

    psi = states(1,:);
    l1 = states(2,:);
    l2 = states(3,:);
    psid = states(4,:);
    l1d = states(5,:);
    l2d = states(6,:); 
    [p, pd ]= computePositionVelocity(params,psi, l1, l2, psid,l1d, l2d);
    
    p_0 = p(:,1);
    p_f = p(:,end);

    % be careful there are only N values in this vector the path migh be
    % underestimated!
%     deltax = diff(p(1,:));  % diff(X);
%     deltay = diff(p(2,:));   % diff(Y);
%     deltaz = diff(p(3,:));    % diff(Z);
%     path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

    p_0 = p_0(:);
    p0 = p0(:);
    p_f= p_f(:);
    pf = pf(:);
    
    %minimize the final kin energy at contact
    Ekinfcost=  params.m/2 * (params.contact_normal'*pd(:,end))*params.contact_normal'*pd(:,end);
      
    % minimize hoist work / energy consumption for the hoist work we integrathe the power on a rough grid
    hoist_work = sum(abs(Fr_l.*l1d)*dt_dyn) + sum(abs(Fr_r.*l2d)*dt_dyn);  %assume the motor is not regenreating
    
     
    
    % smoothnes: minimize jerky control action TODO this is wrong! it goes
    % to -180 and stays there! with sum(abs(diff(Fr_r))) +
    % sum(abs(diff(Fr_l))) but does not converge at all 
    smooth_correct = sum(diff(Fr_r).^2)+ sum(diff(Fr_l).^2); % this is nice but slower
    smooth = sum(diff(Fr_r)) + sum(diff(Fr_l));
    
    
    %fprintf("hoist_work %f\n ",hoist_work)    
    %fprintf("smooth %f\n ", smooth)
    %fprintf("tempo %f\n ", w6*Tf)

     
    %cost =  0.001 * params.w1 *Ekinfcost +   params.w4 *smooth ;% converge
    %super slowly
    cost =  params.w2 *hoist_work +   params.w1 *smooth ;% 72 iter
   % cost =    params.w4 *smooth ;% 27 iter
   % cost =    params.w4 *smooth_correct ;% 96 iter
end