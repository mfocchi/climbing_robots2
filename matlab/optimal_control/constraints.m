function [ineq, eq, number_of_constr, solution_constr] = constraints(x,   p0,  pf,  Fleg_max, Fr_max, mu, params)

 
% ineq are <= 0

Fleg = [ x(1); x(2); x(3)];
Tf = x(4);
Fr_l = x(params.num_params+1:params.num_params+params.N_dyn); 
Fr_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn); 


% check they are column vectors
p0 = p0(:);
pf = pf(:);
     
% size not known
ineq = zeros(1,0);

% number of constraints
number_of_constr.wall_constraints = params.N_dyn;
number_of_constr.retraction_force_constraints = 0;% already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes

if params.FRICTION_CONE
    number_of_constr.force_constraints  = 3;
else
    number_of_constr.force_constraints  = 2; %unilateral and actuation
end
number_of_constr.initial_final_constraints = 1;

if params.obstacle_avoidance
   number_of_constr.via_point = 0;
else
   number_of_constr.via_point = 1;
end

% variable intergration step
dt_dyn = Tf / (params.N_dyn-1);


% single shooting
state0 =  computeStateFromCartesian(params, p0);
[states, t] = computeRollout(state0, 0,dt_dyn, params.N_dyn, Fr_l, Fr_r,Fleg,params.int_method,params.int_steps,params);
psi = states(1,:);
l1 = states(2,:);
l2 = states(3,:);
psid = states(4,:);
l1d = states(5,:);
l2d = states(6,:); 
p = computePositionVelocity(params, psi, l1, l2); %only position
   
% I assume px py pz  are row vectors
p_0 = p(:, 1);
p_f = p(:,end);
 
% init struct foc C++ code generation
solution = struct;
solution_constr.p = p;
solution_constr.psi = psi;
solution_constr.l1 = l1;
solution_constr.l2 = l2;
solution_constr.psid = psid;
solution_constr.l1d = l1d;
solution_constr.l2d = l2d;
solution_constr.time = t;
solution_constr.final_error_discrete = norm(p(:,end) - pf);



% 1 -N_dyn  constraint to do not enter the wall, p_x >=0 

if params.obstacle_avoidance
    
    center = params.obstacle_location; %[0; 3;-7.5];
    radii = params.obstacle_size; 
    
    
    a_y = radii(1)^2/radii(2)^2;
    a_z = radii(1)^2/radii(3)^2;
    radius = radii(1);

     %px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2);
     %-px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0

    % better implementaiton than the one with with complex numbers for code
    % generation, we just check if the argument of the sqrt becomes
    % negative
    for i=1:params.N_dyn 
        arg  = radius.^2 - a_z*( p(3, i) -center(3)).^2 -a_y*(p(2, i)-center(2)).^2;
        %%%add ineq only if inside sphere
        if arg > 0
            ineq = [ineq  (-p(1, i) + center(1) + sqrt(arg) + params.jump_clearance)  ];   
        else 
            ineq = [ineq -p(1,i) ];   

        end

    end
else

    for i=1:params.N_dyn 
        ineq = [ineq -p(1,i) ];   
        %ineq = [ineq -psi(i) ]; 
    end
end



% % % debug
% disp('after wall')
% length(ineq)

% 2- N_dyn constraints on retraction force   -Fr_max < Fr < 0 
% unilaterality

if number_of_constr.retraction_force_constraints>0
    for i=1:params.N_dyn     
         ineq = [ineq  Fr_l(i) ]; % Fr_l <0

    end 
    for i=1:params.N_dyn     
         ineq = [ineq  Fr_r(i) ]; % Fr_r <0

    end 

    % max force
    for i=1:params.N_dyn 
      ineq = [ineq (-Fr_max - Fr_l(i))];    % -Fr_max -Fr_l <0
    end
    for i=1:params.N_dyn 
      ineq = [ineq (-Fr_max - Fr_r(i))];    % -Fr_max -Fr_r <0
    end
end

% debug
% disp('after Fr')
% length(ineq)

% constraints on impulse force
contact_tang_y = cross(cross(params.contact_normal, [0;1;0]),params.contact_normal); 
contact_tang_z = cross(cross(params.contact_normal, [0;0;1]),params.contact_normal); 
% compute components
Fun = params.contact_normal'*Fleg;
Futy = contact_tang_y'*Fleg;
Futz = contact_tang_z'*Fleg;
Fut_norm = sqrt(Futy^2 +Futz^2);

Fun_min = 0;

%3 ------------------------------ Fleg constraints

% unilateral
ineq = [ineq  (-Fun + Fun_min)]  ; %(Fun >fmin ) 
%max force 
ineq = [ineq  (norm(Fleg) -Fleg_max)]   ;%(Fun < fun max ) actuation

if params.FRICTION_CONE
    ineq = [ineq  (Fut_norm -mu*Fun)]; %friction constraints
end
% 
% % debug
% disp('after Fu')
% length(ineq)

% final point  variable slack  
%ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)];
% 4- initial final point  fixed slack 
fixed_slack = 0.02;%*norm(p0 - pf); 

if number_of_constr.initial_final_constraints == 2
    ineq= [ineq (norm(p_0 - p0) - fixed_slack)];
    ineq= [ineq (norm(p_f - pf) - fixed_slack)];
end

if number_of_constr.initial_final_constraints == 1
        ineq= [ineq (norm(p_f - pf) - fixed_slack)];
end


%5 - jump clearance

if number_of_constr.via_point >0       
    ineq = [ineq (-p(1,params.N_dyn/2) +params.jump_clearance) ];   

end

eq = [];


% if any(isinf(ineq))
%     disp('Infn in constraint')
%     find(isinf(ineq)) 
%     isinf(ineq)
% end
% if any(isnan(ineq))
%     disp('Nan in constraint')
%     find(isnan(ineq))
%     isnan(ineq)
% end

end