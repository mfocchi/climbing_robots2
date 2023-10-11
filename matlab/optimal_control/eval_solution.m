function solution = eval_solution(x,  dt, p0, pf, params)


%eval trajectory
Fleg = [ x(1); x(2); x(3)];
Tf = x(4);
Fr_l = x(params.num_params+1:params.num_params+params.N_dyn); 
Fr_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn); 



% resample inputs 
n_samples = floor(Tf/dt);
Fr_l_fine = zeros(1,n_samples);
Fr_r_fine = zeros(1,n_samples);
rough_count = 1;
t_ = 0;
for i=1: n_samples
   t_= t_+dt;
   if t_>= ((n_samples) *dt/(params.N_dyn-1))
        rough_count = rough_count + 1;
        t_ =0;
    end  
    Fr_l_fine(i) =  Fr_l(rough_count);
    Fr_r_fine(i) =  Fr_r(rough_count);
end

% single shooting

state0 =  computeStateFromCartesian(params, p0);

% course integration
dt_dyn = Tf / (params.N_dyn-1); 
[states, t] = computeRollout(state0, 0,dt_dyn, params.N_dyn, Fr_l, Fr_r,Fleg,params.int_method,params.int_steps, params);

psi = states(1,:);
l1 = states(2,:);
l2 = states(3,:);
psid = states(4,:);
l1d = states(5,:);
l2d = states(6,:); 
[p, pd ]= computePositionVelocity(params, psi, l1, l2, psid,l1d, l2d);
   
% fine integration 
[states_fine, t_fine] = computeRollout(state0, 0,dt, n_samples, Fr_l_fine, Fr_r_fine,Fleg, params.int_method, 0, params);
psi_fine = states_fine(1,:);
l1_fine = states_fine(2,:);
l2_fine = states_fine(3,:);
psid_fine = states_fine(4,:);
l1d_fine = states_fine(5,:);
l2d_fine = states_fine(6,:); 
[p_fine, pd_fine ]= computePositionVelocity(params, psi_fine, l1_fine, l2_fine, psid_fine,l1d_fine, l2d_fine);
   

% init struct foc C++ code generation
solution = struct;

%compute path
deltax = diff(p(1,:));  % diff(X);
deltay = diff(p(2,:));   % diff(Y);
deltaz = diff(p(3,:));    % diff(Z);
solution.path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

% check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l

solution.initial_error = norm(p(:,1) -p0);
solution.final_error_real = norm(p(:,end) -pf);

solution.Fleg = Fleg;
solution.Fr_l  = Fr_l;
solution.Fr_r  = Fr_r;
solution.p = p;
solution.psi = psi;
solution.l1 = l1;
solution.l2 = l2;
solution.psid = psid;
solution.l1d = l1d;
solution.l2d = l2d;
solution.time = t;

solution.Fr_l_fine  = Fr_l_fine;
solution.Fr_r_fine  = Fr_r_fine;
solution.p_fine = p_fine;
solution.psi_fine = psi_fine;
solution.l1_fine = l1_fine;
solution.l2_fine = l2_fine;
solution.psid_fine = psid_fine;
solution.l1d_fine = l1d_fine;
solution.l2d_fine = l2d_fine;
solution.time_fine = t_fine;


solution.Tf = Tf;
solution.achieved_target =  p_fine(:,end);


solution.Etot = 0;
solution.Ekin = zeros(1, length(t_fine));
solution.Ekin0x = 0;
solution.Ekin0y = 0;
solution.Ekin0z = 0;
solution.Ekin0 = 0;
solution.intEkin = 0;
solution.U0 = 0;
solution.Uf = 0;

% kinetic energy at the beginning
solution.Ekin0x = params.m/2*pd_fine(1,1)'*pd_fine(1,1);
solution.Ekin0y = params.m/2*pd_fine(2,1)'*pd_fine(2,1);
solution.Ekin0z = params.m/2*pd_fine(3,1)'*pd_fine(3,1);
solution.Ekin0 = params.m/2*pd_fine(:,1)'*pd_fine(:,1);

solution.Ekinfx = params.m/2*pd_fine(1,end)'*pd_fine(1,end);
solution.Ekinfy = params.m/2*pd_fine(2,end)'*pd_fine(2,end);
solution.Ekinfz = params.m/2*pd_fine(3,end)'*pd_fine(3,end);
solution.Ekinf = params.m/2*pd_fine(:,end)'*pd_fine(:,end);


for i =1:length(t_fine)
    solution.Ekin(i) = params.m/2*pd_fine(:,i)'*pd_fine(:,i);
    solution.intEkin = solution.intEkin +  solution.Ekin(i)*dt;
end
    


end