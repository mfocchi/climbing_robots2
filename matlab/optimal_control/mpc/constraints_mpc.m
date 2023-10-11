function [ineq, eq]= constraints_mpc(x, Fr_max, Fr_l0, Fr_r0,mpc_N)

    % size  known
    ineq = zeros(1,2*mpc_N);
    
    % init for cpp  
    delta_Fr_l = x(1:mpc_N); 
    delta_Fr_r = x(mpc_N+1:2*mpc_N); 
    
    % fr + delta F <=0
    for i=1:mpc_N 
        ineq = [ineq (Fr_l0(i) + delta_Fr_l(i))  ];   
        ineq = [ineq (Fr_r0(i) + delta_Fr_r(i))  ];   
    end
      % this creates issues!!!! and does not make to converge
%     % -Frmax < fr + delta F  => -(fr + delta F) -Frmax < 0 
%     for i=1:mpc_N 
%         ineq = [ineq -(Fr_l0(i) + delta_Fr_l(i)) -Fr_max  ];   
%         ineq = [ineq -(Fr_r0(i) + delta_Fr_r(i)) -Fr_max  ]; 
%     end
%     
    eq = [];
end