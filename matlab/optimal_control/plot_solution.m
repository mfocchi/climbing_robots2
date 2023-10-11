function plot_solution(solution,p0, pf,Fleg_max,Fr_max,mu, params)

    figure
%     x(1:3) = solution.Fleg;
%     x(4) =solution.Tf;
%     x(params.num_params+1:params.num_params+params.N_dyn) =solution.Fr_l;
%     x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn)=solution.Fr_r;
%     [c ceq, num_constr, solution_constr] = constraints(x, p0(:), pf(:),  Fleg_max, Fr_max, mu, params);
    plot_curve( solution,solution.solution_constr, p0(:), pf(:), mu,  false, 'r', true, params);

    
    figure
    subplot(2,1,1)           
    plot(solution.time, solution.Fr_l, 'ko-'); grid on;hold on;  ylabel('Frl'); grid on;hold on;
    subplot(2,1,2)   
    plot(solution.time, solution.Fr_r, 'bo-'); grid on;hold on;  ylabel('Frr'); grid on;hold on;

        figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution.solution_constr.time, solution.solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution.solution_constr.time, solution.solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution.solution_constr.time, solution.solution_constr.p(3,:),'ob') ; hold on;
    ylabel('Z')
    
    
end