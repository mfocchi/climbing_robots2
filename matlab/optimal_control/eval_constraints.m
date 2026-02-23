function eval_constraints(c, num_constr, constr_tolerance, debug)
    
if nargin <4
    debug = false;
end
    
wall_constraints_idx = 1;
retraction_force_constraints_idx = num_constr.wall_constraints;
force_constraints_idx = retraction_force_constraints_idx + num_constr.retraction_force_constraints;
final_point_constraints_idx = force_constraints_idx + num_constr.force_constraints;
via_point_idx = final_point_constraints_idx + num_constr.final_constraints;

if debug

    for i=1:length(c)
    
        if (i == wall_constraints_idx)
            disp('wall constraints')
        end
    
        if (i == retraction_force_constraints_idx+1)
            disp('retraction_force_constraints')
        end
    
        if (i == force_constraints_idx+1)
            disp('fleg force_constraints')
        end
    
        if (i == final_point_constraints_idx+1)
            disp('final_point_constraints')
        end
    
        if (i == via_point_idx+1)
            disp('via_point  constraints')
        end
        fprintf("%d %f\n",i,c(i))
    end

end

disp('EVAL CONSTRAINTS: positive  number represent constraint violation\n')
if any(c(wall_constraints_idx:  num_constr.wall_constraints)>constr_tolerance)
    disp('1)- wall constraints violated')
    disp(c(wall_constraints_idx:  num_constr.wall_constraints))
end     


if any( c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+num_constr.retraction_force_constraints)>constr_tolerance)
    disp('2) - retraction force constraints violated')
    disp( c(retraction_force_constraints_idx+1:retraction_force_constraints_idx + num_constr.retraction_force_constraints))
end  


if any(c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)>constr_tolerance)
    disp('3) - leg force constraints violated')
    disp('unilateral (Fun >fmin)')
    disp(c(  force_constraints_idx + 1))
    disp('actuation (Fun < fun max)')
    disp(c(  force_constraints_idx + 2))
    disp('friction  (|Fut| < mu*Fun)')
    disp(c(  force_constraints_idx + 3))
 
end


if any(c(final_point_constraints_idx+1: final_point_constraints_idx + num_constr.final_constraints)>constr_tolerance)
    disp('4) final point constraint violated, ')
    
    if c(final_point_constraints_idx+1) >constr_tolerance
        disp('4.1 p_f(y) < ymax_patch')
        disp(c(final_point_constraints_idx+1))
    end
    if c(final_point_constraints_idx+2) >constr_tolerance 
        disp('4.2 p_f(y) > ymin_patch')
        disp(c(final_point_constraints_idx+2))
    end
    if c(final_point_constraints_idx+3) >constr_tolerance
        disp('4.3 p_f(z) < zmax_patch ')
        disp(c(final_point_constraints_idx+3))
    end
    if c(final_point_constraints_idx+4) >constr_tolerance
        disp('4.4 p_f(z) > zin_patch')
        disp(c(final_point_constraints_idx+4))
    end
    if c(final_point_constraints_idx+5) >constr_tolerance
        disp('4.5 ||pf(x) -wall_x||<fixed_slack ')
        disp(c(final_point_constraints_idx+5))
    end
     
 
end


if any(c(via_point_idx+1: via_point_idx + num_constr.via_point)>constr_tolerance)
    disp('5) via point constraint violated')
    disp(c(via_point_idx+1: via_point_idx + num_constr.via_point))
 
end
    


    
    
    
end