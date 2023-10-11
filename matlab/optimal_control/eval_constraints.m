function eval_constraints(c, num_constr, constr_tolerance)
    
wall_constraints_idx = 1;
retraction_force_constraints_idx = num_constr.wall_constraints;
force_constraints_idx = retraction_force_constraints_idx + num_constr.retraction_force_constraints;
final_point_constraints_idx = force_constraints_idx + num_constr.force_constraints;
via_point_idx = final_point_constraints_idx + num_constr.initial_final_constraints;

% 
% for i=1:length(c)
%     
%     if (i == wall_constraints_idx)
%         disp('wall constraints')
%     end
% 
%     if (i == retraction_force_constraints_idx+1)
%         disp('retraction_force_constraints')
%     end
% 
%     if (i == force_constraints_idx+1)
%         disp('fleg force_constraints')
%     end
% 
%     if (i == final_point_constraints_idx+1)
%         disp('final_point_constraints')
%     end
% 
%     if (i == via_point_idx+1)
%         disp('via_point  constraints')
%     end
%     fprintf("%d %f\n",i,c(i))
% end



if any(c(wall_constraints_idx:  num_constr.wall_constraints)>constr_tolerance)
    disp('1- wall constraints violated')
    c(1:  num_constr.wall_constraints)
end     


if any( c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+num_constr.retraction_force_constraints)>constr_tolerance)
    disp('2- retraction force constraints violated')
     c(retraction_force_constraints_idx+1:retraction_force_constraints_idx + num_constr.retraction_force_constraints)
end  


if any(c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)>constr_tolerance)
    disp('3 -force constraints violated (unilateral flegx>0,max actuation  , (friction) ')
    c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)
 
end


if any(c(final_point_constraints_idx+1: final_point_constraints_idx + num_constr.initial_final_constraints)>constr_tolerance)
    disp('4 final point ')
    c(final_point_constraints_idx+1: final_point_constraints_idx + num_constr.initial_final_constraints)
 
end


if any(c(via_point_idx+1: via_point_idx + num_constr.via_point)>constr_tolerance)
    disp('4 via point ')
    c(via_point_idx+1: via_point_idx + num_constr.via_point)
 
end
    


    
    
    
end