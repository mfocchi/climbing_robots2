function  state = computeStateFromCartesian(p,pd)
global p_a1 p_a2

    psi = atan2(p(1), -p(3));
    l1 = norm(p(:) - p_a1(:));
    l2 = norm(p(:) - p_a2(:));


    
    if nargin >1 % we have also velocity
         % I could do it si=ymbolically 
%         pd_x = l1d*px_l1 + l1*n_pz_l1*psid + (py2b*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(4*b^2*l1^2*px_l1_sinpsi);
%         pd_y =    (l1*l1d - l2*l2d)/b;
%         pd_z = l1*psid*px_l1 - l1d*n_pz_l1 - (py2b*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(4*b^2*l1^2*px_l1_sinpsi);
%     but I prefer to use geometric intuition TODO check this!

        n_par = (p_a1(:) - p_a2(:))/norm(p_a1(:) - p_a2(:));
        n_perp = eye(3) - n_par*n_par'; 
        
        phid = cross(n_par, n_perp)'*pd / cross(n_par,n_perp)'*(p_a1(:)-p_a2(:));
        % for the speed I just project along rope axis
        l1d = (p(:) - p_a1(:))'*pd(:);
        l2d = (p(:) - p_a2(:))'*pd(:);        
    else
            phid = 0;
            l1d = 0.0;
            l2d = 0.0;
    end
    
    state = [psi; l1; l2; phid; l1d; l2d];   
    
end