function [px, py, pz, pdx, pdy, pdz] = forwardKin(psi, l1, l2, psid, l1d,l2d)
global b 
    px =     l1*sin(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2);
    py =         (b^2 + l1^2 - l2^2)/(2*b);
    pz =     -l1*cos(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2);    
    
    
    if nargin>3
        px_l1 = px/l1;
        n_pz_l1 =  -pz/l1;
        px_l1_sinpsi = px/l1/sin(psi);
        py2b = py*2*b;

        pdx =   l1d*px_l1 + l1*n_pz_l1*psid + (py2b*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(4*b^2*l1^2*px_l1_sinpsi);
        pdy =                                                                                              (l1*l1d - l2*l2d)/b;
        pdz  = l1*psid*px_l1 - l1d*n_pz_l1 - (py2b*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(4*b^2*l1^2*px_l1_sinpsi);
    end
end