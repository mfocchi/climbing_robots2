function [p,pd] = computePositionVelocity(params, psi, l1, l2, psid,l1d, l2d )
    px = zeros(1, length(psi));
    py = zeros(1, length(psi));
    pz = zeros(1, length(psi));
    pdx = zeros(1, length(psi));
    pdy = zeros(1, length(psi));
    pdz = zeros(1, length(psi));
    
    if nargin >4

        
        for i=1:length(psi)    
            [px(i), py(i), pz(i), pdx(i), pdy(i), pdz(i)] = forwardKin(params, psi(i), l1(i),l2(i), psid(i), l1d(i),l2d(i));
        end 
        pd = [pdx;pdy;pdz];
    else
        
        px = zeros(1, length(psi));
        py = zeros(1, length(psi));
        pz = zeros(1, length(psi));
        for i=1:length(psi)    
            [px(i), py(i), pz(i)] = forwardKin(params, psi(i), l1(i),l2(i));
        end 
        
    end    
         
    p = [px;py;pz];

       
end