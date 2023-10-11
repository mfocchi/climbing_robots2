function [fleg] = evalImpulse(t, Fleg, params)

    if (t <= params.T_th)
        fleg = Fleg(:);% make sure is a column vector
    else
        fleg = [0;0;0];
    end
  
end