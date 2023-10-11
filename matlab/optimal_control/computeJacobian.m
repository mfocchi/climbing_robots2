function J = computeJacobian(p, params)
 
 J = [ (p(:)-params.p_a1(:))/norm(p(:)-params.p_a1(:)) ,(p(:)-params.p_a2(:))/norm(p(:)-params.p_a2(:))];

end