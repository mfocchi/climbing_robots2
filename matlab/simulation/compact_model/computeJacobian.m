function J = computeJacobian(p)
 global  p_a1 p_a2
 J = [ (p(:)-p_a1(:))/norm(p(:)-p_a1(:)) ,(p(:)-p_a2(:))/norm(p(:)-p_a2(:))];

end