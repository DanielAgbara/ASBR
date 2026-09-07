function mu = manipulability(J)
% Manipulability measure sqrt(det(JJ^T)).

A = J * J.';
mu = sqrt(det(A));
end
