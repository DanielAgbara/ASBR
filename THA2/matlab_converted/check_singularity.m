function is_singular = check_singularity(J)
% True if Jacobian is rank deficient.

is_singular = rank(double(J)) < min(size(J));
end
