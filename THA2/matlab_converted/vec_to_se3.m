function hat_V = vec_to_se3(V)
% Converts twist coordinates V = [w; v] to se(3) matrix.

V = double(V(:));
hat_V = zeros(4, 4);
hat_V(1:3, 1:3) = skew(V(1:3));
hat_V(1:3, 4) = V(4:6);
end
