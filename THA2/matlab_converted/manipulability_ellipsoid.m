function data = manipulability_ellipsoid(J)
% Manipulability ellipsoid data for angular and linear parts.

J_w = J(1:3, :);
J_v = J(4:6, :);

A_w = J_w * J_w.';
[eigvecs_w, D_w] = eig(A_w);
eigvals_w = diag(D_w);
if min(eigvals_w) < 1e-12
    mu1_w = inf; mu2_w = inf;
else
    mu1_w = sqrt(max(eigvals_w) / min(eigvals_w));
    mu2_w = max(eigvals_w) / min(eigvals_w);
end
mu3_w = sqrt(det(A_w));

A_v = J_v * J_v.';
[eigvecs_v, D_v] = eig(A_v);
eigvals_v = diag(D_v);
if min(eigvals_v) < 1e-12
    mu1_v = inf; mu2_v = inf;
else
    mu1_v = sqrt(max(eigvals_v) / min(eigvals_v));
    mu2_v = max(eigvals_v) / min(eigvals_v);
end
mu3_v = sqrt(det(A_v));

data = {{A_w, eigvals_w, eigvecs_w, mu1_w, mu2_w, mu3_w}, ...
        {A_v, eigvals_v, eigvecs_v, mu1_v, mu2_v, mu3_v}};
end
