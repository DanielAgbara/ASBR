function X_out = adjoint_transform(T, X, to_space)
% Transform twist coordinates using Ad_T or Ad_T^{-1}.

if nargin < 3
    to_space = true;
end

X = X(:);
if to_space
    X_out = adjoint(T) * X;
else
    X_out = adjoint_inverse(T) * X;
end
end
