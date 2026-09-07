function X_out_list = adjoint_transform_list(T, X_list, to_space)
% Apply adjoint transform to every element in a cell array.

if nargin < 3
    to_space = true;
end

n = numel(X_list);
X_out_list = cell(size(X_list));
for i = 1:n
    X_out_list{i} = adjoint_transform(T, X_list{i}, to_space);
end
end
