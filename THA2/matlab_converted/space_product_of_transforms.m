function T_ee = space_product_of_transforms(M, T_list)
% Computes T = T1 * T2 * ... * Tn * M.

T = eye(4);
for i = 1:numel(T_list)
    T = T * T_list{i};
end
T_ee = T * M;
end
