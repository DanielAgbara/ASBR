function T_ee = body_product_of_transforms(M, T_list)
% Computes T = M * T1 * T2 * ... * Tn.

T = M;
for i = 1:numel(T_list)
    T = T * T_list{i};
end
T_ee = T;
end
