function [T, S_c, idx_c] = iterative_closest_point(M, S, max_iter, tol)
% 
% Inputs:
%           M: moving model set (3xn)
%           S: fixed/statac scene set (3xm)

T = maximum_likelihood(M, S);

S_c = zeros(size(M));
idx_c = zeros(1, size(M, 2));
err_prev = inf;
for i = 1 : max_iter
    
    for j = 1 : size(M, 2)
        M_j = T(1:3, 1:3) * M(:, j) + T(1:3, 4);
        [S_c(:, j), ~, idx_c(j)] = nearest_neighbor(M_j, S); %getting correspondence
    end

    err = registration_error(T, M, S_c);
    if err < tol || (0.95 < err/err_prev && err/err_prev < 1)
        break
    end
    T = maximum_likelihood(M, S_c);
    err_prev = err;
end
%disp(i);
end