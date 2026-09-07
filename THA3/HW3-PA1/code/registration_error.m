function err = registration_error(T, M, S_c)

    M_reg = T(1:3, 1:3) * M + T(1:3, 4);
    D = M_reg - S_c;
    err = sqrt(mean(D(:).^2));
end