function err = compute_point_error(p_est, p_true)

    p_est = p_est(:);
    p_true = p_true(:);

    err.vec = p_est - p_true;
    err.abs = abs(err.vec);
    err.norm = norm(err.vec);
    err.rms = sqrt(mean(err.vec.^2));
    err.max_abs = max(err.abs);
end