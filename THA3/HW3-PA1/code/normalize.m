function [v_unit, v_norm] = normalize(v)
% Normalize a vector
    v = double(v(:));
    v_norm = norm(v);
    if v_norm == 0
        v_unit = v;
    else
        v_unit = v / v_norm;
    end
end