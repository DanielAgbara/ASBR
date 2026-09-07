function [w_hat, theta] = RToAxisAngle(R, eps_val)
% Convert rotation matrix to axis-angle representation

    if nargin < 2
        eps_val = 1e-6;
    end

    R = double(R);
    tr_R = trace(R);
    I = eye(3);

    % Case I: R = I
    if norm(R - I, 'fro') < eps_val
        w_hat = [0; 0; 1];
        theta = 0.0;
        return;
    end

    % Case II: tr(R) = -1
    if abs(tr_R + 1) < eps_val
        diag_vals = 1 + diag(R);
        [~, i] = max(diag_vals);

        theta = pi;

        w_hat = R(:, i);
        w_hat(i) = w_hat(i) + 1.0;

        denom = sqrt(max(2 * (1 + R(i, i)), eps_val));
        w_hat = w_hat / denom;
    else
        % Case III: general case
        c = (tr_R - 1) / 2;
        c = min(max(c, -1), 1);
        theta = acos(c);

        if abs(sin(theta)) < eps_val
            w_hat = [0;0;1];
            theta = 0.0;
            return;
        end

        W = (R - R.') / (2 * sin(theta));
        w_hat = [W(3,2); W(1,3); W(2,1)];
    end

    n = norm(w_hat);
    if n < eps_val
        w_hat = [0;0;1];
    else
        w_hat = w_hat / n;
    end
end