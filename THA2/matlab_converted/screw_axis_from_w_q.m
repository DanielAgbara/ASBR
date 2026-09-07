function V = screw_axis_from_w_q(w, q)
% Revolute screw axis V = [w; v], where v = -w x q.

w = double(w(:));
q = double(q(:));
V = [w; -skew(w) * q];
end
