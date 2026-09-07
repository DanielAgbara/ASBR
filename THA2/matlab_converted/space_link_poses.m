function T_list = space_link_poses(M_list, S_list, theta)
% Computes link poses in the space frame.

theta = theta(:);
T_list = cell(1, numel(M_list));
T_prefix = eye(4);
T_list{1} = T_prefix * M_list{1};

for i = 1:numel(S_list)
    T_prefix = T_prefix * exp_screw_axis(S_list{i}, theta(i));
    T_list{i+1} = T_prefix * M_list{i+1};
end
end
