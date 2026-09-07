function updateTubularImpedanceVisualizer(vis, p, info, F_input, F_output, F_actual)

for i = 1:length(vis.handles)
    h = vis.handles{i};

    addpoints(h.traj, p(1), p(2), p(3));

    set(h.p, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
    
    set(h.pc, 'XData', info.p_c(1), 'YData', info.p_c(2), 'ZData', info.p_c(3));

    setQuiver3D(h.u,info.p_c, vis.scale.u * info.u);
    % setQuiver3D(h.d, p, vis.scale.d * info.d);

    % Input / output forces
    setQuiver3D(h.Fin, p, vis.scale.force * F_input);
    setQuiver3D(h.Fout, p, vis.scale.force * F_output);
    % setQuiver3D(h.Fact, p, vis.scale.force * F_actual);

    % Decomposed forces
    setQuiver3D(h.Fspring, p, vis.scale.force * info.F_spring);   % k*D
    setQuiver3D(h.Fdamp, p, vis.scale.force * info.F_damping);  % -B*v
    setQuiver3D(h.Ftan, p,vis.scale.force * info.F_tangent);

    % (optional) total attraction
    % if isfield(info, 'F_attraction')
    % setQuiver3D(h.Fattr, p, vis.scale.force * info.F_attraction);
    % end
end

end

%% Local helper
function setQuiver3D(h, p, v)

set(h, ...
    'XData', p(1), ...
    'YData', p(2), ...
    'ZData', p(3), ...
    'UData', v(1), ...
    'VData', v(2), ...
    'WData', v(3));

end