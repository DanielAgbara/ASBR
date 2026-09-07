function vis = initTubularImpedanceVisualizer(curve_vis, p, R, view_option)

fig = figure(1); clf;
screen = get(0, 'ScreenSize');

target_ratio = 6/2;

W = screen(3);
H = W / target_ratio;

set(fig, 'Position', [100, 100, W, H]);
switch lower(view_option)
    case 'xy'
        t = tiledlayout(1,1);
        axs = gobjects(1,1);

        axs(1) = nexttile(t);

        view_list = {[0 90]};
        title_list = {'Top View: XY'};

    case '3d'
        t = tiledlayout(1,1);
        axs = gobjects(1,1);

        axs(1) = nexttile(t);

        view_list = {3};
        title_list = {'3D View'};

    case 'all'
        t = tiledlayout(2,6, 'TileSpacing','compact','Padding','compact');

        axs = gobjects(4,1);

        axs(1) = nexttile(t,1, [1, 2]); % top
        axs(2) = nexttile(t,4, [2, 3]); % 3D
        axs(3) = nexttile(t,7, [1, 2]); % front
        axs(4) = nexttile(t,9, [1, 1]); % right

        ax_leg = nexttile(t, 3);
        
        axis(ax_leg, 'off');
        set(ax_leg, 'Visible','off');

        view_list = {
            [0 90], ...   % top / XY
            3, ...        % 3D
            [0 0], ...    % front / XZ
            [90 0] ...    % right / YZ
        };

        title_list = {
            'Top View: XY', ...
            '3D View', ...
            'Front View: XZ', ...
            'Right View: YZ'
        };


    otherwise
        error("view_option must be 'xy', '3d', or 'all'.");
end

handles = cell(length(axs),1);

for i = 1:length(axs)
    ax = axs(i);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'tight');
    if i == 2
        axis(ax, 'equal');
    end
    

    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, title_list{i});

    plot3(ax, curve_vis(1,:), curve_vis(2,:), curve_vis(3,:), ...
        'k--', 'LineWidth', 1, ...
        'DisplayName', 'curve');

    [tube_X, tube_Y, tube_Z] = makeTubeMesh(curve_vis, R);

    surf(ax, tube_X, tube_Y, tube_Z, ...
        'FaceAlpha', 0.15, ...
        'EdgeAlpha', 0.10, ...
        'FaceColor', [0.5 0.5 0.5], ...
        'EdgeColor', [0.5 0.5 0.5], ...
        'DisplayName', 'tube mesh');

    h.traj = animatedline(ax, ...
        'Color','b', ...
        'LineWidth',1, ...
        'DisplayName','trajectory');

    h.p = plot3(ax, p(1), p(2), p(3), 'ro', ...
        'MarkerFaceColor','r', ...
        'MarkerSize',8, ...
        'DisplayName','end-effector');

    h.pc = plot3(ax, p(1), p(2), p(3), 'ko', ...
        'MarkerFaceColor','y', ...
        'MarkerSize',6, ...
        'DisplayName','closest point');

    h.u = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'Color','g', ...
        'LineWidth',2, ...
        'DisplayName','u: tangent');

    % h.d = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
    %     'm', ...
    %     'LineWidth',2, ...
    %     'DisplayName','d: toward curve');

    h.Fin = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'r', ...
        'LineWidth',2, ...
        'DisplayName','input F');

    % h.Fattr = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
    %     'c', ...
    %     'LineWidth',2, ...
    %     'DisplayName','F attraction');

    h.Fspring = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'Color','magenta', ...
        'LineWidth',2, ...
        'DisplayName','F spring: kD');

    h.Fdamp = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'Color',[1.0 0.5 0.0], ...
        'LineWidth',2, ...
        'DisplayName','F damping: -Bv');

    h.Ftan = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'g', ...
        'LineWidth',2, ...
        'DisplayName','F tangent');

    h.Fout = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'b', ...
        'LineWidth',2.0, ...
        'DisplayName','output F');

    % h.Fact = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
    %     'k', ...
    %     'LineWidth',2.0, ...
    %     'DisplayName','actual F');



    xlim(ax, [-0.5 10.5]);
    ylim(ax, [-3 3]);
    zlim(ax, [-3 3]);

    title(ax, title_list{i});
    view(ax, view_list{i});

    handles{i} = h;
    set(ax, 'FontSize', 15);
end

vis.axs = axs;
vis.handles = handles;

vis.scale.u = 1.0;
vis.scale.d = 1.0;
vis.scale.force = 1.0;

h0 = handles{1};

legend(ax_leg, ...
    [h0.traj, h0.p, h0.pc, h0.u, ...
     h0.Fin, h0.Fspring, h0.Fdamp, h0.Ftan, h0.Fout], ...
    {'trajectory','end-effector','closest point', 'u', ...
     'F Inpput', 'F spring: kD','F damping: -Bv','F tangent','F Overall'}, ...
    'Location','layout');

axis(ax_leg, 'off');
set(ax_leg, 'Visible','off');
set(ax_leg, 'FontSize', 20);

end

%% Local helper
function [tube_X, tube_Y, tube_Z] = makeTubeMesh(curve, R)

% curve: (3xN)
N = size(curve,2);

n_circle = 32;
theta = linspace(0, 2*pi, n_circle);

tube_X = zeros(N, n_circle);
tube_Y = zeros(N, n_circle);
tube_Z = zeros(N, n_circle);

% Tangent vectors
T = zeros(3,N);

for i = 1:N
    if i == 1
        t = curve(:,i+1) - curve(:,i);
    elseif i == N
        t = curve(:,i) - curve(:,i-1);
    else
        t = curve(:,i+1) - curve(:,i-1);
    end
    T(:,i) = t / norm(t);
end

% Initial normal
ref = [0;0;1];
if abs(dot(T(:,1), ref)) > 0.95
    ref = [0;1;0];
end

Nvec = cross(T(:,1), ref);
Nvec = Nvec / norm(Nvec);

Bvec = cross(T(:,1), Nvec);
Bvec = Bvec / norm(Bvec);

normal_frame_N = zeros(3,N);
normal_frame_B = zeros(3,N);

normal_frame_N(:,1) = Nvec;
normal_frame_B(:,1) = Bvec;

% Propagate frame
for i = 2:N
    t_prev = T(:,i-1);
    t_curr = T(:,i);

    axis_rot = cross(t_prev, t_curr);
    axis_norm = norm(axis_rot);

    if axis_norm < 1e-12
        normal_frame_N(:,i) = normal_frame_N(:,i-1);
    else
        axis_rot = axis_rot / axis_norm;
        angle_rot = atan2(axis_norm, dot(t_prev, t_curr));

        normal_frame_N(:,i) = rotateVectorRodrigues( ...
            normal_frame_N(:,i-1), axis_rot, angle_rot);
    end

    normal_frame_N(:,i) = normal_frame_N(:,i) ...
        - dot(normal_frame_N(:,i), t_curr) * t_curr;
    normal_frame_N(:,i) = normal_frame_N(:,i) / norm(normal_frame_N(:,i));

    normal_frame_B(:,i) = cross(t_curr, normal_frame_N(:,i));
    normal_frame_B(:,i) = normal_frame_B(:,i) / norm(normal_frame_B(:,i));
end

% Mesh
for i = 1:N
    for j = 1:n_circle
        p = curve(:,i) ...
            + R*cos(theta(j))*normal_frame_N(:,i) ...
            + R*sin(theta(j))*normal_frame_B(:,i);

        tube_X(i,j) = p(1);
        tube_Y(i,j) = p(2);
        tube_Z(i,j) = p(3);
    end
end

end

function v_rot = rotateVectorRodrigues(v, k, theta)

v_rot = v*cos(theta) ...
    + cross(k, v)*sin(theta) ...
    + k*dot(k, v)*(1 - cos(theta));

end