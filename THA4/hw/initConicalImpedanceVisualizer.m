function vis = initConicalImpedanceVisualizer(x_s, x_t, alpha, p, view_option)

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
            [0 90], ...
            3, ...
            [0 0], ...
            [90 0]
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

    %% Draw cone
    [cone_X, cone_Y, cone_Z] = makeConeMesh(x_s, x_t, alpha);

    surf(ax, cone_X, cone_Y, cone_Z, ...
        'FaceAlpha', 0.18, ...
        'EdgeAlpha', 0.15, ...
        'FaceColor', [0.6 0.6 0.6], ...
        'EdgeColor', [0.4 0.4 0.4], ...
        'DisplayName', 'virtual cone');

    %% Cone axis
    plot3(ax, [x_s(1), x_t(1)], ...
              [x_s(2), x_t(2)], ...
              [x_s(3), x_t(3)], ...
        'k--', 'LineWidth', 1, ...
        'DisplayName', 'cone axis');

    %% Start and target points
    plot3(ax, x_s(1), x_s(2), x_s(3), 'ks', ...
        'MarkerFaceColor','k', ...
        'MarkerSize',7, ...
        'DisplayName','start');

    plot3(ax, x_t(1), x_t(2), x_t(3), 'kp', ...
        'MarkerFaceColor','y', ...
        'MarkerSize',10, ...
        'DisplayName','target');

    %% Moving objects
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

    %% Vector handles
    h.u = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
        'Color','g', ...
        'LineWidth',2, ...
        'DisplayName','u: preferred direction');

    % h.d = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
    %     'm', ...
    %     'LineWidth',2, ...
    %     'DisplayName','d: toward axis');

    % h.n_perp = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
    %     'Color',[0.85 0.2 0.2], ...
    %     'LineWidth',2, ...
    %     'DisplayName','n_\perp');
    % 
    % h.n_parallel = quiver3(ax, p(1), p(2), p(3), 0, 0, 0, 0, ...
    %     'Color',[0.2 0.6 0.2], ...
    %     'LineWidth',2, ...
    %     'DisplayName','n_\parallel');

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

    %% Axis limits
    all_pts = [
    [cone_X(:), cone_Y(:), cone_Z(:)];
    x_s';
    x_t';
    p'];
    margin = 1.0;

    xlim(ax, [min(all_pts(:,1))-margin, max(all_pts(:,1))+margin]);
    ylim(ax, [min(all_pts(:,2))-margin, max(all_pts(:,2))+margin]);
    zlim(ax, [min(all_pts(:,3))-margin, max(all_pts(:,3))+margin]);

    view(ax, view_list{i});

    handles{i} = h;
    set(ax, 'FontSize', 15);
end

vis.axs = axs;
vis.handles = handles;

vis.scale.u = 1.0;
vis.scale.d = 1.0;
vis.scale.force = 1.0;
vis.scale.n = 1.0;

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
function [X, Y, Z] = makeConeMesh(x_s, x_t, alpha)

x_s = x_s(:)';
x_t = x_t(:)';

axis_vec = x_t - x_s;
L = norm(axis_vec);

if L < 1e-12
    error('x_s and x_t are identical. Cone axis is undefined.');
end

a = axis_vec / L;

% Cone radius is largest at x_s and zero at x_t
R_base = L * tan(alpha);

n_len = 50;
n_circle = 40;

lambda = linspace(0, 1, n_len);
theta = linspace(0, 2*pi, n_circle);

% Build normal frame perpendicular to cone axis
ref = [0 0 1];
if abs(dot(a, ref)) > 0.95
    ref = [0 1 0];
end

n1 = cross(a, ref);
n1 = n1 / norm(n1);

n2 = cross(a, n1);
n2 = n2 / norm(n2);

X = zeros(n_len, n_circle);
Y = zeros(n_len, n_circle);
Z = zeros(n_len, n_circle);

for i = 1:n_len
    % From base x_s to target x_t
    center = x_s + lambda(i) * axis_vec;

    % Radius decreases linearly to zero at target
    r = (1 - lambda(i)) * R_base;

    for j = 1:n_circle
        p = center ...
            + r*cos(theta(j))*n1 ...
            + r*sin(theta(j))*n2;

        X(i,j) = p(1);
        Y(i,j) = p(2);
        Z(i,j) = p(3);
    end
end

end