function save_metrics_video2(theta_hist, T_list, Jw_cell, Jv_cell, ...
    norm_w_b_hist, norm_v_b_hist, cond_w_hist, cond_v_hist, iso_w_hist, iso_v_hist, manip_hist, filename)
ticks1 = [0, 1, 2];
ticks2 = [0, 5, 10];
ticks3 = [0, 1, 2, 3];
ticks4 = [0, 50, 100];
ticks5 = [0, 2, 4];
ticks6 = [0, 0.1, 0.2];
ticks7 = [0, 1, 2, 3];
ticks8 = [0, 1, 2];

theta_hist = rad2deg(theta_hist);

N = size(T_list, 3);
frame_step = 1;
frame_idx = unique([1:frame_step:N, N]);

%% =========================
% Separate angular / linear manipulability volumes
%% =========================
manip_w_hist = zeros(N,1);
manip_v_hist = zeros(N,1);

for k = 1:N
    J_w = Jw_cell{k};
    J_v = Jv_cell{k};

    A_w = J_w * J_w.';
    A_v = J_v * J_v.';

    manip_w_hist(k) = safe_sqrt_det(A_w);
    manip_v_hist(k) = safe_sqrt_det(A_v);
end

fprintf('%-8s : mean=% .4f, max=% .4f\n', 'Iso_w',   mean(iso_w_hist),   max(iso_w_hist));
fprintf('%-8s : mean=% .4f, max=% .4f\n', 'Cond_w',  mean(cond_w_hist),  max(cond_w_hist));
fprintf('%-8s : mean=% .4f, min=% .4f\n', 'Manip_w', mean(manip_w_hist), min(manip_w_hist));

fprintf('%-8s : mean=% .4f, max=% .4f\n', 'Iso_v',   mean(iso_v_hist),   max(iso_v_hist));
fprintf('%-8s : mean=% .4f, max=% .4f\n', 'Cond_v',  mean(cond_v_hist),  max(cond_v_hist));
fprintf('%-8s : mean=% .4f, min=% .4f\n', 'Manip_v', mean(manip_v_hist), min(manip_v_hist));


fig = figure('Color','w', 'Position', [100 100 1200 800], 'Renderer', 'opengl');

v = VideoWriter(filename, 'MPEG-4');
v.FrameRate = 20;
open(v);


%% =========================
% y-axis ranges
%% =========================
manip_w_ymax = get_2sigma_ymax(manip_w_hist);
manip_v_ymax = get_2sigma_ymax(manip_v_hist);
cond_w_ymax  = get_2sigma_ymax(cond_w_hist);
cond_v_ymax  = get_2sigma_ymax(cond_v_hist);
iso_w_ymax  = get_2sigma_ymax(iso_w_hist);
iso_v_ymax  = get_2sigma_ymax(iso_v_hist);

fs = 20;
eps_val = 1e-12;

for k = frame_idx
    clf(fig);
    t = tiledlayout(2,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');
    % %% =========================
    % % (1,1) Angular Isotropy
    % %% =========================
    % % ax1 = subplot(4,2,1);
    % ax1 = nexttile;
    % hold(ax1,'on'); grid(ax1,'on');
    % 
    % y5 = cap_inf(iso_w_hist, iso_w_ymax);
    % plot(ax1, 0:N-1, y5, 'r-', 'LineWidth', 1.5);
    % xline(ax1, k-1, 'k--', 'LineWidth', 1.0);
    % plot(ax1, k-1, min(cap_inf(iso_w_hist(k), iso_w_ymax), iso_w_ymax), ...
    %     'ro', 'MarkerFaceColor', 'r');
    % 
    % ylabel(ax1, '\mu_{1, w}', 'FontSize', fs);
    % xlim(ax1, [0 N-1]);
    % 
    % ylim(ax1, [ticks1(1), ticks1(end)]);
    % yticks(ax1, ticks1);
    % set(ax1, 'FontSize', fs);
    % 
    % 
    % %% =========================
    % % (1,2) Linear Isotropy
    % %% =========================
    % % ax2 = subplot(4,2,2);
    % ax2 = nexttile;
    % hold(ax2,'on'); grid(ax2,'on');
    % 
    % y6 = cap_inf(iso_v_hist, iso_v_ymax);
    % plot(ax2, 0:N-1, y6, 'b-', 'LineWidth', 1.5);
    % xline(ax2, k-1, 'k--', 'LineWidth', 1.0);
    % plot(ax2, k-1, min(cap_inf(iso_v_hist(k), iso_v_ymax), iso_v_ymax), ...
    %     'bo', 'MarkerFaceColor', 'b');
    % 
    % ylabel(ax2, '\mu_{1, w}', 'FontSize', fs);
    % 
    % xlim(ax2, [0 N-1]);
    % set(ax2, 'FontSize', fs);
    % 
    % ylim(ax2, [ticks2(1), ticks2(end)]);
    % yticks(ax2, ticks2);
    % 
    % %% =========================
    % % (2,1) Angular Condition
    % %% =========================
    % % ax3 = subplot(4,2,3);
    % ax3 = nexttile;
    % hold(ax3,'on'); grid(ax3,'on');
    % 
    % y3 = cap_inf(cond_w_hist, cond_w_ymax);
    % plot(ax3, 0:N-1, y3, 'r-', 'LineWidth', 1.5);
    % xline(ax3, k-1, 'k--');
    % plot(ax3, k-1, min(y3(k), cond_w_ymax), 'ro', 'MarkerFaceColor', 'r');
    % 
    % ylabel(ax3, '\mu_{2,w}');
    % 
    % yticks(ax3, ticks3);
    % ylim(ax3, [ticks3(1), ticks3(end)]);
    % xlim(ax3, [0 N-1]);
    % set(ax3, 'FontSize', fs);
    % 
    % %% =========================
    % % (2,2) Linear Condition
    % %% =========================
    % % ax4 = subplot(4,2,4);
    % ax4 = nexttile;
    % hold(ax4,'on'); grid(ax4,'on');
    % 
    % y4 = cap_inf(cond_v_hist, cond_v_ymax);
    % plot(ax4, 0:N-1, y4, 'b-', 'LineWidth', 1.5);
    % xline(ax4, k-1, 'k--');
    % plot(ax4, k-1, min(y4(k), cond_v_ymax), 'bo', 'MarkerFaceColor', 'b');
    % 
    % ylabel(ax4, '\mu_{2,v}');
    % 
    % yticks(ax4, ticks4);
    % ylim(ax4, [ticks4(1), ticks4(end)]);
    % xlim(ax4, [0 N-1]);
    % set(ax4, 'FontSize', fs);
    % 
    % %% =========================
    % % (3,1) Angular Manipulability
    % %% =========================
    % % ax5 = subplot(4,2,5);
    % ax5 = nexttile;
    % hold(ax5,'on'); grid(ax5,'on');
    % 
    % y1 = cap_inf(manip_w_hist, manip_w_ymax);
    % plot(ax5, 0:N-1, y1, 'r-', 'LineWidth', 1.5);
    % xline(ax5, k-1, 'k--');
    % plot(ax5, k-1, min(y1(k), manip_w_ymax), 'ro', 'MarkerFaceColor', 'r');
    % 
    % ylabel(ax5, '\mu_{3,w}');
    % 
    % yticks(ax5, ticks5);
    % ylim(ax5, [ticks5(1), ticks5(end)]);
    % xlim(ax5, [0 N-1]);
    % set(ax5, 'FontSize', fs);
    % 
    % %% =========================
    % % (3,2) Linear Manipulability
    % %% =========================
    % % ax6 = subplot(4,2,6);
    % ax6 = nexttile;
    % hold(ax6,'on'); grid(ax6,'on');
    % 
    % y2 = cap_inf(manip_v_hist, manip_v_ymax);
    % plot(ax6, 0:N-1, y2, 'b-', 'LineWidth', 1.5);
    % xline(ax6, k-1, 'k--');
    % plot(ax6, k-1, min(y2(k), manip_v_ymax), 'bo', 'MarkerFaceColor', 'b');
    % 
    % ylabel(ax6, '\mu_{3,v}');
    % 
    % yticks(ax6, ticks6);
    % ylim(ax6, [ticks6(1), ticks6(end)]);
    % xlim(ax6, [0 N-1]);
    % set(ax6, 'FontSize', fs);

    %% =========================
    % (4,1) Angular Error Norm (NO cap_inf)
    %% =========================
    % ax7 = subplot(4,2,7);
    ax7 = nexttile;
    hold(ax7,'on'); grid(ax7,'on');

    y7 = max(norm_w_b_hist, eps_val); % log 안정화

    plot(ax7, 0:N-1, y7, 'r-', 'LineWidth', 1.5);
    xline(ax7, k-1, 'k--');
    plot(ax7, k-1, y7(k), 'ro', 'MarkerFaceColor', 'r');

    ylabel(ax7, '||w_b|| [rad]');
    % set(ax7, 'YScale', 'log');

    xlim(ax7, [0 N-1]);    

    ylim(ax7, [ticks7(1), ticks7(end)]);
    yticks(ax7, ticks7);
    set(ax7, 'FontSize', fs);

    %% =========================
    % (4,2) Linear Error Norm (NO cap_inf)
    %% =========================
    % ax8 = subplot(4,2,8);
    ax8 = nexttile;
    hold(ax8,'on'); grid(ax8,'on');

    y8 = max(norm_v_b_hist, eps_val);

    plot(ax8, 0:N-1, y8, 'b-', 'LineWidth', 1.5);
    xline(ax8, k-1, 'k--');
    plot(ax8, k-1, y8(k), 'bo', 'MarkerFaceColor', 'b');

    ylabel(ax8, '||v_b|| [m]');
    % set(ax8, 'YScale', 'log');

    xlim(ax8, [0 N-1]);
    yticks(ax8, [1e-8, 1e-5, 1e0]);

    ylim(ax8, [ticks8(1), ticks8(end)]);
    yticks(ax8, ticks8);
    set(ax8, 'FontSize', fs);

    %% =========================
    % (5,1) Angle (NO cap_inf)
    %% =========================
    ax9 = nexttile;
    hold(ax9,'on'); grid(ax9,'on');
    
    iters = 0:N-1;
    
    % --- Group 1: joints 1,3,5,7 ---
    idx1 = [1 3 5 7];
    for i = idx1
        plot(ax9, iters, theta_hist(:,i), 'LineWidth', 1.2);
        plot(ax9, k-1, theta_hist(k,i), 'o', 'MarkerFaceColor', 'auto');
    end
    
    xline(ax9, k-1, 'k--');
    
    xlabel(ax9, 'Iteration');
    ylabel(ax9, '[degree]');
    
    xlim(ax9, [0 N-1]);
    
    ylim(ax9, [-180, 180]); yticks(ax9, [-180, 0, 180]);
    % legend(ax9, '\theta_1','\theta_3','\theta_5','\theta_7', 'Location','southeast');
    
    set(ax9, 'FontSize', fs);

    %% =========================
    % (5,2) Angle (NO cap_inf)
    %% =========================
    ax10 = nexttile;
    hold(ax10,'on'); grid(ax10,'on');
    
    % --- Group 2: joints 2,4,6 ---
    idx2 = [2 4 6];
    for i = idx2
        plot(ax10, iters, theta_hist(:,i), 'LineWidth', 1.2);
        plot(ax10, k-1, theta_hist(k,i), 'o', 'MarkerFaceColor', 'auto');
    end
    
    xline(ax10, k-1, 'k--');
    
    xlabel(ax10, 'Iteration');
    ylabel(ax10, '[degree]');
    
    xlim(ax10, [0 N-1]);
    ylim(ax10, [-180, 180]); yticks(ax10, [-180, 0, 180]);

    % legend(ax10, '\theta_2','\theta_4','\theta_6', 'Location','best');
   
    set(ax10, 'FontSize', fs);

    %% =========================
    % Title
    %% =========================
    sgtitle(sprintf('IK Metrics Evolution (iter %d/%d)', k-1, N-1), ...
        'FontSize', fs+2, 'FontWeight', 'bold');

    % set(ax1, 'XTickLabel', []);
    % set(ax2, 'XTickLabel', []);
    % set(ax3, 'XTickLabel', []);
    % set(ax4, 'XTickLabel', []);
    % set(ax5, 'XTickLabel', []);
    % set(ax6, 'XTickLabel', []);
    set(ax7, 'XTickLabel', []);
    set(ax8, 'XTickLabel', []);
    drawnow;
    frame = getframe(fig);
    writeVideo(v, frame);
end

close(v);
% close(fig);
end