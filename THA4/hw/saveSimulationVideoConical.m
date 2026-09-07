function saveSimulationVideoConical(log, x_s, x_t, alpha, view_option, filename, desired_video_time)

N = length(log.t);

video = VideoWriter(filename, 'MPEG-4');
video.FrameRate = N / desired_video_time;
open(video);

vis = initConicalImpedanceVisualizer(x_s, x_t, alpha, log.p(:,1), view_option);

fig = gcf;

for k = 1:N
    updateConicalImpedanceVisualizer( ...
        vis, ...
        log.p(:,k), ...
        log.info(k), ...
        log.F_input(:,k), ...
        log.F(:,k), ...
        log.F_actual(:,k));

    drawnow;
    writeVideo(video, getframe(fig));
end

close(video);

end