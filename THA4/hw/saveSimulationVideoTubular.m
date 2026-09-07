function saveSimulationVideoTubular(log, curve_vis, R, view_option, filename, desired_video_time)

N = length(log.t);

video = VideoWriter(filename, 'MPEG-4');
video.FrameRate = N / desired_video_time;
open(video);

vis = initTubularImpedanceVisualizer(curve_vis, log.p(:,1), R, view_option);

fig = gcf;

for k = 1:N
    updateTubularImpedanceVisualizer( ...
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