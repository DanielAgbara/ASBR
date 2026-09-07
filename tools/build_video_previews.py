"""Build sampled GIF previews and still posters from the THA2 recordings.

Run from the repository root after installing tools/requirements-media.txt.
The MP4 files remain the authoritative, full-speed recordings.
"""
from pathlib import Path
import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
VIDEOS = [
    ('Circle_Path.mp4', 'circle-path', 'Circle path'),
    ('Helix_Path.mp4', 'helix-path', 'Helix path'),
    ('Square_Path.mp4', 'square-path', 'Square path'),
    ('Raster_Path.mp4', 'raster-path', 'Raster path'),
    ('ik_robot_motion.mp4', 'ik-robot-motion', 'Inverse kinematics: robot motion'),
    ('ik_metrics.mp4', 'ik-metrics', 'Inverse kinematics: metrics'),
    ('redun_robot_motion.mp4', 'redundancy-robot-motion', 'Redundancy resolution: robot motion'),
    ('redun_metrics.mp4', 'redundancy-metrics', 'Redundancy resolution: metrics'),
]


def main():
    output = ROOT / 'docs/assets/tha2-videos'
    output.mkdir(parents=True, exist_ok=True)
    for filename, slug, title in VIDEOS:
        source = ROOT / 'THA2/results/videos' / filename
        capture = cv2.VideoCapture(str(source))
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not capture.isOpened() or count <= 0 or fps <= 0:
            raise ValueError(f'Cannot decode {source}')
        frames = []
        for index in np.linspace(0, count - 1, min(32, count), dtype=int):
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
            ok, frame = capture.read()
            if not ok:
                raise ValueError(f'Cannot decode frame {index} of {source}')
            frame = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            frame.thumbnail((560, 340), Image.Resampling.LANCZOS)
            frames.append(frame)
        capture.release()
        frames[len(frames)//2].save(output / f'{slug}.jpg', quality=90)
        frames[0].save(output / f'{slug}.gif', save_all=True,
                       append_images=frames[1:], duration=160, loop=0, optimize=True)
        print(f'{title}: {count} frames, {count/fps:.1f}s; preview generated')


if __name__ == '__main__':
    main()
