# Figure provenance

| File | Origin | Reproduce |
| --- | --- | --- |
| `tha1-screw-motion.png` | New figure computed with THA1 functions | `python -m THA1.demo --output docs/assets/tha1-screw-motion.png` |
| `tha2-kuka.png` | MATLAB rendering of bundled KUKA URDF | `addpath('tools'); verify_matlab` |
| `tha3-registration.png` | Existing `THA3/HW3-PA1/code/debug_registration_figures/pa1-debug-g-_registration_frame_01.png` | Preserved original output, not newly benchmarked |
| `tha3-hand-eye.png` | Final plot from supplied hand–eye demo | `addpath('tools'); verify_matlab` |
| `tha4-tubular.png` | Default tubular fixture demo | `addpath('tools'); verify_matlab` |
| `tha4-conical.png` | Default conical fixture demo | `addpath('tools'); verify_matlab` |

The MATLAB verification helper exports the current figure from each selected demo; the demos can create other figures as well. Its robot rendering requires Robotics System Toolbox. No AI-generated imagery is used for algorithm results.

## THA2 video previews

`tha2-videos/` contains GIF previews and JPEG posters extracted from the four path recordings in `THA2/results/videos/`. Regenerate them with `python tools/build_video_previews.py` after installing `tools/requirements-media.txt`. Previews sample up to 32 frames across each recording and use a fixed display interval; they do not preserve original timing.
