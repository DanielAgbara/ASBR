"""Reproducible screw-motion figure: python -m THA1.demo."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .pa3 import get_twist, screw_to_T, draw_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/tha1/screw-motion.png'))
    args = parser.parse_args()
    twist = get_twist([0, 0, 0], [0, 0, 1], 0.3)
    initial = np.eye(4)
    initial[:3, 3] = [1, 0, 0]
    transforms = [screw_to_T(twist, t) @ initial for t in np.linspace(0, 2*np.pi, 160)]
    points = np.array([t[:3, 3] for t in transforms])
    with plt.style.context('seaborn-v0_8-whitegrid'):
        fig = plt.figure(figsize=(10, 7))
        fig.subplots_adjust(left=0.04, right=0.90, top=0.90, bottom=0.16)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(*points.T, color='#087e8b', linewidth=3, label='Body origin')
        ax.plot([0,0], [0,0], [0,2], '--', color='#334155', label='Screw axis')
        for index in [0,40,80,120,159]:
            t = transforms[index]
            draw_frame(ax, t[:3,3], t[:3,:3], L=0.25)
        ax.set(xlabel='X', ylabel='Y', zlabel='Z', title='THA1 | Rigid-body screw motion')
        ax.set_box_aspect((2,2,2))
        ax.legend(loc='upper left')
        fig.text(0.02, 0.02, 'Computed from the Python SE(3) implementation | pitch = 0.3 units/rad', fontsize=9)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=180, bbox_inches='tight', pad_inches=0.25)
        plt.close(fig)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
