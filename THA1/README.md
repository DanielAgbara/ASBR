# THA1 · Rotations and rigid-body motion

Convert between rotation matrices, axis–angle representations, scalar-first quaternions and Euler angles, then visualize rigid-body motion using screw theory.

![A body frame following a screw trajectory](../docs/assets/tha1-screw-motion.png)

*Reproducible example computed by `demo.py`: a full turn around the z-axis with pitch 0.3 distance units per radian. Colored arrows show the moving frame's axes.*

## What the project implements

| Source | Purpose |
| --- | --- |
| [`rotation.py`](rotation.py) | Rotation conversions, including ZYZ and ZYX Euler decompositions |
| [`pa3.py`](pa3.py) | Twist construction, SE(3) exponential map, screw recovery and frame plotting |
| [`demo.py`](demo.py) | Deterministic, noninteractive screw-motion visualization |
| [`tests/`](tests) | Fixed examples, random round trips and singular-case regression tests |

This follows the rigid transformations and programming work in the supplied THA1 submission. Much of that PDF is handwritten/scanned; the runnable feature inventory above is grounded in the source code.

For screw axis direction `s`, point `q` and pitch `h`, the implementation forms `S = [s; -s × q + h s]` and applies `T(theta) = exp([S] theta) T_initial`. The frame trajectory connects the algebra to a visible motion.

## Run

Install the root `requirements.txt`, then run from the repository root:

```sh
python -m pytest
python -m THA1.demo
python -m THA1.pa3
```

The first demo saves `outputs/tha1/screw-motion.png`. The last command prompts for a 4×4 initial transform, `q`, `s`, pitch and angle. One example is identity orientation with translation `[1,0,0]`, `q=[0,0,0]`, `s=[0,0,1]`, `h=0.3`, `theta=1.57`.

## Reuse

```python
import numpy as np
from THA1.rotation import AxisAngleToR, RToQuaternion
from THA1.pa3 import get_twist, screw_to_T

R = AxisAngleToR([0, 0, 1], np.pi / 2)
q = RToQuaternion(R)  # [w, x, y, z]
T = screw_to_T(get_twist([0, 0, 0], [0, 0, 1], 0.3), np.pi / 2)
```

Change the axis, pitch and initial frame in `demo.py` to explore another trajectory. Angles are radians and translations use a consistent user-chosen length unit.

## Numerical behavior

The quaternion conversion handles exact 180° rotations by selecting a large quaternion component; a sign-only formula is insufficient when the antisymmetric matrix entries vanish. `QuaternionToR` normalizes nonzero inputs and rejects zero or malformed quaternions. Euler angles remain nonunique at gimbal lock, where the conversion raises an error; identity has a conventional ZYZ return value.

Pure translation uses infinite pitch and a motion parameter equal to travel distance. `get_twist` maps this representation to a zero-angular-velocity twist, allowing translation recovery to round-trip through the exponential. The exponential also accounts for nonunit angular twists by scaling the angle and linear component consistently. Regression tests check translation recovery, twist scaling and motion composition.

[Back to the collection](../README.md)
