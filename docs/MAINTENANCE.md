# Maintenance and validation

- Added root and per-project READMEs, source maps, setup instructions and visual provenance.
- Added a MATLAB launcher with isolated per-project search paths, restored environment state, explicit toolbox checks and output directories.
- Made URDF/data paths independent of the working directory. Unknown-dataset results now save in the output directory instead of overwriting supplied reference answers.
- Replaced opaque `aaa_*` experiment names with descriptive `demo_*` names.
- Made expensive THA2/conical video export opt-in through `export_videos = false` in the demo scripts.
- Fixed Python rotation-to-quaternion conversion at exact half-turns, normalized quaternion inputs and clipped the inverse-cosine argument before evaluation.
- Added a module-compatible Python import, deterministic screw-motion demo, pure-translation recovery and consistent scaling of nonunit twists.
- Removed duplicate `lsqlin` calls from the two constrained IK solvers and noisy Jacobian/reflection debugging prints.
- Added dependency declarations, focused test discovery, MIT licensing and third-party notices.
- Excluded local prototypes and generated clutter from publication while preserving them locally.

## Validation boundaries

Python: **61 tests passed** under Python 3.13. Run `python -m pytest` to exercise rotation round trips, exact-half-turn regressions, translation recovery and screw-motion composition. The identity-axis warning is expected.

MATLAB R2025b: the known-transform registration check, portable data loading, debug/unknown registration demos, hand–eye demo, tubular/conical demos, KUKA model import and default pose IK were exercised. Default pose IK stored 17 iterates and reported final translational twist error about `8.67e-7`. This is one test case, not a global convergence guarantee.

Optimization Toolbox is absent locally, so constrained IK runtime checks and distance sweeps remain unverified. Video exporters and the other THA2 solver demos require separate validation; no cross-platform execution claim is made.

## Next improvements to the library interfaces

The algorithms are still course implementations. The entry points and documentation are now clearer, but a complete library redesign would additionally:

1. Convert experiment scripts into functions accepting configuration structs and returning explicit results, eliminating workspace clearing and launcher script evaluation.
2. Consolidate repeated kinematics functions only after comparing conventions and regression-testing both projects. Same names do not establish equivalence.
3. Return explicit convergence status, final constraint residuals and infeasibility diagnostics from every iterative solver.
4. Add property tests for Jacobians, synthetic pivot/hand–eye ground truth, degenerate inputs and constrained-step feasibility.
5. Validate all public vector/matrix shapes and nonfinite values consistently.

The launcher currently executes each experiment in a private function workspace. Direct reuse should call the documented algorithm functions with their required inputs. Retained variants such as `redundancy_resolution2.m` should not be deleted solely because of their names: establish which behavior each contains first.
