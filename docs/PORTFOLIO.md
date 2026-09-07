# Portfolio case study

**Title:** Algorithms for Sensor-Based Robotics

**Short description:** A four-project series connecting rigid-body geometry, redundant manipulator kinematics, sensor calibration and constrained motion. Developed with Min-Geun Park using Python and MATLAB for ME 384R at UT Austin.

**Technical story:** We built rotation and screw-motion routines, modeled a seven-joint KUKA arm with the Product of Exponentials, implemented point/pivot and hand–eye calibration, and explored impedance-based virtual fixtures and constrained inverse kinematics. Simulations make the coordinate transformations and controller behavior visible.

**Evidence to show:** Reproducible THA1 screw motion, THA2 robot model and original trajectory video, THA3 calibration output, and THA4 tubular/conical simulation snapshots.

**Reflection:** Numerical conventions and initialization matter. Quaternion half-turns need stable handling, calibration depends on pose diversity, and locally linearized constraints can permit wall penetration during large moves. These observations connect the mathematical derivations to practical implementation limits.

**Links after publication:** `https://github.com/DanielAgbara/ASBR` and its `THA1`–`THA4` folders. Do not state individual ownership of specific algorithms without confirming the original division of work with the project authors.
