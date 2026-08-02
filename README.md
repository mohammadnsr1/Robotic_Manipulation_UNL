# Robotic_Manipulation_UNL

Code from robotic manipulation work at the University of Nebraska-Lincoln: a standalone camera-guided ball-shooting project, plus MATLAB/Arduino course work on kinematics and manipulability.

## Ball-Shooting Manipulator (Camera project)

A custom 6-DOF manipulator with a single Pixy2 camera mounted on it. The arm scans for a red ball, estimates its distance/angle from the camera via a calibrated linear regression (pixel width → distance), then scans for the yellow goal the same way, computes the required joint angles with inverse kinematics, and shoots the ball toward the goal.

- [Nebraska_Course_work/Camera/Camera.ino](Nebraska_Course_work/Camera/Camera.ino) — main sketch: Pixy2 object detection (signature 3 = ball, signature 2 = gate/goal), pixel-to-distance calibration via `LinearRegression`/`getDistance`, inverse kinematics to aim, and PWM servo control to shoot.

## Arduino: Inverse Kinematics Teleop

- [Nebraska_Course_work/Inverse_kinematics/Inverse_kinematics.ino](Nebraska_Course_work/Inverse_kinematics/Inverse_kinematics.ino) — serial keyboard teleop (`q/a`, `w/s`, `e/d` to jog x/y/z, `r/f` for the gripper, `h` to home) that solves 3-DOF inverse kinematics each step and drives the servos via `Adafruit_PWMServoDriver`.

## MATLAB Course Work

- [Nebraska_Course_work/exponential_map.m](Nebraska_Course_work/exponential_map.m) — shared function: exponential map (Rodrigues' formula) for a twist `xi` and rotation `t`. Used by the Jacobian and forward-kinematics scripts below.
- [Nebraska_Course_work/Jacobians.m](Nebraska_Course_work/Jacobians.m) — Homework 6: derives the analytic, spatial, and body Jacobians for a 3-link arm symbolically, cross-checks them against each other, and solves for singularities.
- [Nebraska_Course_work/InverseKinematics_Check.m](Nebraska_Course_work/InverseKinematics_Check.m) — Problem 1: forward-kinematics a reference pose, then inverts it to confirm the closed-form inverse kinematics recover the same joint angles.
- [Nebraska_Course_work/wam_fk.m](Nebraska_Course_work/wam_fk.m) — forward kinematics of a 7-DOF WAM-style arm from a logged joint-angle trajectory (`qData.txt`), using `exponential_map.m` to build the product of exponentials and plot the end-effector path.
- [Nebraska_Course_work/plot_manipulability_measure_helper_function.m](Nebraska_Course_work/plot_manipulability_measure_helper_function.m) — completed solution: sweeps joint angles, computes the analytic Jacobian and manipulability measure `mu3 = sqrt(det(J*J'))`, and plots it over the workspace.
- [Nebraska_Course_work/plot_manipulability_measure_helper_function_original.m](Nebraska_Course_work/plot_manipulability_measure_helper_function_original.m) — the fill-in-the-blank assignment template (Jacobian and manipulability computation left blank) that the file above completes. Kept alongside the solution intentionally, not a duplicate.

## License

Course work is under the [Apache 2.0 License](Nebraska_Course_work/LICENSE).
