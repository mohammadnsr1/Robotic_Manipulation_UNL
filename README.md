# Robotic_Manipulation_UNL

Code from robotic manipulation work at the University of Nebraska Lincoln: a standalone camera guided ball shooting manipulator project, plus MATLAB and Arduino course work on kinematics and manipulability.

## Ball Shooting Manipulator (Camera project)

A 6 DOF manipulator with a Pixy2 camera mounted on it plays a simple version of soccer. The ball sits inside the robot's reach and the goal is placed at a random location and angle. The arm scans for the ball and the goal with the camera, estimates the distance to each one from its pixel width using a calibrated linear regression, works out the joint angles needed to reach the ball through inverse kinematics, and then executes a kick toward the goal.

Main sketch: [Nebraska_Course_work/Camera/Camera.ino](Nebraska_Course_work/Camera/Camera.ino)

### How it works

**Detection and calibration**

The Pixy2 identifies objects by color signature: signature 3 is the red ball, signature 2 is the yellow goal marker. To find each object, the base servo sweeps from 30 to 150 degrees in small steps. At every step the code checks for blocks matching the target signature and keeps a running record of the largest pixel width seen so far, along with the angle at which it was seen. Since the camera reports the widest pixel measurement when an object is squarely in view, the angle tied to that maximum width is taken as the object's angle. The sweep runs once for the ball and once for the goal.

Distance is handled separately from angle. A `LinearRegression` function fits a slope and intercept to calibration tables of known pixel widths and known distances, one table for the ball and one for the goal, using a least squares fit. The `getDistance` function then applies `slope * pixels + intercept` to the final captured pixel width to estimate how far away each object is. A fixed constant is added to both results afterward to correct for the offset between the camera and the arm's reference point.

**Localization**

Each object now has a distance and an angle, which is a polar description of its position relative to the robot base. The code converts the ball's sensed angle into Cartesian coordinates with `x = distance * cos(angle)` and `y = distance * sin(angle)`, after first subtracting the leg length from the distance so the target point is where the leg tip should land, not the center of the ball.

Two corrections keep the end effector from missing the ball. First, the distance used in the position calculation is measured back by the length of the leg, so the arm stops behind the ball instead of on top of it. Second, if the sensed ball angle falls outside a centered range, the code nudges it by 10 degrees so the arm does not line up beside the ball instead of behind it.

**Aiming and kicking**

The base rotation angle comes from `atan2` on the computed x and y. The second joint angle comes from direct trigonometry on the ball distance, and the third joint is set equal to the second so the last link stays parallel to the ground.

To orient the leg for the kick, the code takes the difference between the goal's angle and the ball's angle, then applies the law of cosines to the ball distance, goal distance, and that angle difference to find the distance between the ball and the goal. A second application of the law of cosines, using the three known sides of that triangle, gives the leg orientation angle needed to send the ball toward the goal, with a sign check to pick the correct side.

The servo motion is sequenced so the leg cannot touch the ball early: the leg orientation joint and the kicking joint move into a cocked position first, then the base rotates into place, then the remaining arm joints move to the computed position, and only after all of that does the kicking joint swing through to strike the ball.

### Hardware

- Arduino Mega microcontroller
- Pixy2 camera for detecting the ball and the goal
- Two servos added to the third joint to control leg orientation
- 3D printed leg end effector mounted on the final joint for kicking
- 3D printed goal marked with yellow sticky notes on top for reliable camera detection

### Results and limitations

The robot consistently identified and localized the ball and the goal, positioned the leg behind the ball, and executed kicks toward the goal. Variance in the ball's final trajectory traced back to pixel based distance estimation and to noise in the angle measurement, not to the inverse kinematics itself, which reliably reproduced the computed joint angles.

### Team and course

Built by Mohammad Nasr with Thien Pham and Trevor Adelung as co authors, for Mech 453 853 Robotics, Kinematics and Design.

## Arduino: Inverse Kinematics Teleop

- [Nebraska_Course_work/Inverse_kinematics/Inverse_kinematics.ino](Nebraska_Course_work/Inverse_kinematics/Inverse_kinematics.ino): serial keyboard teleop (`q/a`, `w/s`, `e/d` to jog x/y/z, `r/f` for the gripper, `h` to home) that solves 3 DOF inverse kinematics each step and drives the servos via `Adafruit_PWMServoDriver`.

## MATLAB Course Work

- [Nebraska_Course_work/exponential_map.m](Nebraska_Course_work/exponential_map.m): shared function for the exponential map (Rodrigues' formula) of a twist `xi` and rotation `t`. Used by the Jacobian and forward kinematics scripts below.
- [Nebraska_Course_work/Jacobians.m](Nebraska_Course_work/Jacobians.m): Homework 6, derives the analytic, spatial, and body Jacobians for a 3 link arm symbolically, cross checks them against each other, and solves for singularities.
- [Nebraska_Course_work/InverseKinematics_Check.m](Nebraska_Course_work/InverseKinematics_Check.m): Problem 1, computes forward kinematics for a reference pose, then inverts it to confirm the closed form inverse kinematics recovers the same joint angles.
- [Nebraska_Course_work/wam_fk.m](Nebraska_Course_work/wam_fk.m): forward kinematics of a 7 DOF WAM style arm from a logged joint angle trajectory (`qData.txt`), using `exponential_map.m` to build the product of exponentials and plot the end effector path.
- [Nebraska_Course_work/plot_manipulability_measure_helper_function.m](Nebraska_Course_work/plot_manipulability_measure_helper_function.m): completed solution that sweeps joint angles, computes the analytic Jacobian and manipulability measure `mu3 = sqrt(det(J*J'))`, and plots it over the workspace.
- [Nebraska_Course_work/plot_manipulability_measure_helper_function_original.m](Nebraska_Course_work/plot_manipulability_measure_helper_function_original.m): the fill in the blank assignment template (Jacobian and manipulability computation left blank) that the file above completes. Kept alongside the solution intentionally, not a duplicate.

## License

Course work is under the [Apache 2.0 License](Nebraska_Course_work/LICENSE).
