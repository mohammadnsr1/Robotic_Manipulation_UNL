%%% Inverse Kinematics Confirmation %%%
%%%%%%        Problem 1          %%%%%%

clc; clear; close all;

% define lengths and angles
d1 = 110; %[mm]
L2 = 105; %[mm]
L3 = 150; %[mm]
t1 = deg2rad(90);
t2 = deg2rad(45);
t3 = deg2rad(80);

% forward kinematics
s = @sin;
c = @cos;

x = c(t1)*(L2*c(t2) + L3*c(t2-t3));
y = s(t1)*(L2*c(t2) + L3*c(t2-t3));
z = d1 + L2*s(t2) + L3*s(t2-t3);

% define inverse kinematics
r = sqrt((z-d1).^2 + x.^2 + y.^2);
C_b = (r.^2 + L2.^2 - L3.^2)/(2.*r.*L2);
S_b = sqrt(1-C_b.^2);
C_a = (L3.^2 + L2.^2 - r.^2)/(2.*L3.*L2);
S_a = sqrt(1-C_a.^2);

th1 = atan2(y,x);
th2 = atan2(S_b,C_b) + atan2 (z-d1,sqrt(x.^2 + y.^2));
th3 = pi - atan2(S_a,C_a);

theta1 = rad2deg(th1)
theta2 = rad2deg(th2)
theta3 = rad2deg(th3)



