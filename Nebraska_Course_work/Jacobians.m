%%%%% Homework 6 %%%%%
%%% Derive analytic, spatial, and body Jacobians %%%

clc; clear; close all;

syms t1 t2 t3 L1 L2 L3 'real'

%% define forward kinematics map

s = @sin;
c = @cos;

x = c(t1)*(L2*c(t2) + L3*c(t2-t3));
y = s(t1)*(L2*c(t2) + L3*c(t2-t3));
z = L1 + L2*s(t2) + L3*s(t2-t3);

%% create analytical Jacobian

Analytic_J = [diff(x,t1) diff(x,t2) diff(x,t3);
      diff(y,t1) diff(y,t2) diff(y,t3);
      diff(z,t1) diff(z,t2) diff(z,t3)]

%% create spatial Jacobian

w1 = [0 0 1]'; q1 = [0 0 0]';
w2 = [0 -1 0]'; q2 = [0 0 L1]';
w3 = [0 1 0]'; q3 = [L2 0 L1]';

twist1 = [cross(-w1,q1); w1];
twist2 = [cross(-w2,q2); w2];
twist3 = [cross(-w3,q3); w3];

%exponential map for first twist
gs1 = exponential_map(twist1, t1);
gs2 = exponential_map(twist2, t2);
gs3 = exponential_map(twist3, t3);
% create adjoints
g12 = gs1;
g13 = gs1*gs2;



% Determine Ad 2 to convert twist 2 to twist 2 prime
R12 = g12(1:3, 1:3);
P12 = g12(1:3, 4);
skew_P12 = [0 -P12(3) P12(2); P12(3) 0 -P12(1); -P12(2) P12(1) 0];
Ad12 = [R12 skew_P12*R12; zeros(3,3) R12];

% Determine Ad 3 to convert twist 3 to twist 3 prime
R13 = g13(1:3, 1:3);
P13 = g13(1:3, 4);
skew_P13 = [0 -P13(3) P13(2); P13(3) 0 -P13(1); -P13(2) P13(1) 0];
Ad13 = [R13 skew_P13*R13; zeros(3,3) R13];

% Obtain twist 2 and 3 prime
twist2_P = Ad12*twist2;
twist3_P = Ad13*twist3;

Space_J = simplify([twist1 twist2_P twist3_P])
%% create body Jacobian (adjoint transformation of space)

gst_zero = [eye(3,3) [L2+L3; 0; L1]; 0 0 0 1];
% Determine Adj 1 to convert twist 1 to twist 1 dagger
g1 = gs1*gs2*gs3*gst_zero;
r1 = g1(1:3, 1:3);
p1 = g1(1:3, 4);
skew_p1 = [0 -p1(3) p1(2); p1(3) 0 -p1(1); -p1(2) p1(1) 0];
Adj1 = [r1 skew_p1*r1; zeros(3,3) r1];

% Determine Adj 2 to convert twist 2 to twist 2 dagger
g2 = gs2*gs3*gst_zero;
r2 = g2(1:3, 1:3);
p2 = g2(1:3, 4);
skew_p2 = [0 -p2(3) p2(2); p2(3) 0 -p2(1); -p2(2) p2(1) 0];
Adj2 = [r2 skew_p2*r2; zeros(3,3) r2];

% Determine Adj 3 to convert twist 3 to twist 3 dagger
g3 = gs3*gst_zero;
r3 = g3(1:3, 1:3);
p3 = g3(1:3, 4);
skew_p3 = [0 -p3(3) p3(2); p3(3) 0 -p3(1); -p3(2) p3(1) 0];
Adj3 = [r3 skew_p3*r3; zeros(3,3) r3];

twist1_D = inv(Adj1)*twist1;
twist2_D = inv(Adj2)*twist2;
twist3_D = inv(Adj3)*twist3;

Body_J = simplify([twist1_D twist2_D twist3_D])

% compare to space jacobian using adjoint transformation
g1n = gs1*gs2*gs3*gst_zero;
r1n = g1n(1:3, 1:3);
p1n = g1n(1:3, 4);
skew_p1n = [0 -p1n(3) p1n(2); p1n(3) 0 -p1n(1); -p1n(2) p1n(1) 0];
Adj1n = [r1n skew_p1n*r1n; zeros(3,3) r1n];

Check_Body = simplify(inv(Adj1n)*Space_J)
%% Derive singularities using analytical Jacobian (Problem 3)

det_AJ = real(simplify(det(Analytic_J)));

Jval = subs(Analytic_J, [L1 L2 L3], [1 1 1]);
eqn1 = real(simplify(det(Jval))) == 0;

disp('Singularities')
S_t1 = simplify(solve(eqn1, t1, 'Real', true))
S_t2 = simplify(solve(eqn1, t2, 'Real', true))
S_t3 = simplify(solve(eqn1, t3, 'Real', true))