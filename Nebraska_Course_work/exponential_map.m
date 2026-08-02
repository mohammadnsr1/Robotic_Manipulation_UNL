% This function computes the exponential map for a revolute joint
% given the twist vector (xi) and amount of rotation (t). 
% INPUTS:
%     xi: is a 6x1 twist vector
%     t:  theta, is the amount of rotation in radians;
% OUTPUT:
%     gi: exponential map twist xi and rotation t.

function [gi] = exponential_map(xi, t)
 w = [xi(4);xi(5);xi(6)];
 w_hat = [0 -xi(6) xi(5);xi(6) 0 -xi(4);-xi(5) xi(4) 0];
 v = [xi(1);xi(2);xi(3)];
 R = eye(3) + w_hat*sin(t) + (w_hat^2)*(1 - cos(t));
 gi = [R (eye(3) - R)*(cross(w,v)) + w*w'*v*t ; 0 0 0 1]; 
end