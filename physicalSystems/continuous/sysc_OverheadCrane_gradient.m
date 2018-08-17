function [ x_dot ] = sysc_OverheadCrane_gradient( x, u )
%SYSC_OVERHEADCRANE_GRADIENT Compute the gradients for the differential
%equation of an overhead gantry crane
%
% This function computes the continuous-time gradients of the states for a
% nonlinear gantry crane which can move along one axis, and has a variable
% length string.
%
% This model comes from:
%   B. Khusainov, E. Kerrigan, A. Suardi, and G. Constantinides,
%   “Nonlinear predictive control on a heterogeneous computing platform,”
%   Control Engineering Practice, vol. 78, no. September, pp. 105–115, 2018.
%
%
% The state-space of the model is:
%   [Cart lateral position
%    Cart lateral velocity
%    Length of the string
%    Velocity of the length of the string
%    Angle of the string and the cart
%    Angular velocity of the string and the cart]
% The control inputs of the model are:
%   [ String velocity control
%     Cart velocity control]
%
%
% Usage:
%   x_dot = SYSC_OVERHEADCRANE_GRADIENT( x, u );
%
% Inputs:
%   x - The current state
%   u - The current control input
%
% Outputs:
%   x_dot - The state derivative vector
%
%
% Created by: Ian McInerney
% Created on: July 19, 2018
% Version: 1.0
% Last Modified: July 19, 2018
%
% Revision History
%   1.0 - Initial release


% Some constants
Tc = 0.13;
Tl = 0.07;
g = 9.81;

% Compute the derivatives
x_dot(1) = x(2);
x_dot(2) = (-x(2) + u(2) ) / Tc;
x_dot(3) = x(4);
x_dot(4) = (-x(4) + u(1) ) / Tl;
x_dot(5) = x(5);
x_dot(6) = ( (-x(2) + u(2))/Tc*cos(x(5)) + g*sin(x(5)) + 2*x(4)*x(5) ) / x(3);


end

