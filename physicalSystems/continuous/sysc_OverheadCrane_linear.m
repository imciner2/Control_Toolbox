function [ A, B ] = sysc_OverheadCrane_linear( x_eq, u_eq )
%SYSC_OVERHEADCRANE_LINEAR Compute the linearization of an overhead gantry crane
%
% This function computes the linear state space for a nonlinear gantry
% crane which can move along one axis, and has a variable length string.
% The linearization is performed around the equilibrium point supplied by
% the user.
%
% This model comes from:
%   B. Khusainov, E. Kerrigan, A. Suardi, and G. Constantinides,
%   “Nonlinear predictive control on a heterogeneous computing platform,”
%   Control Engineering Practice, vol. 78, no. September, pp. 105–115, 2018.
%
%
% The state-space of the model is:
%   [Cart lateral position (X_c)
%    Cart lateral velocity (V_c)
%    Length of the string (X_l)
%    Velocity of the length of the string (V_l)
%    Angle of the string and the cart (theta)
%    Angular velocity of the string and the cart (theta_dot) ]
% The control inputs of the model are:
%   [ String velocity control (u_l)
%     Cart velocity control (u_c) ]
%
%
% Usage:
%   x_dot = SYSC_OVERHEADCRANE_LINEAR( x, u );
%
% Inputs:
%   x_eq - The equilibrium state
%   u_eq - The equilibrium control input
%
% Outputs:
%   A - The linear state transition matrix
%   B - The linear input mapping matrix
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


% Extract the equilibrium conditions
x_c = x_eq(1);
v_c = x_eq(2);
x_l = x_eq(3);
v_l = x_eq(4);
theta = x_eq(5);
omega = x_eq(6);
u_l = u_eq(1);
u_c = u_eq(2);

% Compute the state transition matrix
A = [ 0,                    1,                                                                 0,             0,                                                0, 0;
      0,                -1/Tc,                                                                 0,             0,                                                0, 0;
      0,                    0,                                                                 0,             1,                                                0, 0;
      0,                    0,                                                                 0,         -1/Tl,                                                0, 0;
      0,                    0,                                                                 0,             0,                                                0, 0;
      0, -cos(theta)/(Tc*x_l), -(2*omega*v_l + g*sin(theta) + (cos(theta)*(u_c - v_c))/Tc)/x_l^2, (2*omega)/x_l, (g*cos(theta) - (sin(theta)*(u_c - v_c))/Tc)/x_l, 0];

  
% Compute the input mapping matrix
B = [    0,                   0;
         0,                1/Tc;
         0,                   0;
      1/Tl,                   0;
         0,                   0;
         0, cos(theta)/(Tc*x_l)];

end

