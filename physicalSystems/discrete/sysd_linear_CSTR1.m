function [ A, B, C, Bd, Ts ] = sysd_linear_CSTR1(  )
%SYSD_LINEAR_CSTR1 Generate the DT system matrices for a linearization of a
%CSTR system
%
% Generate the linearized state matrices for the 3-state, 2-input CSTR
% system described in:
%   G. Pannocchia and J. B. Rawlings, “Disturbance models for offset-free
%   model-predictive control,” AIChE Journal, vol. 49, no. 2,
%   pp. 426–437, 2003.
%
% The sampling time is 1 minute (60 seconds).
% 
%
% Usage:
%   [ A, B, C, Bd, Ts ] = SYSD_LINEAR_CSTR1( );
%
% Outputs:
%   A  - The DT state transition matrix for the system.
%   B  - The DT input matrix for the system
%   C  - The output mapping matrix
%   Bd - The disturbance input mapping matrix
%   Ts - The sampling time (in seconds)
%
%
% Created by: Ian McInerney
% Created on: July 23, 2018
% Version: 1.0
% Last Modified: July 23, 2018
%
% Revision History
%   1.0 - Initial release

Ts = 60;

A = [0.2511, -3.368e-3, -7.056e-4;
      11.06,    0.3296,    -2.545;
        0.0,       0.0,         1];

B = [ -5.426e-3,  1.530e-5;
          1.297,    0.1218;
            0.0, -6.592e-2];

Bd = [-1.762e-5;
       7.784e-2;
       6.592e-2];

C = eye(3);

end