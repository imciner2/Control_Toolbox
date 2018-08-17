function [ A, B, C, Ts ] = sysd_linear_CSTR2(  )
%SYSD_LINEAR_CSTR2 Generate the DT system matrices for a linearization of a
%CSTR system
%
% Generate the linearized state matrices for the 3-state, 2-input CSTR
% system described in:
%   T. A. N. Heirung, J. A. Paulson, J. O’Leary, and A. Mesbah,
%   “Stochastic model predictive control — how does it work?,” Computers
%   and Chemical Engineering, vol. 114, no. June, pp. 158–170, 2017.
%
% The sampling time is 0.002.
% 
%
% Usage:
%   [ A, B, C, Ts ] = SYSD_LINEAR_CSTR2( );
%
% Outputs:
%   A  - The DT state transition matrix for the system.
%   B  - The DT input matrix for the system
%   C  - The output mapping matrix
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

Ts = 0.002;

A = [ 0.95123, 0;
     0.08833, 0.81873];

B = [-0.0048771;
    -0.0020429];

C = eye(2);

end