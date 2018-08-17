function [ A, B ] = sysc_InvPend_01(  )
%SYSC_INVEPEND_01 Continuous-time inverted pendulum system (linear)
%
% This provides the state transition matrix and input matrix for a
% continuous-time inverted pendulum on a cart. The state mapping is as
% follows:
%   [cart position;
%    cart velocity;
%    pendulum angle;
%    pendulum angular velocity];
%
% This model comes from
%   D. Buchstaller, E. C. Kerrigan, and G. A. Constantinides, “Sampling
%   and controlling faster than the computational delay,” IET Control
%   Theory and Applications, vol. 6, no. 8, pp. 1071–1079, May 2012.
%
%
% Usage:
%   [ A, B ] = SYSC_INVEPEND_01( );
%
% Outputs:
%   A - The CT state transition matrix for the system.
%   B - The CT input matrix for the system
%
%
% Created by: Ian McInerney
% Created on: July 19, 2018
% Version: 1.0
% Last Modified: July 19, 2018
%
% Revision History
%   1.0 - Initial release

% The state transition matrix
A = [ 0,      1,     0, 0;
      0, -0.196, 0.016, 0;
      0,      0,     0, 1;
      0, -0.054,   2.7, 0];

% The input mapping matrix
B = [0;
     0.016;
     0;
     0.54];
    


end

