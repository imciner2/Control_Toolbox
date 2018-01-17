function [ A, B ] = sysd_BallOnPlate(  )
%SYSD_BALLONPLATE Generate the DT system matrices for a ball on a plate
%system
%
% Generate the discrete-time system A and B matrices for a ball on a
% plate system with a single actuator. This system was discretized with
% Ts=10ms.
% 
% The state vector used for the system is:
%   [ x, v ] (e.g. position and velocity along one axis)
%
% This system is described in Section 9.6 of:
%   S. Richter, “Computational complexity certification of gradient methods
%   for real-time model predictive control,” ETH Zurich, 2012.
%
%
% Usage:
%   [ A, B ] = sysd_BallOnPlate( );
%
% Outputs:
%   A - The DT state transition matrix for the system.
%   B - The DT input matrix for the system
%
%
% Created by: Ian McInerney
% Created on: January 17, 2018
% Version: 1.0
% Last Modified: January 17, 2018
%
% Revision History
%   1.0 - Initial release
    
    

%% Create the system matrices
A = [1, 0.001;
     0,     1];
B = [-0.0004;
     -0.0701];

end

