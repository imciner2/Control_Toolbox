function [ A, B ] = sysd_JonesMorari(  )
%SYSD_BALLONPLATE Generate the DT system matrices for a system described in
%a research paper
%
% Generate the state matrices for the 4-state, 2-input system described in
% the paper
%   C. N. Jones and M. Morari, “The Double Description Method for the
%   Approximation of Explicit MPC Control Laws,” in 47th IEEE Conference
%   on Decision and Control (CDC), 2008, pp. 4724–4730.
%
% Note, no explicit sampling time was provided in the paper.
% 
%
% Usage:
%   [ A, B ] = sysd_JonesMorari( );
%
% Outputs:
%   A - The DT state transition matrix for the system.
%   B - The DT input matrix for the system
%
%
% Created by: Ian McInerney
% Created on: January 18, 2018
% Version: 1.0
% Last Modified: January 18, 2018
%
% Revision History
%   1.0 - Initial release  
    

%% Create the system matrices
A = [0.7, -0.1, 0.0, 0.0;
     0.2, -0.5, 0.1, 0.0;
     0.0,  0.1, 0.1, 0.0;
     0.5,  0.0, 0.5, 0.5];
 
B = [0.0, 0.1;
     0.1, 1.0;
     0.1, 0.0;
     0.0, 0.0];

end

