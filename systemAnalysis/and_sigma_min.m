function [ sigma_min ] = and_sigma_min( sys )
%AND_SIGMA_MIN Find the smallest singular value of a DT system
%
% Find the smallest singular value of a given discrete-time system.
%
%
% Usage:
%   sigma_min = and_sigma_min( sys );
%
% Inputs:
%   sys - The system object
%
% Outputs:
%   sigma_min - The smallest singular value
%
%
% Created by: Ian McInerney
% Created on: January 19, 2018
% Version: 1.0
% Last Modified: January 19, 2018
%
% Revision History
%   1.0 - Initial release

%% Greedly search the space of frequencies
n = 10000;
omega = linspace(0, 2*pi/(sys.Ts), n);

sv = sigma(sys, omega);


%% Pull out the smallest singular value
sigma_min = min(min(sv));


end

