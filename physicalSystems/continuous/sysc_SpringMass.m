function [ A, B ] = sysc_SpringMass( n, varargin )
%SYSC_SPRINGMASS Generate the CT system matrices for a Spring-Mass system
%
% Generate the continuous-time system A and B matrices for a spring-mass
% system that takes the form of:
%      k1 m1 k2 m2 k3 m3 k4  kn mn k(n+1)
%      ~~~*~~~~~*~~~~~*~~~~...~~*~~~~~
% 
% The differential equation that models the ith mass is:
%       m_i*a_i = k_(i-1)*x_(i-1) + k_(i+1)*x_(i+1) - (k_(i-1) + k_(i+1))*x_i
%
% The state vector used for the system is:
%   [ v_1, v_2, ..., v_n, x_1, x_2, ..., x_n ]
%
% This system places two inputs, the first as a force input on m1, the
% second as a velocity input on mn.
%
%
% Usage:
%   [ A, B ] = sysc_SpringMass( n );
%   [ A, B ] = sysc_SpringMass( n, k );
%   [ A, B ] = sysc_SpringMass( n, k, m );
%
% Inputs:
%   n - The number of masses
%   k - A vector of n+1 spring constants. If all springs are the same, a
%       single value can be provided. The default is k=1.
%   m - A vector of n masses. If all masses are the same, a single value
%       can be provided. The default is m=1.
%
% Outputs:
%   A - The CT state transition matrix for the system.
%   B - The CT input matrix for the system
%
%
% Created by: Ian McInerney
% Created on: January 17, 2018
% Version: 1.0
% Last Modified: January 17, 2018
%
% Revision History
%   1.0 - Initial release



%% Parse the input arguments
k = 1;
m = 1;
if (nargin == 2)
    k = varargin{1};
elseif (nargin == 3)
    k = varargin{1};
    m = varargin{2};
end

if ( isempty(k) )
    k = 1;
end


%% Create the spring constants and masses (if required)
if ( length(k) == 1 )
    k = kron(k, ones(1,n+1) );
end
if ( length(m) == 1 )
    m = kron(m, ones(1,n) );
end
    
    

%% Create the system matrices
Z = zeros(n);
I = eye(n);

% Create the state matrix
s = k(1:end-1) + k(2:end);  % The main diagonal matrix elements
o1 = k(1:end-2)./m(1:end-1);         % The elements below the diagonal
o2 = k(3:end)./m(2:end);           % The elements above the diagonal

T = diag(-s./m) + diag(o1, -1) + diag(o2, 1);

A = [Z, T;
     I, Z];

% Create the input matrix 
B = zeros(2*n, 2);
B(1,1) = 1/m(1);
B(end,end) = 1/m(end);

end

