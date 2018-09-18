function [ Qd, Rd, Sd ] = c2d_weights( sys, Qc, Rc, Ts )
%C2D_WEIGHTS Discretize the weights for the optimal control problem
%
% This function will compute the optimal discrete-time weights that make
% the discrete-time LQR cost function equivalent to the continuous-time LQR
% cost function.
%
% The algorithm used in this function is based on the matrix-exponential
% method presented in Section 9.3.4 of
%   G. F. Franklin, J. D. Powell, and M. L. Workman, Digital Control of
%   Dynamic Systems, Third. Menlo Park, CA, USA: Addison-Wesley, 1998.
%
%
% Usage:
%   [ Qd, Rd, Sd ] = C2D_WEIGHTS( sys, Qc, Rc, Ts);
%
% Inputs:
%   sys - The continuous-time system
%   Qc  - The continuous-time state weighting matrix
%   Rc  - The continuous-time input weighting matrix
%   Ts  - The sampling time
%
% Output:
%   Qd - The discrete-time state weighting matrix
%   Rd - The discrete-time input weighting matrix
%   Sd - The discrete-time cross-term weights
%
%
% Created by: Ian McInerney
% Created on: September 18, 2018
% Version: 1.0
% Last Modified: September 18, 2018
%
% Revision History
%   1.0 - Initial release


%% Check to see if a discrete-time system was supplied
if ( sys.Ts ~= 0 )
        error('Must supply the system in continuous-time.');
end


%% Extract the system matrices
A = sys.A;
B = sys.B;
[n, m] = size(B);

%% Compute the matrix exponential
zn  = zeros(n);
zm  = zeros(m);
znm = zeros(n, m);
zmn = znm';

mmat = [-A', znm,  Qc, znm;
        -B',  zm, zmn,  Rc;
         zn, znm,   A,   B;
        zmn,  zm, zmn,  zm];
        
ex = expm(mmat.*Ts);


%% Extract the weights
phi11 = ex(    1:(n+m),     1:(n+m));
phi12 = ex(    1:(n+m), (n+m+1):end);
phi22 = ex((n+m+1):end, (n+m+1):end);

phi = phi22'*phi12;

Qd = phi(      1:n,       1:n);
Sd = phi(      1:n, (n+1):end);
Rd = phi((n+1):end, (n+1):end);


end

