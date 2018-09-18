function [Q, R, P, a] = inverseCLQR(K, A, B)
% INVERSECLQR Calculate the inverse of the LQR problem
%
% This function solves the inverse LQR problem for a continuous-time
% system. This problem is: Given a linear-time invariant control matrix K,
% and a dynamical system, what are the LQR gain matrices Q and R that would
% produce that control matrix.
%
% The solution method is based upon the paper:
%  M. Priess, et. al. (2015) , "Solutions to the Inverse LQR Problem with
%  Application to Bilogical Systems Analysis", IEEE Transactions on Control
%  System Technology, 23, 2.
%  DOI: 10.1109/TCST.2014.2343935
%
%
% Usage:
%  [Q, R, P, a] = inverseCLQR(K, A, B)
%    Inputs: 
%       K = linear time-invariant gain matrix
%       A = Plant A matrix
%       B = Plant B matrix
%    Outputs:
%       Q = State weighting matrix
%       R = Input weighting matrix
%       P = Solution to the Algebraic Riccatti Equation
%       a = Optimization cost
%
%
% Created by: Ian McInerney
% Created on: May 29, 2017
% Version: 1.0
% Last Modified: May 29, 2017


% Determine information about the system size
[numStates, numInputs ] = size(B);

I = eye(numStates + numInputs);
z = zeros(numStates, numInputs);

% Create the CVX problem
cvx_begin SDP
    variable P(numStates, numStates) symmetric;
    variable Q(numStates, numStates) symmetric;
    variable R(numInputs, numInputs) symmetric;
    variable a;
    
    minimize a^2
    subject to:
        P >= 0;
        A'*P + P*A - P*B*K + Q == 0;
        B'*P - R*K == 0;
        I <= [ Q, z;
              z', R ] <= a.*I;
cvx_end

end