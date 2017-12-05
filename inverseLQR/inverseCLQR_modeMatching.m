function [Q, R, S, P, Kopt] = inverseCLQR_modeMatching(K, A, B, R, Qform, a)
%INVERSECLQR_MODEMATCHING Solve the inverse LQR problem by matching system modes
%
% This function solves the inverse LQR problem (for continuous time) by
% matching system modes between the closed loop system with the input
% controller and the modes of the system with the LQR controller created
% using the estimated Q, S, and R matrices.
%
% Usage:
%  [Q, R, S, P, Kopt] = inverseCLQR(K, A, B, R, Qform, a)
%    Inputs:
%      K = The constant-gain matrix to 
%      A = The system's state transition matrix
%      B = The system's input matrix
%      R = The R matrix to use in the inverse problem
%      Qform = A matrix containing 1s in every location Q can have a value
%      a = Regularization parameter (must be >0)
%    Outputs:
%      Q = State wieghting matrix
%      R = Input weighting matrix (same as supplied)
%      S = State-Input corss term weighting matrix
%      P = Solution to the Algebraic Riccatti Equation
%      Kopt = Optimal controller
%
%
% Created by: Ian McInerney
% Created on: June 13, 2017
% Version: 1.0
% Last Modified: June 14, 2017


%% Determine information about the system size
[numStates, numInputs ] = size(B);

I = eye(numStates + numInputs);

%% Setup the optimization problem

% Create the given closed loop system and find its eigenvalues/vectors
CLstateMatrix = (A - B*K);
[T, D] = eig(CLstateMatrix);

% Create the optimization problem
cvx_begin SDP
    cvx_solver sedumi
    variable P(numStates, numStates) symmetric;
    variable Q(numStates, numStates) symmetric semidefinite;
    variable S(numStates, numInputs);
    
    minimize norm( inv(T)*B*(-K + inv(R)*(S' + B'*P))*T, 'fro')
    subject to:
        P >= 0;
        
        CLstateMatrix'*P + P*CLstateMatrix == 0;

        A'*P + P*A - (P*B + S)*K + Q.*Qform == 0;
        
        [ Q.*Qform, S;
         S', R ] - a*I>= 0;
cvx_end


Kopt  = lqr(A, B, Q, R, S);

end