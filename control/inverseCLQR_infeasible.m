function [Q, R, P, Kopt, a] = inverseCLQR_infeasible(K, A, B, NUM_ITER, lambda)
% INVERSECLQR_INFEASIBLE Calculate the inverse of the LQR problem if it is
% a set of infeasible LMIs
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
%  [Q, R, P, a] = inverseCLQR_infeasible(K, A, B, NUM_ITER)
%    Inputs: 
%       K = linear time-invariant gain matrix
%       A = Plant A matrix
%       B = Plant B matrix
%       NUM_ITER = Number of iterations for the infeasible algorithm
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

%% Create the CVX problem for the approximate solution
cvx_begin SDP
    variable P_0(numStates, numStates) symmetric;
    variable Q_0(numStates, numStates) symmetric semidefinite;
    variable R_0(numInputs, numInputs) symmetric semidefinite;
    variable S_0(numStates, numInputs);
    
    minimize norm(S_0, 'fro')
    subject to:
        P_0 >= 0;
        B'*P_0 + S_0' - R_0*K == 0;
        A'*P_0 + P_0*A - (P_0*B + S_0)*K + Q_0 == 0;
        [ Q_0, S_0;
         S_0', R_0 ] > 0;
cvx_end


%% Setup for the iterative method
Q = Q_0
R = R_0


%% Run the iterative method
for (k = 1:1:NUM_ITER)
    %disp(['Iteration ', num2str(k)]);
    [theta, ind, numQelements] = toTheta(Q, R);
    numElements = length(theta);
    
    theta_new = zeros(numElements, 1);
    
    % Find the K and P for the current iteration (using the continuous-time
    % Riccati equation
    [P, L, Kk] = care(A, B, Q, R);
    
    Abar = A - B*inv(R)*B'*P;
    
    % Iterate over the elements of theta
    for (i = 1:1:numElements)        
        % Compute the derivatives of Q and R for this element
        if (i <= numQelements)
            % It is an element of Q, so R is zero
            dR = zeros(numInputs, numInputs);
            
            % Create the matrix derivative
            dQ = matrixDeriv(Q, ind(i,2), ind(i,1) );
        else
            % It is an element of R, so Q is zero
            dQ = zeros(numStates, numStates);
            
            % Create the matrix derivative
            dR = matrixDeriv(inv(R), ind(i,2), ind(i,1) );
        end

        dP = lyap(Abar, dQ - P*B*dR*B'*P);
        
        dKv = vectorize( dR*B'*P + inv(R)*B'*dP );
        
        e = vectorize(Kk) - vectorize(K);
        
        de = dKv'*e + e'*dKv;
        
        theta_new(i) = theta(i) - lambda*de;
    end
    
    %% Form the new Q and R matrices
    [Qnew, Rnew] = fromTheta( theta_new, numStates, numInputs);
    
    eigenQ = eig(Qnew);
    eigenQ( abs(eigenQ) < 1e-10 ) = 0;
    if ( min( eigenQ ) >= 0 )
        Q = Qnew;
    end

    eigenR = eig(Rnew);
    eigenR( abs(eigenR) < 1e-10 ) = 0;
    if ( min( eigenR ) >= 0 )
        R = Rnew;
    end
    
end

    [P, L, Kopt] = care(A, B, Q, R);

end

function [ theta, ind, numQelements ] = toTheta( Q, R )
    % Append Q
    [row, col] = size(Q);
    theta = zeros( row*(row-1)/2, 1);
    
    k = 1;
    for (i=1:1:col)
        for (j=1:1:i)
            theta(k) = Q(j, i);
            ind(k,1) = i;
            ind(k,2) = j;
            k = k+1;
        end
    end
    numQelements = k-1;
    
    % Append R
    [row, col] = size(R);
    theta = [theta; zeros( row*(row-1)/2, 1)];
    
    for (i=1:1:col)
        for (j=1:1:i)
            theta(k) = R(j, i);
            ind(k,1) = i;
            ind(k,2) = j;
            k = k+1;
        end
    end
end

function [ Q, R ] = fromTheta( theta, numStates, numInputs )
    % Create Q
    Q = zeros(numStates);
    
    k = 1;
    for (i=1:1:numStates)
        for (j=1:1:i)
            Q(j, i) = theta(k);
            Q(i, j) = theta(k);
            k = k+1;
        end
    end
    
    % Create R
    R = zeros(numInputs);
    
    for (i=1:1:numInputs)
        for (j=1:1:i)
            R(j, i) = theta(k);
            R(i, j) = theta(k);
            k = k+1;
        end
    end
end

function [v] = vectorize(A)
    [row, col] = size(A);
    v = reshape(A, row*col, 1);
end

function [ A ] = unvectorize(v, row, col)
    A = reshape(v, row, col);
end

function [ J ] = matrixDeriv(A, row, col)
    [Arow, Acol] = size(A);
    J = zeros(Arow, Acol);
    J(row, col) = A(row, col);
end