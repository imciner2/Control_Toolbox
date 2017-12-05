% This script is an example using the inverse LQR function.
%
% The example is taken from this paper:
%  M. Priess, et. al. (2015) , "Solutions to the Inverse LQR Problem with
%  Application to Bilogical Systems Analysis", IEEE Transactions on Control
%  System Technology, 23, 2.
%  DOI: 10.1109/TCST.2014.2343935

%% The plant matrices
A = [ 0,  1,  7,  9;
      4, -8, -5, -3;
      8, -7,  7, -6;
     10, -5, -5, -5];
B = [ 2,  2,  5, -9;
     -1,  1,  5, -9;
     -3,  9, -3,  1;
      7, -4,  1,  6];

C = eye(4);


%% The controller
Ke = [ 2.376,   -1.328,  1.188,   1.847;
       4.294,  -0.8621,   3.77,   1.509;
      -2.279,    1.366, -2.067,  -1.323;
        -2.7, -0.06092, -2.036, -0.7217];

    
%% Solve the inverse LQR problem
[Q, R, P] = inverseCLQR_feasible(Ke, A, B);

Q
R

%% The plant matrices
A = [  100,   0, -1;
         0, 0.1, 50;
     0.333,  10,  0];
B = [ -1,   0,  10;
       1,   1,   0;
     0.1, -20,   4];

C = eye(3);


%% The controller
Kplace = place(A, B, [-90, -20, -10])
Ke = [ -3.47,   20.2,  49.3;
         3.7, 0.0519, 0.714;
        18.7,   2.21,  4.83];

    
%% Solve the inverse LQR problem
[Q, R, P, Kopt] = inverseCLQR_infeasible(Kplace, A, B, 2000, 10000);

Q
R
Kopt