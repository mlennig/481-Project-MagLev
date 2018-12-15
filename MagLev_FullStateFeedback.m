% 9. Full State Feedback Control
% Design Criteria
%   a. Transient Design Specification:  < 10% overshoot
%   b. Steady-state Design Specification: Settling Time < 1 second
% 10. Step Response, Square Wave Response, Sinusoidal Response, Transfer Function of Controller 

% 1. Choose closed-loop pole locations
% 2. Find K values: det(sI - A + BK) = Characteristic Polynomial (C.P.) of
% closed loop system

%%%%%%%%%%%%% SISO, Location #1, Linearized Actuator, Linearized Sensor %%%%%%%%%%%%% 
% p.133 3a)
A = [0 1; 0 0];
B = [0; 826];
C = [1 0];

% Create state-space model 
ss_ol = ss(A,B,C,0);

% Convert state-space representation to transfer function
[num,denom]= ss2tf(A,B,C,0)
tf_ol = tf(num,denom)

%%% Control Design Using Pole Placement %%%
%%%% Full-State Feedback System %%%%

% Full-sate means that all state variables are
% known to the controller at all times. 
% For our system, we have a sensor measuring 
% 1. Magnet #1's position
% 2. Magnet #1's velocit

% Let reference = r = 0. 
% Input is then u = -Kx
% State space eq'sn for CL Feedback System:
% xdot = Ax + B(-Kx) = (A - BK)x
% y = Cx

% Stability & time-domain performance of the CL sys
% are primarily determined by the location of the 
% eigenvalues of the matrix (A - BK).
% A, BK are both 2x2 matrices, which
% means there exist 2 poles for the system. 

% By choosing an appropriate state-feedback gain 
% matrix K, we can place the CL poles anywhere, 
% since the system is controllable. 

% After testing different complex poles, 
% we place two complex poles such that
% the overshoot is not too large. 
% The further to the left we place the poles, 
% the faster the response of the system, and 
% the better the transient response. 
% However, the farther we move the poles to the left,
% the more control effort is required. 
p1 = -20 + 20i;
p2 = -20 - 20i;

% Find state-feedback gain, K, 
% which will provide the desired 
% closed-loop poles. 
K = place(A,B,[p1 p2]);

sys_cl = ss(A-B*K,B,C,0);   % Generate closed-loop system
TFFS = tf(sys_cl)           % Obtain transfer function of the closed-loop system     

% 10. Step Response, Square Wave Response, Sinusoidal Response, Transfer Function of Controller 
% Obtain Step Response of system with Full State Feedback Controller 
figure(1)
subplot(3,1,1)
step(TFFS)
title('Step Response of System with Full-State Feedback Controller')

% Obtain Step Response of system with Full State Feedback Controller
subplot(3,1,2)
[u_square,t] = gensig('square',4,10,0.1);
lsim(TFFS,u_square,t)
title('Square Wave Response of System with Full-State Feedback Controller')

% Obtain Step Response of system with Full State Feedback Controller
subplot(3,1,3)
[u_sin,t] = gensig('sin',4,10,0.1);
lsim(TFFS,u_sin,t)
title('Sinusoidal Response of System with Full-State Feedback Controllerr')

