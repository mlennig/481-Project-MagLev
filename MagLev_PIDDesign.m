% PID Design
% 1. Design Criteria
%   a. Transient Design Specification
%   b. Steady-state Design Specification
% 2. Design PID
% 3. Introduction of Noise 
% 4. Step Response
% 5. Square Wave Response
% 6. Sinusoidal Response 


% SISO, Location #1, Linearized Actuator, Linearized Sensor
% p.133 3a)
A = [0 1; 0 0];
B = [0; 826];
C = [1 0];

% Create state-space model 
ss_ol = ss(A,B,C,0);

% Convert state-space representation to transfer function
[num,denom]= ss2tf(A,B,C,0)
tf_ol = tf(num,denom)

% Find poles of open-loop system
% by finding the eigenvalues of the 
% system matrix A. These poles are
% equal to the poles of the transfer
% function, and determine stability. 
% det(sI - A) = 0
poles = eig(A)

% 6. Design a PID Controller
    % Design Specifications: 
        % Stable
        % less than 10% overshoot
        % Settling time of < 1 second

sisotool(ss_ol)
%Exported Controller Designs
%Design 1 - PID
PIDcontroller
TFPID
%PID=pid(PIDcontroller/0.048798)
%[Kp1,Ki1,Kd1,Tf1] = piddata(PIDcontroller)

%Design 2 - PD
PDcontroller
TFPD
[Kp1,Ki1,Kd1,wse;/'.Z"Tf2] = piddata(PDcontroller)