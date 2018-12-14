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

%sisotool(ss_ol)
%Exported Controller Designs
%Design 1 - PID
PIDcontroller
TFPID

% 7. Design 1 ? PID: Step, Square Wave, and Sinusoidal Responses
% Obtain Step Response of system with PID 
figure(1)
subplot(3,2,1)
step(TFPID)
title('Step Response of System with PID Controller')

% Obtain Square Wave Response of system with PID
subplot(3,2,3)
[u_square,t] = gensig('square',4,10,0.1);
lsim(TFPID,u_square,t)
title('Square Wave Response of System with PID Controller')

% Obtain Sinusoidal Response of system with PID 
subplot(3,2,5)
[u_sin,t] = gensig('sin',4,10,0.1);
lsim(TFPID,u_sin,t)
title('Sinusoidal Response of System with PID Controller')

%Design 2 - PD
PDcontroller
TFPD
[Kp1,Ki1,Kd1,Tf2] = piddata(PDcontroller)

% 7. Design 2 ? PD: Step, Square Wave, and Sinusoidal Responses
% Obtain Step Response of system with PID 
subplot(3,2,2)
step(TFPD)
title('Step Response of System with PD Controller')

% Obtain Square Wave Response of system with PID
subplot(3,2,4)
[u_square,t] = gensig('square',4,10,0.1);
lsim(TFPD,u_square,t)
title('Square Wave Response of System with PD Controller')

% Obtain Sinusoidal Response of system with PID 
subplot(3,2,6)
[u_sin,t] = gensig('sin',4,10,0.1);
lsim(TFPD,u_sin,t)
title('Sinusoidal Response of System with PD Controller')



%Design 2 - PD
PDcontroller
TFPD
[Kp1,Ki1,Kd1,Tf2] = piddata(PDcontroller)

% 7. Design 2 ? PD: Step, Square Wave, and Sinusoidal Responses
% Obtain Step Response of system with PID 
subplot(3,2,2)
step(TFPD)
title('Step Response of System with PD Controller')

% Obtain Square Wave Response of system with PID
subplot(3,2,4)
[u_square,t] = gensig('square',4,10,0.1);
lsim(TFPD,u_square,t)
title('Square Wave Response of System with PD Controller')

% Obtain Sinusoidal Response of system with PID 
subplot(3,2,6)
[u_sin,t] = gensig('sin',4,10,0.1);
lsim(TFPD,u_sin,t)
title('Sinusoidal Response of System with PD Controller')


% 8. Noise Injection

% Design 1 ? PID: Step, Square Wave, and Sinusoidal Responses
% Obtain Step Response of system with PID 
figure(2)
subplot(3,2,1)
t = 0:0.01:10;
u_step = 0.001*ones(size(t));
% Inject white noise into the system
y_step = awgn(u_step,10,'measured');
plot(t,[u_step y])
legend('Original Signal','Signal with AWGN')
title('Step Response of System with PID Controller')

% Obtain Square Wave Response of system with PID
subplot(3,2,3)
[u_square,t] = gensig('square',4,10,0.1);
% Inject white noise into the system
y_square = awgn(u_square,10,'measured');
plot(t,[u_square y])
legend('Original Signal','Signal with AWGN')
title('Square Wave Response of System with PID Controller')

% Obtain Sinusoidal Response of system with PID 
subplot(3,2,5)
[u_sin,t] = gensig('sin',4,10,0.1);
% Inject white noise into the system
y_sin = awgn(u_sin,10,'measured');
plot(t,[u_sin y])
legend('Original Signal','Signal with AWGN')
title('Sinusoidal Response of System with PID Controller')