% Open Loop system characterization, identification and representation.
% 1. State Space Representation
% 2. Transfer Function of Open Loop System
% 3. O.C.F, C.C.F, J.C.F.
% 4. Impulse Response & Step Response
% 5. Bode Plot & Root Locus 

% 1. State Space Representation
% SISO, Location #1, Linearized Actuator, Linearized Sensor
% p.133 3a)
A = [0 1; 0 0];
B = [0; 826];
C = [1 0];

% Create state-space model 
ss_ol = ss(A,B,C,0);

% 2. Transfer Function of Open Loop System
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

% 3. O.C.F, C.C.F, J.C.F. 
% Compute the observability matrix
Ob = obsv(A,C)
% Determine the number of unobservable states
unob = length(A)-rank(Ob)
```````````````

% Compute the controllability matrix 
Co = ctrb(A,B)
% Determine the number of uncontrollable states
unco = length(A) - rank(Co)

% Create Jordan form of matrix A
JA = jordan(A)

% 4. Impulse Response & Step Response
% Impulse Response
figure (1)
subplot(2,1,1)
impulse(ss_ol)
title('Impulse Reponse of Open-Loop Linearized Magnetic Levitation System');

% Step Response
subplot(2,1,2)
step(ss_ol)
title('Step Reponse of Open-Loop Linearized Magnetic Levitation System');

% 5. Bode Plot & Root Locus 
% Bode Plot
figure (2)
subplot(2,1,1)
bodeplot(ss_ol)
title('Bode Plot of Open-Loop Linearized Magnetic Levitation System');

% Root Locus
subplot(2,1,2)
rlocusplot(ss_ol)
title('Root Locus of Open-Loop Linearized Magnetic Levitation System');



