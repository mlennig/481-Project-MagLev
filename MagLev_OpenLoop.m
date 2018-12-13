% Open Loop system characterization, identification and representation.
% 1. State Space Representation
% 2. Transfer Function of Open Loop System
% 3. O.C.F, C.C.F, J.C.F.
% TODO
% 1. O.C.F
% 2. C.C.F
% 3. Impulse Response
% 4. Step Response
% 5. Bode Plot Open Loop 
% 6. Root Locus Open Loop 

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



figure(1);
plot(t,y);
title('Open-Loop Response to Non-Zero Initial Condition');
xlabel('Time (sec)');
ylabel('Magnet Position (cm)');




figure (2)
subplot(2,1,1)
bodeplot(sys1)
title('Bode Plot of Open-Loop Linearized Magnetic Levitation System');
subplot(2,1,2)
rlocusplot(sys1)
title('Root Locus of Open-Loop Linearized Magnetic Levitation System');

figure(3)
subplot(2,1,1)
step(sys1)
title('Step Reponse of Open-Loop Linearized Magnetic Levitation System');
subplot(2,1,2)
impulse(sys1)
title('Impulse Reponse of Open-Loop Linearized Magnetic Levitation System');

%sisotool(sys)

