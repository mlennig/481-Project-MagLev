% i. Open Loop system characterization, identification and representation.

% State Equations 
% SISO, Location #1, Linearized Actuator, Linearized Sensor
% p.133 3a)
A = [0 1; 0 0];
B = [0; 826];
C = [1 0];

% Find poles of open-loop system
% by finding the eigenvalues of the 
% system matrix A. These poles are
% equal to the poles of the transfer
% function, and determine stability. 
% det(sI - A) = 0
poles = eig(A)

% Provide a non-zero initial condition
% to the system to observe what happens. 

% Specify the time samples for the simulation
% t = 0:dt:Tfinal
% dt = t(2) - t(1), used to discretize the cont. model
t = 0:0.01:2;
% Specify input value of 1 at t(i). 
u = zeros(size(t));
% Specify an initial condition for the system states. 
% x0 is a vector whose entries are the values of the
% corresponding states of sys given chosen arbitrary 
% initial conditions. 
x0 = [0.01 0];

% Create state-space model 
sys = ss(A,B,C,0);

% Convert state-space representation to transfer function
[num,denom]= ss2tf(A,B,C,0)
sys1 = tf(num,denom)

% Use lsim to simulate the response of sys given inputs
% y = array, system response
% t = time vector used for simulation 
% x = state trajectories 
[y,t,x] = lsim(sys,u,t,x0);

figure(1);
plot(t,y);
title('Open-Loop Response to Non-Zero Initial Condition');
xlabel('Time (sec)');
ylabel('Magnet Position (cm)');


% Create Jordan form of matrix A
JA = jordan(A)

% Compute the observability matrix
Ob = obsv(A,C)
% Determine the number of unobservable states
unob = length(A)-rank(Ob)

m = ssest(data,n,'Form','canonical')

% Compute the controllability matrix 
Co = ctrb(A,B)
% Determine the number of uncontrollable states
unco = length(A) - rank(Co)

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

