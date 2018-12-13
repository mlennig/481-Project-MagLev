% Full State Feedback
% 1. Design Criteria
%   a. Transient Design Specification
%   b. Steady-state Design Specification 
% 2. Step Response
% 3. Square Wave Response
% 4. Sinusoidal Response 
% 5. Transfer Function of Controller 

%%%%%%%%%%%%%%%%%%%%% Website work begins http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace

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


% Use lsim to simulate the response of sys given inputs
% y = array, system response
% t = time vector used for simulation 
% x = state trajectories 
[y,t,x] = lsim(ss_ol,u,t,x0);


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
%p1 = -20
%p2 = -21

% Find state-feedback gain, K, 
% which will provide the desired 
% closed-loop poles. 
K = place(A,B,[p1 p2]);

% Generate closed-loop system
sys_cl = ss(A-B*K,B,C,0);

[y,t,x] = lsim(sys_cl,u,t,x0);

figure(2)
plot(t,y)
xlabel('Time (sec)')
ylabel('Magnet #1 Position (cm)')

%%% Introducing the Reference Input %%%

% We choose a small step input, such
% that our linearization remains valid. 
t = 0:0.01:2;
u = 0.001*ones(size(t));

sys_cl = ss(A-B*K,B,C,0);
lsim(sys_cl,u,t);
%subplot(3,1,2)
%xlabel('Time (sec)')
%ylabel('Magnet #1 Position (cm)')
%axis([0 2 -4E-6 0])

% Scale the reference input to make it 
% equal to the Kx in steady-state.  
Nbar = rscale(sys,K)

[y,t,x] = lsim(sys_cl,Nbar*u,t)
figure(3)
plot(t,y)
title('Linear Simulation Results (with Nbar)')
xlabel('Time (sec)')
ylabel('Magnet #1 Position (cm)')
axis([0 2 0 1.2*10^-3])

%%% Observer Design %%%
% Observers are used when we can't measure
% all state variables x. 
% For the Magnetic levitation system, 
% we add two new estimated state variables, 
% xhat to the system. 
% The observer is like a copy of the plant:
% It has the same input and a similar D.E.
% An additional term compares the actual measured 
% output, y, to the estimated output, yhat = Cxhat, 
% and helps correct the estimated state xhat, such
% that it approaches the actual value of the x state. 

% The error dynamics of the observer are given by
% the poles of A - LC.

% The first step is to choose the observer gain L. 
% We want the dynamics of the observer to be much faster
% than the system itself, so we place the poles
% 5x farther to the left of the dominant poles of the sys. 
op1 = -100;
op2 = -101;

% Replace matrix B by the Matrix C and take
% the transposes of each matrix to get the 
% observability matrices. 
L = place(A',C',[op1 op2])';

% The estimated state feedback u = -Kxhat
% is used to find the state and error
% equations for full-state feedback with an observer
At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B*Nbar
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

% Test the system response to a non-zero initial
% conidition with no reference input. 
% We assume the observer begins with an initial
% estimate equal to zero, such that the initial 
% estimation error is equal to the initial state vector
% e = x. 
sys = ss(At,Bt,Ct,0);
[y,t,x] = lsim(sys,zeros(size(t)),t,[x0 x0]);
figure(4)
plot(t,y)
title('Linear Simulation Results (with observer)')
xlabel('Time (sec)')
ylabel('Magnet #1 Position (cm)')

t = 0:1E-6:0.1;
x0 = [0.01 0.5];
[y,t,x] = lsim(sys,zeros(size(t)),t,[x0 x0]);

n = 2;
e = x(:,n+1:end);
x = x(:,1:n);
x_est = x - e;

% Save state variables explicitly to aid in plotting
y = x(:,1); y_dot = x(:,2); 
y_est = x_est(:,1); y_dot_est = x_est(:,2); 
figure(5)
plot(t,y,'-r',t,y_est,':r',t,y_dot,'-b',t,y_dot_est,':b')
legend('y','y_{est}','ydot','ydot_{est}')
xlabel('Time (sec)')

%%%%%%%%%%%%%%%%%% End website work from http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace







% Experiment 10 - Full Order and Reduced Order Observer Design
 % By Siddharth Kaul
 %--------------------------------------------------------------------------
 %
 % Given -------------------------------------------------------------------
 num = [0 0 826];
 den = [1 0 0];
 sys = tf(num,den);
 desiredPoles = [-100 -10];
 disp('Given Transfer Function: ');
 sys
 % Now finding whether observable or not -----------------------------------
 [MatrixA,MatrixB,MatrixC,MatrixD] = tf2ss(num,den)
 mysys = canon(ss(MatrixA,MatrixB,MatrixC,MatrixD),'companion')
 
 % If observable then only continue other wise no oberver design possible --
if(rank(MatrixA)==rank(mysys.A))
disp('System is Observable');

% Observer Design Possible --------------------------------------------
observerGain = acker(MatrixA.',MatrixC.',desiredPoles.').';
disp('Observer Gain Matrix');
disp(observerGain);

% New System with observer --------------------------------------------
newMatrixA = MatrixA - (observerGain*MatrixC);

newMatrixB = eye(rank(MatrixA));
newMatrixC = eye(rank(MatrixA));
newMatrixD = eye(rank(MatrixA));

mysys = ss(newMatrixA,newMatrixB,newMatrixC,newMatrixD);
mysys;

timeT = 0:.1:2;

initialX = [1 0 0];

x = initial(mysys, initialX,timeT);
x1 = [1 0]*x';
x2 = [0 1]*x';


subplot(3,1,1);
plot(timeT,x1,'r',timeT,x2,'g');
title('Response to initial Condition of State Variables Observer');
xlabel('Time -->');
ylabel('Magnitude -->');
disp('New System Matrix A');
disp(newMatrixA);
else
disp('System is Unobservable');
% No observer design possible -----------------------------------------
end
%--------------------------------------------------------------------------
% Now first we need to make controller with given specification of 20.8%
% overshoot and 4 seconds settling time. For such a controller we
% calculated the required poles to be
% [-1+.637i -1-.637i -4]
%--------------------------------------------------------------------------
% So ----------------------------------------------------------------------
% we already have MatrixA -------------------------------------------------
desiredPoles = [-1+.637i -1-.637i];
controllerGain = acker(MatrixA,MatrixB,desiredPoles);
disp('Controller Gain is ');
disp(controllerGain);
newMatrixA = MatrixA - (MatrixB*controllerGain);
newMatrixB = eye(3);
newMatrixC = eye(3);
newMatrixD = eye(3);
initialX = [1 0];
timeT = 0:.1:5;
mysys = ss(MatrixA,newMatrixB,newMatrixC,newMatrixD);
x = initial(mysys, initialX,timeT);
x1 = [1 0]*x';
x2 = [0 1]*x';
subplot(3,1,2);
plot(timeT,x1,'r',timeT,x2,'g');
title('Response to initial Condition of State Variables Controller');
xlabel('Time -->');
ylabel('Magnitude -->');
%--------------------------------------------------------------------------
% Now finding the reduced order observer that is 10 times faster than the
% designed controller.
% With this criteria in mind we can get the MatrixAaa, MatrixAb and so on
% and the desired observer poles becomes -10+6.377i -10-6.477i
%--------------------------------------------------------------------------
% So ----------------------------------------------------------------------
MatrixAaa = [0];
MatrixAab = [1 0];
MatrixAba = [0;-10];
MatrixAbb = [0 1;-17 -8];
desiredObserverPoles = [-10+6.377i -10-6.377i];
observerGain = acker(MatrixAbb.',MatrixAab.',desiredObserverPoles.').'
newMatrixAAbb = MatrixAbb - (observerGain*MatrixAab);
newMatrixB = eye(2);
newMatrixC = eye(2);
newMatrixD = eye(2);
mysys = ss(newMatrixAAbb,newMatrixB,newMatrixC,newMatrixD);
initialX = [1 0];
timeT = 0:.1:2;
x = initial(mysys,initialX,timeT);
x1 = [1 0]*x';
x2 = [0 1]*x';
subplot(3,1,3);
plot(timeT,x1,'r',timeT,x2,'g');
title('Response to initial Condition of State Variables Reduced Observer');
xlabel('Time -->');
ylabel('Magnitude -->');
% -------------------------------------------------------------------------
% End of Program. Created By Siddharth Kaul
% -------------------------------------------------------------------------