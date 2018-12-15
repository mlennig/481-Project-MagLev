% 11. Full Order Observer
%  a) Step Response
%  b) Sinusoidal Response 
%  c) Transfer Function of Controller 

%%%%%%%%%%%%% SISO, Location #1, Linearized Actuator, Linearized Sensor %%%%%%%%%%%%% 
% p.133 3a)
A = [0 1; 0 0];
B = [0; 826];
C = [1 0];

% Create state-space model 
ss_ol = ss(A,B,C,0);
TFOL = tf(ss_ol)

desiredPoles = [-20 + 20i -20 - 20i]
K = place(A,B,desiredPoles);
Nbar = rscale(ss_ol,K)

% Find Observer Gain G
observerGain = acker(A.',C.', desiredPoles.').';
disp('Observer Gain Matrix');
disp(observerGain);
G = observerGain;

% Calculate New System with Observer
At = [ A-B*K             B*K
       zeros(size(A))    A-G*C ];

Bt = [    B*Nbar
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

sys_cl_FullObs = ss(At,Bt,Ct,0);

% Generate transfer function of controller-estimator
TFFO = tf(sys_cl_FullObs)

% Transfer function of system with full order controller estimator: 
% Gec * Gp / (1 + Gec*Gp)
TFFO_sys = TFFO*TFFS/(1 + TFFO*TFFS)

% 11. Step Response, Square Wave Response, Sinusoidal Response, Transfer Function of Controller 
% Obtain Step Response of system with Controller-Estimator (Full Observer)
figure(1)
subplot(3,2,1)
step(TFFO_sys)
title('Step Response of System with Controller-Estimator (Full Order)')

% Obtain Step Response of system with Controller-Estimator (Full Observer)
subplot(3,2,3)
[u_square,t] = gensig('square',4,10,0.0001);
lsim(TFFO_sys,u_square,t)
title('Square Wave Response of System with Controller-Estimator (Full Order)')

% Obtain Step Response of system with Controller-Estimator (Full Observer)
subplot(3,2,5)
[u_sin,t] = gensig('sin',4,10,0.001);
lsim(TFFO_sys,u_sin,t)
title('Sinusoidal Response of System with Controller-Estimator (Full Order)')

% Noise Injection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% Obtain Step Response of system with Controller-Estimator (Full Observer) with NOISE
subplot(3,2,2)
t_step = 0:0.01:0.6;
u_step = 0.001*ones(size(t_step));
% Inject white noise into the system
y_step = awgn(u_step,15,'measured');
lsimplot(TFFO_sys,y_step,t_step)
title({'\fontsize{16}PID Controller';'\fontsize{11}Step response with SNR 15'})

% Obtain Square Wave Response of system with Controller-Estimator (Full
% Observer) with NOISE
subplot(3,2,4)
[u_square,t] = gensig('square',4,10,0.1);
% Inject white noise into the system
y_square = awgn(u_square,15,'measured');
lsimplot(TFFO_sys,y_square,t)
%plot(t,[u_square y_square])
title('Square Wave Response with SNR 15')

% Obtain Sinusoidal Response of system with Controller-Estimator (Full
% Observer) with NOISE
subplot(3,2,6)
[u_sin,t] = gensig('sin',4,10,0.1);
% Inject white noise into the system
y_sin = awgn(u_sin,15,'measured');
lsimplot(TFFO_sys,y_sin,t)
%plot(t,[u_sin y])
title('Sinusoidal Response with SNR 15')



