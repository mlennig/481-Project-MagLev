% 11. Reduced Order Observer
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

a11 = 0;
A1e = 1;
b1 = 0;
Ae1 = 0;
Aee = 0;
Be = 826;

% Let tau = 0.1 sec and zeta = 1
s = a11;
alpha = s + 10;

Ge = alpha * inv(A1e) * 1;

A_ = Aee - Ge * A1e
B_ = Ae1 - Ge * a11 + Aee * Ge - Ge * A1e * Ge
C_ = Be - Ge * b1


% Create state-space model 
ss_ol = ss(A,B,C,0);
TFOL = tf(ss_ol)

% u = -K1y - Ke * x_e

