% e481 project - Maglev controller design
% Miriam Lennig - 21297899
% James Mariotti - 26413080

% 1) ~~~~~~~~~ State and Output equations

% Linearization
% Scaled in units of 1 count per cm
% Sensor #1, 
e1_ = -10720;
f1_ = 692;
g1_ = -2.27; 
h1_ = -0.0000448;

% Sensor #2
e2_ = 5600;
f2_ = -555;
g2_ = 1.195;
h2_ = 0.0000595;

% What even are these?
a = 1.65;
b = 6.2;
c = 2.69;
d = 4.2;

% The units of the denominator in the RHS of 
% F_w = u1test / a(y1 + b) ^ 4
% are DAC counts per Newton. In order for Ku1Fu1 
% to have units of N/10000, a must be divided by 
% 10000. 
a_c = a/10000;
b_c = b;

% Scale efgh in units of 10000 counts per cm.
% Sensor #1, 
e1 = -10720 * 10000;
f1 = 692 * 10000;
g1 = -2.27 * 10000;
h1 = -0.0000448 * 10000;

% Sensor #2
e2 = 5600 * 10000;
f2 = -555 * 10000;
g2 = 1.195 * 10000;
h2 = 0.0000595 * 10000;

% C Matrix diagonal values
C1 = 1;
C2 = 1;
C3 = 1;
C4 = 1;

% Mass of each magnet in Newtons, +/- 0.04 N
m = 1.18; 


% kinematic Model
% |-^k1^-[M1]-^k12^-[M2]-^k2^-|
%        -F1->      -F2->
%      F1=ku1*u1  F2=ku2*u2

%Variables needed
syms k1 k2 k12 ku11 ku22 u1 u2 u1_0 y12_0 y1_0 u2_0 y2_0

% k gains
k1 = (4 * u1_0) / (a * (y1_0 + b)^5)
k2 = (4 * u2_0) / (a * (y2_0 + b)^5)
k12 = (4 * c) / (y12_0 + d)^5
ku1 = 1 / (a *(y1_0 + b)^4)
ku2 = 1 / (a *(-y2_0 + b)^4)


%MIMO or SIMO
if (mimoFlag) == FALSE {
    %MIMO Model - State space equations
    A = [0 1 0 0; (-(k1 + k12) / m) 0 (k12 / m) 0; 0 0 0 1; (k12 / m) 0 ((k2 - k12) / m) 0] 
    B = [0 0; (ku11/m) 0; 0 0; 0 (ku22/m)]
    C = [C1 0 0 0; 0 C2 0 0; 0 0 C3 0; 0 0 0 C4]
    D = [0 0; 0 0; 0 0; 0 0] 
    U = [u1; u2]
% }
% else {
%     %SISO Model - State space equations
%     
%       %Not done
%     A = [0 1 0 0; (-(k1 + k12) / m) 0 (k12 / m) 0; 0 0 0 1; (k12 / m) 0 ((k2 - k12) / m) 0] 
%     B = [0 0; (ku11/m) 0; 0 0; 0 (ku22/m)]
%     C = [C1 0 0 0; 0 C2 0 0; 0 0 C3 0; 0 0 0 C4]
%     %Done
%     D = [0 0; 0 0;] 
%     U = [u1]
% }
%[L,Z] = ss2tf(A,B,C,D,1)
%[L,Z] = ss2tf(A,B,C,D,2)

Co = ctrb(A,B)
Ob = obsv(A,C)




