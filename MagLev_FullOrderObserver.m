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

desiredPoles = [-20 + 20i -20 - 20i]

% Observer Design Possible --------------------------------------------
observerGain = acker(A.',C.', desiredPoles.').';
disp('Observer Gain Matrix');
disp(observerGain);
% New System with observer --------------------------------------------
newA = A - (observerGain*C);
newB = eye(rank(A));
newC = eye(rank(A));
mysys = ss(newA,newB,newC,0);
mysys;
timeT = 0:.1:2;
initialX = [1 0];
x = initial(mysys,initialX,timeT);
x1 = [1 0]*x';
x2 = [0 1]*x';
subplot(3,1,1);
plot(timeT,x1,'r',timeT,x2,'g');
title('Response to initial Condition of State Variables Observer');
xlabel('Time -->');
ylabel('Magnitude -->');
disp('New System Matrix A');
disp(newA);
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
% we already have A -------------------------------------------------
desiredPoles = [-1+.637i -1-.637i -4];
controllerGain = acker(A,B,desiredPoles);
disp('Controller Gain is ');
disp(controllerGain);
newA = A - (B*controllerGain);
newB = eye(3);
newC = eye(3);
newD = eye(3);
initialX = [1 0 0];
timeT = 0:.1:5;
mysys = ss(A,newB,newC,newD);
x = initial(mysys, initialX,timeT);
x1 = [1 0 0]*x';
x2 = [0 1 0]*x';
x3 = [0 0 1]*x';
subplot(3,1,2);
plot(timeT,x1,'r',timeT,x2,'g',timeT,x3,'b');
title('Response to initial Condition of State Variables Controller');
xlabel('Time -->');
ylabel('Magnitude -->');
%--------------------------------------------------------------------------
% Now finding the reduced order observer that is 10 times faster than the
% designed controller.
% With this criteria in mind we can get the Aaa, Ab and so on
% and the desired observer poles becomes -10+6.377i -10-6.477i
%--------------------------------------------------------------------------
% So ----------------------------------------------------------------------
Aaa = [0];
Aab = [1 0];
Aba = [0;-10];
Abb = [0 1;-17 -8];
desiredObserverPoles = [-10+6.377i -10-6.377i];
observerGain = acker(Abb.',Aab.',desiredObserverPoles.').'
newAAbb = Abb - (observerGain*Aab);
newB = eye(2);
newC = eye(2);
newD = eye(2);
mysys = ss(newAAbb,newB,newC,newD);
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