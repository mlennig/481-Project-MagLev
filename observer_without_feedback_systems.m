%%  Original Plant
a=[0 1; 0 0];
b=[0; 826];
c=[1 0];
d=0

sys=ss(a,b,c,d)
eig(sys)
rank(obsv(sys))

%% Observer pole placement at  -10 and -9
% This observer will lead to a fast approximation of the states
L_T=place(a',c',[-20 + 20i -20 - 20i])
L=L_T'


%% Observer pole placement at  -1 and -2
% This observer will lead to a slower approximation of the states
% L_anastrofos=place(a',c',[-1,-2])
% L=L_anastrofos'



% State observer Feedback

a=[-20.6 1;0 -1]
b=[0;1]
c=[1 1]
d=0
sys=ss(a,b,c,d)
eig(sys)
rank(obsv(sys))
rank(ctrb(sys))
K=place(a,b,[-5 -6])

open_system('modelname')
open_system('modelname/Subsystem')





