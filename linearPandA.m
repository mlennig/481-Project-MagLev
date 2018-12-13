%3a) linearized plant and sensor

    A = [0 1; 0 0] 
    B = [0 ; 826 ]
    C = [1 0]
    D = [0]
    %D = [0 0; 0 0; 0 0; 0 0] 
%    U = [u1]
    
J =jordan(A)
[num,denom]= ss2tf(A,B,C,D)
sys1 = tf(num,denom)
Co = ctrb(A,B)
Ob = obsv(A,C)

figure (1)
subplot(2,2,1)
bodeplot(sys1)
subplot(2,2,2)
rlocusplot(sys1)
subplot(2,2,3)

%sisotool(sys)
step(sys1)
subplot(2,2,4)
impulse(sys1)

% take the form S^2+2zwS+w^2
% settling time = 1/wn
%0.1



K=place(A,B,[-25 -40])

Acl=A-B*K
Bcl=B
Ccl=C
Dcl=0
Syscl=ss(Acl,Bcl,Ccl,Dcl)
t=[0:0.001:2]
ref=1*ones(size(t))
[y,t,x]=lsim(Syscl, ref,t )

figure (2)
plot(t,y)