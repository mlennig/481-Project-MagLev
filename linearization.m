syms e f g h

plot(y1raw,y1cal)
title('Graph of calibration data')
ylabel('y1cal')
xlabel('y1raw')

eqn1 = e/y1raw(1) + f/sqrt(y1raw(1)) + g + h*y1raw(1) == y1cal(1) 
eqn2 = e/y1raw(2) + f/sqrt(y1raw(2)) + g + h*y1raw(2) == y1cal(2) 
eqn3 = e/y1raw(3) + f/sqrt(y1raw(3)) + g + h*y1raw(3) == y1cal(3) 
eqn4 = e/y1raw(4) + f/sqrt(y1raw(4)) + g + h*y1raw(4) == y1cal(4) 
eqn5 = e/y1raw(5) + f/sqrt(y1raw(5)) + g + h*y1raw(5) == y1cal(5) 
eqn6 = e/y1raw(6) + f/sqrt(y1raw(6)) + g + h*y1raw(6) == y1cal(6) 
eqn7 = e/y1raw(7) + f/sqrt(y1raw(7)) + g + h*y1raw(7) == y1cal(7)
eqn8 = e/y1raw(8) + f/sqrt(y1raw(8)) + g + h*y1raw(8) == y1cal(8)

sol = solve([eqn2, eqn3, eqn4,eqn5], [e, f, g, h]);
eSol = sol.e
fSol = sol.f
gSol = sol.g
hSol = sol.h

vpa(eSol,6)

% [A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4], [e, f, g, h])
% 
% 
% x = linsolve(A,B)
% 
% simplify(eSol)
% simplify(fSol)
% simplifyFraction(gSol)
% simplifyFraction(hSol)

