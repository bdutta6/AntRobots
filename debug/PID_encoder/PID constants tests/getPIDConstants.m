function[kp,kd,ki] = getPIDConstants(ku,Pu)
%for motor encoder A, i found ku and Pu to be 1.0625 and 11.76, respectively
kp = 0.6*ku;
Td = 0.125*Pu;
Ti = 0.5*Pu;

kd = Td * kp;

ki = kp / Ti;

end
