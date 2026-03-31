clc; clear;
W = 40000;
rho = 1.225;
S = 22.3;
b = 15;
AR = b^2/S;
C_Lmax = 2.7;
e = 0.9;
k = 1/(pi*AR*e);
V_stall = sqrt((2*W)/(rho*S*C_Lmax));
V_Lo = 1.2*V_stall;
v_avg = 1.2*V_stall*.7;
C_D0 = 0.04;
mu_r = 0.4;
g = 9.81;
h = 15.24;

q_avg = 0.5*rho*v_avg^2;

D_phi = q_avg*S*C_D0 + k*W^2/(q_avg*S);

L = q_avg*S*C_Lmax;

T = (1.44*W^2)/(152.4*rho*S*g*C_Lmax) + (D_phi+mu_r*(W-L));
q_Lo = 0.5*rho*V_stall*1.2;
D_Lo = q_Lo*S*C_D0 + k*W^2/(q_Lo*S);
ROC = V_Lo*(T - D_Lo)/W;
gamma = atan(ROC/V_Lo);
d_extra = 152.4 - d_Lo



P = T*v_avg;
fprintf('The thrust requirement is : %3.2f kN\n', T/1000)
fprintf('The power requirement is : %3.2f kW\n', P/1000)
