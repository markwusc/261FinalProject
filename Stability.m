clc; clear; close all

%% Given aircraft geometry
S = 44.9;          % wing area, m^2
b = 18;            % wingspan, m
c_bar = S/b;       % approximate mean aerodynamic chord, m
AR = b^2/S;        % aspect ratio

%% Tail geometry
S_H = 14;          % horizontal tail area, m^2
S_V = 6.2;         % vertical tail area, m^2
l_H = 6.5;         % horizontal tail arm, m
l_V = 6.5;         % vertical tail arm, m

%% Assumptions
eta_H = 0.90;      % horizontal tail dynamic pressure ratio
eta_V = 0.95;      % vertical tail dynamic pressure ratio
de_da = 0.35;      % downwash gradient
lambda = 0.45;     % wing taper ratio

CLalpha_H = 4.0;   % horizontal tail lift curve slope, 1/rad
CLalpha_V = 4.0;   % vertical tail lift curve slope, 1/rad

tau_e = 0.5;       % elevator effectiveness
tau_r = 0.5;       % rudder effectiveness

%% Wing lift curve slope
CLalpha_w = (2*pi*AR)/(2 + sqrt(4 + AR^2));

%% Tail volume coefficients
V_H = S_H*l_H/(S*c_bar);
V_V = S_V*l_V/(S*b);

%% Neutral point and static margin
h_ac_w = 0.25;     % wing aerodynamic center location, x/c
h_np = h_ac_w + eta_H*V_H*(CLalpha_H/CLalpha_w)*(1 - de_da);

SM = 0.10;         % desired static margin
h_cg = h_np - SM;

%% Pitch derivatives
Cm_alpha = -CLalpha_w*SM;
Cm_q = -2*eta_H*CLalpha_H*V_H*(l_H/c_bar);
Cm_delta_e = -eta_H*V_H*CLalpha_H*tau_e;

%% Roll derivatives
Cl_beta = -0.08;
Cl_p = -(CLalpha_w/12)*(1 + 3*lambda);
Cl_r = 0.15;
Cl_delta_a = 0.10;

%% Yaw derivatives
Cn_beta = eta_V*V_V*CLalpha_V;
Cn_r = -2*eta_V*V_V*CLalpha_V*(l_V/b);
Cn_p = -0.04;
Cn_delta_r = eta_V*V_V*CLalpha_V*tau_r;

%% Print results
fprintf('Wing Area S = %.2f m^2\n', S);
fprintf('Wingspan b = %.2f m\n', b);
fprintf('Mean chord c_bar = %.2f m\n', c_bar);
fprintf('Aspect ratio AR = %.2f\n\n', AR);

fprintf('Horizontal tail volume coefficient V_H = %.3f\n', V_H);
fprintf('Vertical tail volume coefficient V_V = %.3f\n\n', V_V);

fprintf('Neutral point h_np = %.3f c_bar\n', h_np);
fprintf('Desired static margin SM = %.3f\n', SM);
fprintf('Required CG location h_cg = %.3f c_bar\n\n', h_cg);

fprintf('Pitch derivatives:\n');
fprintf('Cm_alpha = %.3f 1/rad\n', Cm_alpha);
fprintf('Cm_q = %.3f\n', Cm_q);
fprintf('Cm_delta_e = %.3f 1/rad\n\n', Cm_delta_e);

fprintf('Roll derivatives:\n');
fprintf('Cl_beta = %.3f 1/rad\n', Cl_beta);
fprintf('Cl_p = %.3f\n', Cl_p);
fprintf('Cl_r = %.3f\n', Cl_r);
fprintf('Cl_delta_a = %.3f 1/rad\n\n', Cl_delta_a);

fprintf('Yaw derivatives:\n');
fprintf('Cn_beta = %.3f 1/rad\n', Cn_beta);
fprintf('Cn_r = %.3f\n', Cn_r);
fprintf('Cn_p = %.3f\n', Cn_p);
fprintf('Cn_delta_r = %.3f 1/rad\n', Cn_delta_r);

%% Stability checks

fprintf('\nSTABILITY CHECKS:\n');

%% Longitudinal static stability
% Requirement: Static margin > 0 and Cm_alpha < 0

if SM > 0
    fprintf('PASS: Static margin is positive. SM = %.3f\n', SM);
else
    fprintf('FAIL: Static margin is not positive. SM = %.3f\n', SM);
end

if Cm_alpha < 0
    fprintf('PASS: Longitudinal static stability. Cm_alpha = %.3f 1/rad\n', Cm_alpha);
else
    fprintf('FAIL: Aircraft is longitudinally unstable. Cm_alpha = %.3f 1/rad\n', Cm_alpha);
end

%% Pitch damping
% Requirement: Cm_q < 0

if Cm_q < 0
    fprintf('PASS: Pitch damping is stable. Cm_q = %.3f\n', Cm_q);
else
    fprintf('FAIL: Pitch damping is unstable. Cm_q = %.3f\n', Cm_q);
end

%% Elevator control authority
% Requirement: Cm_delta_e should usually be negative

if Cm_delta_e < 0
    fprintf('PASS: Elevator produces stabilizing pitch control. Cm_delta_e = %.3f 1/rad\n', Cm_delta_e);
else
    fprintf('CHECK: Elevator sign convention may be reversed. Cm_delta_e = %.3f 1/rad\n', Cm_delta_e);
end

%% Lateral stability
% Requirement: Cl_beta < 0 for positive dihedral effect

if Cl_beta < 0
    fprintf('PASS: Lateral static stability. Cl_beta = %.3f 1/rad\n', Cl_beta);
else
    fprintf('FAIL: Lateral static stability is poor. Cl_beta = %.3f 1/rad\n', Cl_beta);
end

%% Roll damping
% Requirement: Cl_p < 0

if Cl_p < 0
    fprintf('PASS: Roll damping is stable. Cl_p = %.3f\n', Cl_p);
else
    fprintf('FAIL: Roll damping is unstable. Cl_p = %.3f\n', Cl_p);
end

%% Aileron control authority
% Requirement: Cl_delta_a > 0 under standard sign convention

if Cl_delta_a > 0
    fprintf('PASS: Aileron control derivative has correct sign. Cl_delta_a = %.3f 1/rad\n', Cl_delta_a);
else
    fprintf('CHECK: Aileron sign convention may be reversed. Cl_delta_a = %.3f 1/rad\n', Cl_delta_a);
end

%% Directional static stability
% Requirement: Cn_beta > 0

if Cn_beta > 0
    fprintf('PASS: Directional static stability. Cn_beta = %.3f 1/rad\n', Cn_beta);
else
    fprintf('FAIL: Directional static stability is poor. Cn_beta = %.3f 1/rad\n', Cn_beta);
end

%% Yaw damping
% Requirement: Cn_r < 0

if Cn_r < 0
    fprintf('PASS: Yaw damping is stable. Cn_r = %.3f\n', Cn_r);
else
    fprintf('FAIL: Yaw damping is unstable. Cn_r = %.3f\n', Cn_r);
end

%% Rudder control authority
% Requirement: Cn_delta_r > 0 under standard sign convention

if Cn_delta_r > 0
    fprintf('PASS: Rudder control derivative has correct sign. Cn_delta_r = %.3f 1/rad\n', Cn_delta_r);
else
    fprintf('CHECK: Rudder sign convention may be reversed. Cn_delta_r = %.3f 1/rad\n', Cn_delta_r);
end

%% Overall stability result

passed_all = ...
    SM > 0 && ...
    Cm_alpha < 0 && ...
    Cm_q < 0 && ...
    Cl_beta < 0 && ...
    Cl_p < 0 && ...
    Cn_beta > 0 && ...
    Cn_r < 0;

fprintf('\nOVERALL STABILITY RESULT:\n');

if passed_all
    fprintf('PASS: Aircraft passes preliminary static and damping stability checks.\n');
else
    fprintf('FAIL: Aircraft does not pass all preliminary stability checks.\n');
end