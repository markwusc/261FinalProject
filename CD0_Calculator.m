% Parasite Drag Build-Up Using Geometry Extracted From PLANE.step
%
% Geometry was extracted from the STEP file outside of MATLAB.
% MATLAB does NOT need to read the CAD file in this script.
%
% Coordinate interpretation from CAD:
% X = wingspan direction
% Y = aircraft length direction
% Z = vertical direction
%
% Drag build-up method:
% D0 = q_inf * A_total
% CD0 = A_total / S_ref
%
% A_total = A_fuse + A_wing + A_htail + A_vtail + A_aux

clc; clear; close all

%% GIVEN VALUES

S_ref = 44.9;              % wing reference area, m^2
V_cruise = 69;             % cruise speed, m/s
AR_given = 7.2;            % given aspect ratio
airfoil = "NASA/Langley LS(1)-0417 (GA(W)-1)";

%% CAD-EXTRACTED GEOMETRY

% From PLANE.step point data
b_wing = 18.000;           % m, full wingspan from CAD
L_total = 13.699;          % m, full aircraft length including tail
L_fuse = 7.276;            % m, central fuselage/body length estimate

c_root = 3.197;            % m, approximate root chord from CAD
c_tip = 1.928;             % m, approximate tip chord from CAD
lambda_wing = c_tip / c_root;

% Horizontal tail extracted from CAD
b_htail = 4.877;           % m
c_htail_root = 1.817;      % m
c_htail_tip = 0.991;       % m
lambda_htail = c_htail_tip / c_htail_root;

% Vertical tail estimate from CAD side projection
S_vtail = 1.20;            % m^2, approximate exposed vertical tail area

%% ASSUMPTIONS

rho = 1.225;               % kg/m^3, sea-level density
mu = 1.789e-5;             % Pa*s, sea-level dynamic viscosity

l_over_d_fuse = 2.89;      % fuselage fineness ratio, from your input
d_fuse = L_fuse / l_over_d_fuse;

tc_wing = 0.17;            % LS(1)-0417 is approximately 17 percent thick
tc_tail = 0.12;            % assumed tail airfoil thickness ratio

aux_percent = 0.15;        % auxiliary drag allowance for struts, gaps, gear, antennas, etc.

%% DYNAMIC PRESSURE

q_inf = 0.5 * rho * V_cruise^2;

%% WING GEOMETRY CHECK

S_wing_CAD_trap = b_wing * (c_root + c_tip) / 2;
AR_from_CAD = b_wing^2 / S_ref;

MAC_wing = (2/3) * c_root * ...
    ((1 + lambda_wing + lambda_wing^2) / (1 + lambda_wing));

%% FUSELAGE PARASITE DRAG AREA

Re_fuse = rho * V_cruise * L_fuse / mu;
Cf_fuse = skinFrictionCf(Re_fuse);

S_wet_fuse = (pi*d_fuse/2) * (d_fuse + L_fuse);

FF_fuse = 1 + 1.5*(l_over_d_fuse)^(-1.5) + 7*(l_over_d_fuse)^(-3);

A_fuse = FF_fuse * Cf_fuse * S_wet_fuse;

%% WING PARASITE DRAG AREA

Re_wing = rho * V_cruise * MAC_wing / mu;
Cf_wing = skinFrictionCf(Re_wing);

FF_wing = 1 + 2*tc_wing + 60*tc_wing^4;

A_wing = 2 * Cf_wing * FF_wing * S_ref;

%% HORIZONTAL TAIL PARASITE DRAG AREA

S_htail = b_htail * (c_htail_root + c_htail_tip) / 2;

MAC_htail = (2/3) * c_htail_root * ...
    ((1 + lambda_htail + lambda_htail^2) / (1 + lambda_htail));

Re_htail = rho * V_cruise * MAC_htail / mu;
Cf_htail = skinFrictionCf(Re_htail);

FF_htail = 1 + 2*tc_tail + 60*tc_tail^4;

A_htail = 2 * Cf_htail * FF_htail * S_htail;

%% VERTICAL TAIL PARASITE DRAG AREA

% Use an approximate vertical tail MAC of 1.0 m based on CAD side projection
MAC_vtail = 1.0;

Re_vtail = rho * V_cruise * MAC_vtail / mu;
Cf_vtail = skinFrictionCf(Re_vtail);

FF_vtail = 1 + 2*tc_tail + 60*tc_tail^4;

A_vtail = 2 * Cf_vtail * FF_vtail * S_vtail;

%% AUXILIARY DRAG AREA

A_clean = A_fuse + A_wing + A_htail + A_vtail;
A_aux = aux_percent * A_clean;

A_total = A_clean + A_aux;

%% FINAL PARASITE DRAG

CD0 = A_total / S_ref;
D0 = q_inf * A_total;

%% PRINT RESULTS

fprintf('Parasite Drag Build-Up Using CAD-Extracted Geometry\n')
fprintf('--------------------------------------------------\n\n')

fprintf('Airfoil: %s\n', airfoil)
fprintf('Cruise speed = %.2f m/s\n', V_cruise)
fprintf('Dynamic pressure q_inf = %.2f Pa\n\n', q_inf)

fprintf('CAD-Extracted Main Geometry:\n')
fprintf('Wing span b = %.3f m\n', b_wing)
fprintf('Full aircraft length = %.3f m\n', L_total)
fprintf('Estimated fuselage/body length = %.3f m\n', L_fuse)
fprintf('Wing root chord = %.3f m\n', c_root)
fprintf('Wing tip chord = %.3f m\n', c_tip)
fprintf('Wing taper ratio = %.3f\n', lambda_wing)
fprintf('Wing MAC = %.3f m\n', MAC_wing)
fprintf('Wing area from CAD trapezoid = %.3f m^2\n', S_wing_CAD_trap)
fprintf('Reference wing area used = %.3f m^2\n', S_ref)
fprintf('AR from CAD span and S_ref = %.3f\n\n', AR_from_CAD)

fprintf('Fuselage Estimate:\n')
fprintf('l/d used = %.3f\n', l_over_d_fuse)
fprintf('Fuselage diameter estimate = %.3f m\n', d_fuse)
fprintf('Fuselage wetted area = %.3f m^2\n', S_wet_fuse)
fprintf('Re_fuse = %.3e\n', Re_fuse)
fprintf('Cf_fuse = %.5f\n', Cf_fuse)
fprintf('FF_fuse = %.4f\n', FF_fuse)
fprintf('A_fuse = %.4f m^2\n\n', A_fuse)

fprintf('Wing Drag:\n')
fprintf('Re_wing = %.3e\n', Re_wing)
fprintf('Cf_wing = %.5f\n', Cf_wing)
fprintf('FF_wing = %.4f\n', FF_wing)
fprintf('A_wing = %.4f m^2\n\n', A_wing)

fprintf('Tail Drag:\n')
fprintf('Horizontal tail area = %.3f m^2\n', S_htail)
fprintf('Horizontal tail MAC = %.3f m\n', MAC_htail)
fprintf('A_htail = %.4f m^2\n', A_htail)
fprintf('Vertical tail area estimate = %.3f m^2\n', S_vtail)
fprintf('A_vtail = %.4f m^2\n\n', A_vtail)

fprintf('Equivalent Parasite Drag Areas:\n')
fprintf('A_fuse  = %.4f m^2\n', A_fuse)
fprintf('A_wing  = %.4f m^2\n', A_wing)
fprintf('A_htail = %.4f m^2\n', A_htail)
fprintf('A_vtail = %.4f m^2\n', A_vtail)
fprintf('A_aux   = %.4f m^2\n', A_aux)
fprintf('A_total = %.4f m^2\n\n', A_total)

fprintf('Final Results:\n')
fprintf('CD0 = %.5f\n', CD0)
fprintf('Parasite drag D0 = %.2f N\n', D0)
fprintf('Parasite drag D0 = %.2f lbf\n', D0 * 0.224809)

%% LOCAL FUNCTION

function Cf = skinFrictionCf(ReL)
    if ReL <= 5e5
        Cf = 1.33 / sqrt(ReL);
    else
        Cf = 0.074 / ReL^(1/5);
    end
end