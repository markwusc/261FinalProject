%% Stability and Control Tail Sizing with Preliminary CG Envelope
% =========================================================================
% Objective:
% Size the horizontal and vertical stabilizers using the tail volume
% coefficient method and define a preliminary CG envelope.
%
% This script also defines the CG reference system used by the separate
% CG envelope script.
% =========================================================================

clc; clear; close all

%% Given Wing Geometry

S_wing = 36.8;      % Wing reference area [m^2]
b_wing = 15.2;      % Wing span [m]

AR_wing = b_wing^2 / S_wing;     % Wing aspect ratio [-]
c_bar = S_wing / b_wing;         % Approximate mean aerodynamic chord [m]

fprintf('Wing Geometry:\n')
fprintf('Wing area, S = %.2f m^2\n', S_wing)
fprintf('Wing span, b = %.2f m\n', b_wing)
fprintf('Wing aspect ratio, AR = %.2f\n', AR_wing)
fprintf('Mean aerodynamic chord, c_bar = %.2f m\n\n', c_bar)

%% Longitudinal Locations

x_nose = 0;             % Nose reference location [m]

% Baseline aircraft CG and aerodynamic center locations
x_CG_baseline = 4.00;   % Baseline CG location from nose [m]
x_wing_AC = 4.30;       % Wing aerodynamic center from nose [m]
x_HT_AC = 9.50;         % Horizontal tail aerodynamic center from nose [m]
x_VT_AC = 9.30;         % Vertical tail aerodynamic center from nose [m]

% Leading edge of MAC estimate
% Assumption: wing aerodynamic center is at 25% MAC
x_LEMAC = x_wing_AC - 0.25*c_bar;

% Moment arms
l_wing_to_CG = x_CG_baseline - x_wing_AC;
L_HT = x_HT_AC - x_CG_baseline;
L_VT = x_VT_AC - x_CG_baseline;

fprintf('Longitudinal Locations:\n')
fprintf('Baseline CG location from nose = %.2f m\n', x_CG_baseline)
fprintf('Wing AC location from nose = %.2f m\n', x_wing_AC)
fprintf('Estimated LEMAC location from nose = %.2f m\n', x_LEMAC)
fprintf('Horizontal tail AC location from nose = %.2f m\n', x_HT_AC)
fprintf('Vertical tail AC location from nose = %.2f m\n', x_VT_AC)
fprintf('Wing AC to CG distance = %.2f m\n', l_wing_to_CG)
fprintf('Horizontal tail moment arm, L_HT = %.2f m\n', L_HT)
fprintf('Vertical tail moment arm, L_VT = %.2f m\n\n', L_VT)

%% Preliminary CG Envelope

% The neutral point target is based on preserving approximately 10% static
% margin at the baseline CG location.
static_margin_target = 0.10;       % 10% MAC
SM_m = static_margin_target * c_bar;
x_NP_target = x_CG_baseline + SM_m;

% Preliminary allowable CG limits
% Forward limit selected to avoid excessive nose-heavy trim requirement.
% Aft limit selected forward of the neutral point to preserve positive
% static margin.
CG_forward_percent_MAC = 5;        % [% MAC]
CG_aft_percent_MAC     = 20;       % [% MAC]

x_CG_forward = x_LEMAC + (CG_forward_percent_MAC/100)*c_bar;
x_CG_aft     = x_LEMAC + (CG_aft_percent_MAC/100)*c_bar;

baseline_percent_MAC = 100*(x_CG_baseline - x_LEMAC)/c_bar;
aft_limit_static_margin = 100*(x_NP_target - x_CG_aft)/c_bar;

fprintf('Preliminary CG Envelope:\n')
fprintf('Forward CG limit = %.2f m from nose = %.1f%% MAC\n', ...
    x_CG_forward, CG_forward_percent_MAC)
fprintf('Aft CG limit     = %.2f m from nose = %.1f%% MAC\n', ...
    x_CG_aft, CG_aft_percent_MAC)
fprintf('Baseline CG      = %.2f m from nose = %.1f%% MAC\n', ...
    x_CG_baseline, baseline_percent_MAC)
fprintf('Target neutral point = %.2f m from nose\n', x_NP_target)
fprintf('Static margin at aft CG limit = %.1f%% MAC\n\n', aft_limit_static_margin)

%% Tail Volume Coefficients

C_HT = 0.70;        % Horizontal tail volume coefficient [-]
C_VT = 0.04;        % Vertical tail volume coefficient [-]

fprintf('Tail Volume Coefficients:\n')
fprintf('Horizontal tail volume coefficient, C_HT = %.3f\n', C_HT)
fprintf('Vertical tail volume coefficient, C_VT = %.3f\n\n', C_VT)

%% Tail Area Sizing

S_HT = (C_HT * S_wing * c_bar) / L_HT;       % Horizontal tail area [m^2]
S_VT = (C_VT * S_wing * b_wing) / L_VT;      % Vertical tail area [m^2]

fprintf('Initial Tail Sizing Results:\n')
fprintf('Horizontal stabilizer area, S_HT = %.2f m^2\n', S_HT)
fprintf('Vertical stabilizer area, S_VT = %.2f m^2\n\n', S_VT)

fprintf('Area Ratios:\n')
fprintf('S_HT / S_wing = %.3f\n', S_HT / S_wing)
fprintf('S_VT / S_wing = %.3f\n\n', S_VT / S_wing)

%% Tail Planform Assumptions

AR_HT = 4.0;           % Horizontal tail aspect ratio [-]
lambda_HT = 0.45;      % Horizontal tail taper ratio [-]
sweep_HT_deg = 5;      % Horizontal tail quarter-chord sweep [deg]

AR_VT = 1.6;           % Vertical tail aspect ratio [-]
lambda_VT = 0.50;      % Vertical tail taper ratio [-]
sweep_VT_deg = 25;     % Vertical tail quarter-chord sweep [deg]

%% Horizontal Tail Planform

b_HT = sqrt(AR_HT * S_HT);
c_root_HT = 2 * S_HT / (b_HT * (1 + lambda_HT));
c_tip_HT = lambda_HT * c_root_HT;
MAC_HT = (2/3) * c_root_HT * ...
    ((1 + lambda_HT + lambda_HT^2) / (1 + lambda_HT));

fprintf('Horizontal Tail Planform:\n')
fprintf('Horizontal tail aspect ratio, AR_HT = %.2f\n', AR_HT)
fprintf('Horizontal tail span, b_HT = %.2f m\n', b_HT)
fprintf('Horizontal tail root chord = %.2f m\n', c_root_HT)
fprintf('Horizontal tail tip chord = %.2f m\n', c_tip_HT)
fprintf('Horizontal tail MAC = %.2f m\n', MAC_HT)
fprintf('Horizontal tail sweep = %.2f deg\n\n', sweep_HT_deg)

%% Vertical Tail Planform

h_VT = sqrt(AR_VT * S_VT);
c_root_VT = 2 * S_VT / (h_VT * (1 + lambda_VT));
c_tip_VT = lambda_VT * c_root_VT;
MAC_VT = (2/3) * c_root_VT * ...
    ((1 + lambda_VT + lambda_VT^2) / (1 + lambda_VT));

fprintf('Vertical Tail Planform:\n')
fprintf('Vertical tail aspect ratio, AR_VT = %.2f\n', AR_VT)
fprintf('Vertical tail height, h_VT = %.2f m\n', h_VT)
fprintf('Vertical tail root chord = %.2f m\n', c_root_VT)
fprintf('Vertical tail tip chord = %.2f m\n', c_tip_VT)
fprintf('Vertical tail MAC = %.2f m\n', MAC_VT)
fprintf('Vertical tail sweep = %.2f deg\n\n', sweep_VT_deg)

%% Elevator and Rudder Initial Sizing

elevator_chord_ratio = 0.30;
rudder_chord_ratio = 0.35;

S_elevator = elevator_chord_ratio * S_HT;
S_rudder = rudder_chord_ratio * S_VT;

fprintf('Initial Control Surface Sizing:\n')
fprintf('Elevator area estimate = %.2f m^2\n', S_elevator)
fprintf('Rudder area estimate = %.2f m^2\n\n', S_rudder)

%% Plot Tail Size Sensitivity to Tail Arm

L_HT_range = linspace(3.0, 8.0, 100);
L_VT_range = linspace(3.0, 8.0, 100);

S_HT_range = (C_HT * S_wing * c_bar) ./ L_HT_range;
S_VT_range = (C_VT * S_wing * b_wing) ./ L_VT_range;

figure('Name','Horizontal Tail Area Sensitivity')
plot(L_HT_range, S_HT_range, 'LineWidth', 2)
grid on
xlabel('Horizontal Tail Moment Arm, L_{HT} [m]')
ylabel('Horizontal Stabilizer Area, S_{HT} [m^2]')
title('Horizontal Stabilizer Area vs Tail Moment Arm')

figure('Name','Vertical Tail Area Sensitivity')
plot(L_VT_range, S_VT_range, 'LineWidth', 2)
grid on
xlabel('Vertical Tail Moment Arm, L_{VT} [m]')
ylabel('Vertical Stabilizer Area, S_{VT} [m^2]')
title('Vertical Stabilizer Area vs Tail Moment Arm')

%% Simple Top-View Tail Layout Plot

figure('Name','Simple Top-View Layout')
hold on
grid on
axis equal

% Fuselage centerline
plot([0, 10.5], [0, 0], 'k--', 'LineWidth', 1.5)

% CG
plot(x_CG_baseline, 0, 'ro', 'MarkerFaceColor', 'r')
text(x_CG_baseline, 0.25, 'Baseline CG')

% Forward and aft CG limits
plot(x_CG_forward, 0, 'ks', 'MarkerFaceColor', 'k')
text(x_CG_forward, -0.45, 'Fwd CG Limit')

plot(x_CG_aft, 0, 'ks', 'MarkerFaceColor', 'k')
text(x_CG_aft, -0.45, 'Aft CG Limit')

% Wing AC
plot(x_wing_AC, 0, 'bo', 'MarkerFaceColor', 'b')
text(x_wing_AC, -0.35, 'Wing AC')

% Horizontal tail AC
plot(x_HT_AC, 0, 'go', 'MarkerFaceColor', 'g')
text(x_HT_AC, 0.25, 'HT AC')

% Neutral point
plot(x_NP_target, 0, 'md', 'MarkerFaceColor', 'm')
text(x_NP_target, 0.45, 'Approx. NP')

% Approximate wing
wing_y = [-b_wing/2, b_wing/2];
plot([x_wing_AC, x_wing_AC], wing_y, 'b', 'LineWidth', 3)

% Approximate horizontal tail
HT_y = [-b_HT/2, b_HT/2];
plot([x_HT_AC, x_HT_AC], HT_y, 'g', 'LineWidth', 3)

xlabel('x-location from nose [m]')
ylabel('Spanwise location, y [m]')
title('Simple Top-View Layout: Wing, CG Limits, Neutral Point, and Tail')

legend('Fuselage centerline', ...
       'Baseline CG', ...
       'Forward/Aft CG Limits', ...
       '', ...
       'Wing AC', ...
       'Horizontal Tail AC', ...
       'Neutral Point', ...
       'Wing span', ...
       'Horizontal tail span', ...
       'Location','bestoutside')

%% Summary

fprintf('SUMMARY:\n')
fprintf('Using S = %.2f m^2 and b = %.2f m:\n', S_wing, b_wing)
fprintf('Required horizontal stabilizer area = %.2f m^2\n', S_HT)
fprintf('Required vertical stabilizer area = %.2f m^2\n', S_VT)
fprintf('Horizontal tail span estimate = %.2f m\n', b_HT)
fprintf('Vertical tail height estimate = %.2f m\n', h_VT)
fprintf('Baseline CG = %.2f m from nose = %.1f%% MAC\n', ...
    x_CG_baseline, baseline_percent_MAC)
fprintf('Allowable CG envelope = %.1f%% MAC to %.1f%% MAC\n', ...
    CG_forward_percent_MAC, CG_aft_percent_MAC)
fprintf('Distance from CG to horizontal tail AC = %.2f m\n', L_HT)
fprintf('Distance from CG to vertical tail AC = %.2f m\n', L_VT)