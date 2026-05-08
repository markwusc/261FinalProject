%% Stability and Control Tail Sizing
% Objective:
% Size the horizontal and vertical stabilizers using the tail volume
% coefficient method.
%
% Given:
% Wing area, S = 36.8 m^2
% Wing span, b = 15.2 m
% Aircraft CG is approximately 4 m from the nose
%
% Method:
% Horizontal Tail Volume Coefficient:
%   C_HT = (S_HT * L_HT) / (S_wing * c_bar)
%
% Vertical Tail Volume Coefficient:
%   C_VT = (S_VT * L_VT) / (S_wing * b)
%
% Rearranged:
%   S_HT = (C_HT * S_wing * c_bar) / L_HT
%   S_VT = (C_VT * S_wing * b) / L_VT
%
% Assumptions:
% 1. Aircraft has a conventional tail.
% 2. Tail aerodynamic centers are assumed to be at the quarter-chord
%    points of the horizontal and vertical stabilizers.
% 3. Wing aerodynamic center is assumed to be at the quarter-chord of
%    the mean aerodynamic chord.
% 4. Tail moment arms are measured from aircraft CG to tail aerodynamic
%    centers.
% 5. Tail volume coefficients are initial sizing estimates and should be
%    refined later with static margin, trim, elevator/rudder authority,
%    and dynamic stability checks.
% 6. All dimensions are in SI units.

clc; clear; close all

%% Given Wing Geometry

S_wing = 37.3;      % Wing reference area, m^2
b_wing = 15.9;      % Wing span, m

AR_wing = b_wing^2 / S_wing;     % Wing aspect ratio
c_bar = S_wing / b_wing;         % Approximate mean aerodynamic chord, m

fprintf('Wing Geometry:\n')
fprintf('Wing area, S = %.2f m^2\n', S_wing)
fprintf('Wing span, b = %.2f m\n', b_wing)
fprintf('Wing aspect ratio, AR = %.2f\n', AR_wing)
fprintf('Mean aerodynamic chord, c_bar = %.2f m\n\n', c_bar)

%% Longitudinal Locations

x_nose = 0;             % Nose reference location, m
x_CG = 4.0;             % CG location from nose, m

% Assumed aircraft geometry
% Change these once CAD or layout dimensions are known
x_wing_AC = 4.3;        % Wing aerodynamic center from nose, m
x_HT_AC = 9.5;          % Horizontal tail aerodynamic center from nose, m
x_VT_AC = 9.3;          % Vertical tail aerodynamic center from nose, m

% Moment arms from CG to aerodynamic centers
l_wing_to_CG = x_CG - x_wing_AC;     % Positive if CG is aft of wing AC
L_HT = x_HT_AC - x_CG;               % Horizontal tail moment arm, m
L_VT = x_VT_AC - x_CG;               % Vertical tail moment arm, m

fprintf('Longitudinal Locations:\n')
fprintf('CG location from nose = %.2f m\n', x_CG)
fprintf('Wing AC location from nose = %.2f m\n', x_wing_AC)
fprintf('Horizontal tail AC location from nose = %.2f m\n', x_HT_AC)
fprintf('Vertical tail AC location from nose = %.2f m\n', x_VT_AC)
fprintf('Wing AC to CG distance = %.2f m\n', l_wing_to_CG)
fprintf('Horizontal tail moment arm, L_HT = %.2f m\n', L_HT)
fprintf('Vertical tail moment arm, L_VT = %.2f m\n\n', L_VT)

%% Tail Volume Coefficients

% Typical initial values from conceptual aircraft design methods.
% These should be changed depending on aircraft type.
%
% C_HT typical range:
%   Sailplane: ~0.5
%   General aviation: ~0.7
%   Jet transport: ~1.0
%
% C_VT typical range:
%   General aviation: ~0.04
%   Jet transport: ~0.08
%   Fighter: ~0.09

C_HT = 0.70;        % Horizontal tail volume coefficient
C_VT = 0.04;        % Vertical tail volume coefficient

fprintf('Tail Volume Coefficients:\n')
fprintf('Horizontal tail volume coefficient, C_HT = %.3f\n', C_HT)
fprintf('Vertical tail volume coefficient, C_VT = %.3f\n\n', C_VT)

%% Tail Area Sizing

S_HT = (C_HT * S_wing * c_bar) / L_HT;       % Horizontal tail area, m^2
S_VT = (C_VT * S_wing * b_wing) / L_VT;      % Vertical tail area, m^2

fprintf('Initial Tail Sizing Results:\n')
fprintf('Horizontal stabilizer area, S_HT = %.2f m^2\n', S_HT)
fprintf('Vertical stabilizer area, S_VT = %.2f m^2\n\n', S_VT)

fprintf('Area Ratios:\n')
fprintf('S_HT / S_wing = %.3f\n', S_HT / S_wing)
fprintf('S_VT / S_wing = %.3f\n\n', S_VT / S_wing)

%% Tail Planform Assumptions

% Horizontal stabilizer assumptions
AR_HT = 4.0;           % Horizontal tail aspect ratio
lambda_HT = 0.45;      % Horizontal tail taper ratio
sweep_HT_deg = 5;      % Horizontal tail quarter-chord sweep, deg

% Vertical stabilizer assumptions
AR_VT = 1.6;           % Vertical tail aspect ratio
lambda_VT = 0.50;      % Vertical tail taper ratio
sweep_VT_deg = 25;     % Vertical tail quarter-chord sweep, deg

%% Horizontal Tail Planform

b_HT = sqrt(AR_HT * S_HT);                         % Horizontal tail span, m
c_root_HT = 2 * S_HT / (b_HT * (1 + lambda_HT));   % Root chord, m
c_tip_HT = lambda_HT * c_root_HT;                  % Tip chord, m
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

% For vertical tail:
% AR_VT = h_VT^2 / S_VT
% where h_VT is vertical tail height.

h_VT = sqrt(AR_VT * S_VT);                         % Vertical tail height, m
c_root_VT = 2 * S_VT / (h_VT * (1 + lambda_VT));   % Root chord, m
c_tip_VT = lambda_VT * c_root_VT;                  % Tip chord, m
MAC_VT = (2/3) * c_root_VT * ...
    ((1 + lambda_VT + lambda_VT^2) / (1 + lambda_VT));

fprintf('Vertical Tail Planform:\n')
fprintf('Vertical tail aspect ratio, AR_VT = %.2f\n', AR_VT)
fprintf('Vertical tail height, h_VT = %.2f m\n', h_VT)
fprintf('Vertical tail root chord = %.2f m\n', c_root_VT)
fprintf('Vertical tail tip chord = %.2f m\n', c_tip_VT)
fprintf('Vertical tail MAC = %.2f m\n', MAC_VT)
fprintf('Vertical tail sweep = %.2f deg\n\n', sweep_VT_deg)

%% Static Margin Estimate

% This is a simple placeholder estimate.
% A real static margin calculation requires lift curve slopes,
% downwash gradient, tail efficiency, and exact CG/AC locations.

static_margin_target = 0.10;       % 10 percent MAC, typical starting target
SM_m = static_margin_target * c_bar;

x_NP_target = x_CG + SM_m;         % Desired neutral point location

fprintf('Simple Static Margin Target:\n')
fprintf('Target static margin = %.1f percent MAC\n', static_margin_target * 100)
fprintf('Target static margin distance = %.2f m\n', SM_m)
fprintf('Approximate target neutral point location = %.2f m from nose\n\n', x_NP_target)

%% Elevator and Rudder Initial Sizing

% Typical initial control surface ratios.
% These should be refined using trim and control authority analysis.

elevator_chord_ratio = 0.30;       % Elevator chord / horizontal tail chord
rudder_chord_ratio = 0.35;         % Rudder chord / vertical tail chord

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

figure
plot(L_HT_range, S_HT_range, 'LineWidth', 2)
grid on
xlabel('Horizontal Tail Moment Arm, L_{HT} (m)')
ylabel('Horizontal Stabilizer Area, S_{HT} (m^2)')
title('Horizontal Stabilizer Area vs Tail Moment Arm')

figure
plot(L_VT_range, S_VT_range, 'LineWidth', 2)
grid on
xlabel('Vertical Tail Moment Arm, L_{VT} (m)')
ylabel('Vertical Stabilizer Area, S_{VT} (m^2)')
title('Vertical Stabilizer Area vs Tail Moment Arm')

%% Simple Top-View Tail Layout Plot

figure
hold on
grid on
axis equal

% Fuselage centerline
plot([0, 10.5], [0, 0], 'k--', 'LineWidth', 1.5)

% CG
plot(x_CG, 0, 'ro', 'MarkerFaceColor', 'r')
text(x_CG, 0.25, 'CG')

% Wing AC
plot(x_wing_AC, 0, 'bo', 'MarkerFaceColor', 'b')
text(x_wing_AC, -0.35, 'Wing AC')

% Horizontal tail AC
plot(x_HT_AC, 0, 'go', 'MarkerFaceColor', 'g')
text(x_HT_AC, 0.25, 'HT AC')

% Approximate wing
wing_y = [-b_wing/2, b_wing/2];
plot([x_wing_AC, x_wing_AC], wing_y, 'b', 'LineWidth', 3)

% Approximate horizontal tail
HT_y = [-b_HT/2, b_HT/2];
plot([x_HT_AC, x_HT_AC], HT_y, 'g', 'LineWidth', 3)

xlabel('x-location from nose (m)')
ylabel('Spanwise location, y (m)')
title('Simple Top-View Layout: Wing, CG, and Horizontal Tail')
legend('Fuselage centerline', 'CG', 'Wing AC', 'Horizontal Tail AC', ...
    'Wing span', 'Horizontal tail span', 'Location', 'best')

%% Summary

fprintf('SUMMARY:\n')
fprintf('Using S = %.2f m^2 and b = %.2f m:\n', S_wing, b_wing)
fprintf('Required horizontal stabilizer area = %.2f m^2\n', S_HT)
fprintf('Required vertical stabilizer area = %.2f m^2\n', S_VT)
fprintf('Horizontal tail span estimate = %.2f m\n', b_HT)
fprintf('Vertical tail height estimate = %.2f m\n', h_VT)
fprintf('Distance from wing AC to CG = %.2f m\n', l_wing_to_CG)
fprintf('Distance from CG to horizontal tail AC = %.2f m\n', L_HT)
fprintf('Distance from CG to vertical tail AC = %.2f m\n', L_VT)