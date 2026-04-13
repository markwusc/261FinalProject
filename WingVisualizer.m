% Top-Down Wing Planform Visualizer
% Input S and b in metric, but convert everything else to imperial

clc; clear; close all

%% USER INPUTS (METRIC)

S_m2 = 48;        % wing area, m^2
b_m  = 18;        % wingspan, m
lambda = 0.45;    % taper ratio = c_tip / c_root

% Optional sweep angle at quarter-chord
sweep_deg = 5;    % degrees

%% UNIT CONVERSIONS

m_to_ft = 3.28084;
m2_to_ft2 = 10.7639;

S = S_m2 * m2_to_ft2;     % wing area, ft^2
b = b_m  * m_to_ft;       % wingspan, ft

%% GEOMETRY CALCULATIONS (IMPERIAL)

semi_span = b/2;
sweep = deg2rad(sweep_deg);

% For a trapezoidal wing:
% S = b*(c_root + c_tip)/2
% c_tip = lambda*c_root
% => c_root = 2S / [b*(1+lambda)]

c_root = 2*S / (b*(1 + lambda));
c_tip  = lambda * c_root;

% Quarter-chord sweep offset from root to tip
x_qc_shift = semi_span * tan(sweep);

% Leading edge positions
xLE_root = 0;
xTE_root = c_root;

xQC_root = xLE_root + 0.25*c_root;
xQC_tip  = xQC_root + x_qc_shift;

xLE_tip = xQC_tip - 0.25*c_tip;
xTE_tip = xLE_tip + c_tip;

%% PLANFORM CORNERS

% Right half-wing polygon
x_right = [xLE_root, xTE_root, xTE_tip, xLE_tip];
y_right = [0,        0,        semi_span, semi_span];

% Left half-wing polygon
x_left = [xLE_root, xTE_root, xTE_tip, xLE_tip];
y_left = [0,        0,       -semi_span, -semi_span];

%% PLOT

figure('Color','w');
hold on; grid on; axis equal

fill(x_right, y_right, [0.7 0.85 1.0], 'EdgeColor', 'k', 'LineWidth', 1.5)
fill(x_left,  y_left,  [0.7 0.85 1.0], 'EdgeColor', 'k', 'LineWidth', 1.5)

% Draw centerline
plot([0 0], [-semi_span semi_span], 'k--', 'LineWidth', 1)

% Draw quarter-chord line
plot([xQC_root xQC_tip], [0 semi_span], 'r-', 'LineWidth', 2)
plot([xQC_root xQC_tip], [0 -semi_span], 'r-', 'LineWidth', 2)

xlabel('x (ft)')
ylabel('y (ft)')
title('Top-Down Wing Planform View')

%% ANNOTATION

text(c_root*0.1, 1.5, sprintf('c_{root} = %.2f ft', c_root), 'FontSize', 11)
text(xLE_tip + 0.3, semi_span - 1.5, sprintf('c_{tip} = %.2f ft', c_tip), 'FontSize', 11)
text(c_root*0.1, -semi_span - 2.5, sprintf('S = %.2f ft^2,  b = %.2f ft,  AR = %.2f', S, b, b^2/S), 'FontSize', 11)
text(c_root*0.1, -semi_span - 4.5, sprintf('\\lambda = %.2f,  sweep = %.1f^\\circ', lambda, sweep_deg), 'FontSize', 11)

xlim([min([x_left x_right]) - 3, max([x_left x_right]) + 3])
ylim([-semi_span - 6, semi_span + 6])

%% PRINT RESULTS

fprintf('\n================ WING PLANFORM GEOMETRY ================\n');
fprintf('Input wing area, S         = %.3f m^2\n', S_m2);
fprintf('Input wingspan, b          = %.3f m\n', b_m);
fprintf('Wing area, S              = %.3f ft^2\n', S);
fprintf('Wingspan, b               = %.3f ft\n', b);
fprintf('Aspect ratio, AR          = %.3f\n', b^2/S);
fprintf('Taper ratio, lambda       = %.3f\n', lambda);
fprintf('Sweep angle               = %.3f deg\n', sweep_deg);
fprintf('Root chord, c_root        = %.3f ft\n', c_root);
fprintf('Tip chord, c_tip          = %.3f ft\n', c_tip);
fprintf('========================================================\n\n');