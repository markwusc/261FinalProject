%% Payload_Range_Diagram.m
% =========================================================================
% Payload-range diagram for Arctic STOL aircraft
%
% Values based on current report payload-range section:
%   MTOW = 4077 kg
%   OEW  = 2150 kg
%   Useful load = 1927 kg
%   Max payload = 1399 kg
%   Design fuel = 535 kg
%   Design range = 450 nmi
%   Ferry range ≈ 850 nmi
%
% The diagram includes:
%   A: Maximum payload, zero range
%   B: Design mission point, max payload at 450 nmi
%   D: Ferry range, zero payload
% =========================================================================

clear; clc; close all;

%% Aircraft weight data from report

MTOW = 4077;              % maximum takeoff weight [kg]
OEW  = 2150;              % operating empty weight [kg]
W_payload_max = 1399;     % maximum design payload [kg]
W_fuel_design = 535;      % design mission fuel [kg]

%% Mission/range data

R_design = 450;           % required/design range [nmi]
R_ferry  = 850;           % estimated ferry range at zero payload [nmi]

%% Useful load check

useful_load = MTOW - OEW;                 % payload + fuel [kg]
fuel_at_max_payload = useful_load - W_payload_max;

fprintf('Useful load = %.1f kg\n', useful_load)
fprintf('Fuel available at max payload = %.1f kg\n', fuel_at_max_payload)
fprintf('Design fuel = %.1f kg\n\n', W_fuel_design)

if abs(useful_load - 1927) > 1e-6
    warning('Useful load does not match the report value of 1927 kg.')
end

if abs(fuel_at_max_payload - W_fuel_design) > 10
    warning(['Fuel available at max payload differs from design fuel by more than 10 kg. ', ...
             'Check MTOW, OEW, payload, and fuel consistency.'])
end

%% Payload-range points

% Point A: maximum payload, zero range
R_A = 0;
P_A = W_payload_max;

% Point B: design mission point
R_B = R_design;
P_B = W_payload_max;

% Point D: ferry range, zero payload
R_D = R_ferry;
P_D = 0;

% Tradeoff region from design point to ferry point
R_trade = linspace(R_B, R_D, 150);
P_trade = linspace(P_B, P_D, 150);

%% Plot payload-range diagram

figure('Name','Payload-Range Diagram', ...
       'Units','normalized','Position',[0.12 0.15 0.72 0.55]);

hold on; grid on; box on;

% Main payload-range boundary
h1 = plot([R_A R_B], [P_A P_B], 'k-', 'LineWidth', 2.4);
h2 = plot(R_trade, P_trade, 'k:', 'LineWidth', 2.4);

% Key points
h3 = plot(R_B, P_B, 'ro', ...
          'MarkerFaceColor','r', ...
          'MarkerSize',8);

h4 = plot(R_D, P_D, 'bo', ...
          'MarkerFaceColor','b', ...
          'MarkerSize',8);

% Requirement and reference lines
xline(R_design, '--', '450 nmi Required Range', ...
      'LineWidth', 1.4, ...
      'LabelVerticalAlignment','bottom');

yline(W_payload_max, '--', 'Max Payload', ...
      'LineWidth', 1.2, ...
      'LabelHorizontalAlignment','left');

xlabel('Range, $R$ [nmi]','Interpreter','latex')
ylabel('Payload, $W_{payload}$ [kg]','Interpreter','latex')
title('Payload-Range Diagram')

xlim([0, 1.10*R_D])
ylim([0, 1.15*W_payload_max])

legend([h1 h2 h3 h4], ...
       'Maximum-payload range segment', ...
       'Payload-fuel tradeoff region', ...
       'Design mission point', ...
       'Ferry range point', ...
       'Location','northeast')

%% Print summary

fprintf('Payload-Range Summary:\n')
fprintf('Point A: Range = %.0f nmi, Payload = %.0f kg\n', R_A, P_A)
fprintf('Point B: Range = %.0f nmi, Payload = %.0f kg\n', R_B, P_B)
fprintf('Point D: Range = %.0f nmi, Payload = %.0f kg\n\n', R_D, P_D)

fprintf('Aircraft Values Used:\n')
fprintf('MTOW = %.0f kg\n', MTOW)
fprintf('OEW = %.0f kg\n', OEW)
fprintf('Useful load = %.0f kg\n', useful_load)
fprintf('Maximum payload = %.0f kg\n', W_payload_max)
fprintf('Fuel available at max payload = %.0f kg\n', fuel_at_max_payload)
fprintf('Design fuel = %.0f kg\n', W_fuel_design)
fprintf('Design range = %.0f nmi\n', R_design)
fprintf('Estimated ferry range = %.0f nmi\n', R_ferry)

%% Save figure

print(gcf, 'Payload_Range_Diagram_Corrected.png', '-dpng', '-r600')