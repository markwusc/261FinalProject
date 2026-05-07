%% Payload_Range_Diagram.m
% =========================================================================
% Payload-range diagram for Arctic STOL aircraft
%
% Uses preliminary design values:
%   MTOW = 4150 kg
%   OEW  = 2172 kg
%   Fuel = 548 kg
%   Max payload = 1429 kg
%   Design range = 450 nmi
%
% The diagram includes:
%   A: Maximum payload, no extra range fuel
%   B: Maximum payload at design fuel/range
%   C: Reduced payload, maximum fuel-limited range
%   D: Ferry range, zero payload
% =========================================================================

clear; clc; close all;

%% Aircraft weight data

MTOW = 4150;          % maximum takeoff weight [kg]
OEW  = 2172;          % operating empty weight [kg]
W_fuel_design = 548;  % design fuel weight [kg]
W_payload_max = 1429; % maximum payload [kg]

%% Mission/range data

R_design = 450;       % required/design range [nmi]

% Approximate fuel-specific range based on design mission
% This assumes range scales approximately linearly with usable fuel
% for a first-order payload-range plot.
nmi_per_kg_fuel = R_design / W_fuel_design;

%% Useful load checks

useful_load = MTOW - OEW;                 % payload + fuel available [kg]
fuel_at_max_payload = MTOW - OEW - W_payload_max;

fprintf('Useful load = %.1f kg\n', useful_load)
fprintf('Fuel available at max payload = %.1f kg\n', fuel_at_max_payload)

if fuel_at_max_payload < W_fuel_design
    warning('Max payload plus design fuel exceeds MTOW. Check weight assumptions.')
end

%% Payload-range points

% Point A: zero-range max-payload point
R_A = 0;
P_A = W_payload_max;

% Point B: design mission point
R_B = R_design;
P_B = W_payload_max;

% Point C: maximum fuel with payload reduced to remain at MTOW
% For this simple model, assume fuel can increase until payload reaches zero.
% If max fuel tank capacity is known, replace W_fuel_max with that value.
W_fuel_max = useful_load;     % all useful load used as fuel at ferry condition

% Range at maximum possible fuel
R_D = W_fuel_max * nmi_per_kg_fuel;
P_D = 0;

% Point C can be taken as the start of payload tradeoff region.
% Since design fuel is already the selected full mission fuel, C = B unless
% a larger tank capacity is specified. To show the tradeoff, define C as the
% point where fuel begins increasing above design fuel.
R_C = R_B;
P_C = P_B;

% Generate sloped tradeoff line from max-payload design point to ferry point
R_trade = linspace(R_C, R_D, 100);
W_fuel_trade = R_trade / nmi_per_kg_fuel;
P_trade = MTOW - OEW - W_fuel_trade;

% Limit payload to physically valid values
P_trade(P_trade < 0) = 0;

%% Plot payload-range diagram

figure('Name','Payload-Range Diagram', ...
       'Units','normalized','Position',[0.12 0.15 0.72 0.55]);

hold on; grid on; box on;

% Main payload-range boundary
plot([R_A R_B], [P_A P_B], 'k-', 'LineWidth', 2.4)
plot(R_trade, P_trade, 'k-', 'LineWidth', 2.4)

% Design point
plot(R_B, P_B, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8)

% Ferry point
plot(R_D, P_D, 'bo', 'MarkerFaceColor','b', 'MarkerSize',8)

% Required range line
xline(R_design, '--', '450 nmi Required Range', ...
    'LineWidth', 1.4, ...
    'LabelVerticalAlignment','bottom');

% Payload reference line
yline(W_payload_max, '--', 'Max Payload', ...
    'LineWidth', 1.2, ...
    'LabelHorizontalAlignment','left');



xlabel('Range [nmi]')
ylabel('Payload [kg]')
title('Payload-Range Diagram')

xlim([0, 1.10*R_D])
ylim([0, 1.15*W_payload_max])

legend('Payload-range boundary', ...
       'Payload tradeoff region', ...
       'Design mission point', ...
       'Ferry range point', ...
       'Location','northeast')

%% Print summary

fprintf('\nPayload-Range Summary:\n')
fprintf('Point A: Range = %.0f nmi, Payload = %.0f kg\n', R_A, P_A)
fprintf('Point B: Range = %.0f nmi, Payload = %.0f kg\n', R_B, P_B)
fprintf('Point D: Range = %.0f nmi, Payload = %.0f kg\n', R_D, P_D)

fprintf('\nAssumed fuel-specific range = %.3f nmi/kg fuel\n', nmi_per_kg_fuel)
fprintf('Estimated ferry range = %.0f nmi\n', R_D)

%% Save figure

print(gcf, 'Payload_Range_Diagram.png', '-dpng', '-r600')