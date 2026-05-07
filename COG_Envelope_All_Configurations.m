%% COG_Envelope_All_Configurations.m
% =========================================================================
% Preliminary CG envelope check for all aircraft mission configurations.
%
% Configurations:
%   1. Passenger transport
%   2. Cargo transport
%   3. Medevac
%
% This script uses estimated longitudinal station locations based on the
% preliminary fuselage/cabin layout. All x-locations are measured from the
% aircraft nose.
% =========================================================================

clear; clc; close all;

%% Aircraft Reference Geometry

S_wing = 36.8;          % wing reference area [m^2]
b_wing = 15.2;          % wing span [m]
c_bar  = S_wing / b_wing;   % mean aerodynamic chord [m]

x_wing_AC = 4.30;       % wing aerodynamic center from nose [m]
x_LEMAC   = x_wing_AC - 0.25*c_bar;

% Baseline CG and approximate neutral point from stability script
x_CG_baseline = 4.00;   % baseline CG from nose [m]
x_NP_target   = 4.24;   % approximate neutral point from nose [m]

%% Preliminary CG Envelope Limits

% Envelope selected to keep all configurations forward of the approximate
% neutral point while allowing realistic loading variation.
CG_forward_percent_MAC = 5;      % forward CG limit [% MAC]
CG_aft_percent_MAC     = 20;     % aft CG limit [% MAC]

x_CG_forward = x_LEMAC + (CG_forward_percent_MAC/100)*c_bar;
x_CG_aft     = x_LEMAC + (CG_aft_percent_MAC/100)*c_bar;

%% Weight and Station Assumptions
% -------------------------------------------------------------------------
% Weight units: kg
% Station units: m from nose
% -------------------------------------------------------------------------

% Empty aircraft
% Includes structure, engine, fixed systems, unusable fluids, and fixed cabin
% equipment. Location selected to match the baseline stability estimate.
W_empty = 2172;
x_empty = 4.00;

% Crew
W_pilot_each = 90.7;        % 200 lb per pilot [kg]
x_pilots = 2.35;            % cockpit station [m]

% Fuel
W_fuel_full = 548;          % full mission fuel [kg]
x_fuel = 4.25;              % wing fuel CG near wing box [m]

% Passenger cabin
W_passenger_with_bag = 113.4;   % 250 lb passenger + baggage [kg]

% Three passenger rows, 1-2 seating arrangement
x_row1 = 3.45;
x_row2 = 4.15;
x_row3 = 4.85;

% Cargo configuration
% These locations keep the heavy generator/diesel close to the wing box
% to minimize CG travel during reconfiguration.
W_removed_seats = 90;
x_removed_seats = 4.15;

W_generator = 99.3;
x_generator = 4.05;

W_diesel = 177;
x_diesel = 4.35;

W_drum = 11.4;
x_drum = 4.35;

% Additional cargo ballast/equivalent payload
% Configuration 2 must carry payload equivalent to passenger configuration
% plus removable seats. The generator, diesel, and drum do not alone reach
% that value, so the remaining cargo is placed near the wing box.
W_cargo_equivalent_total = 1021 + 90;   % 9 passengers+baggage equivalent + seats [kg]
W_known_cargo = W_generator + W_diesel + W_drum;
W_additional_cargo = W_cargo_equivalent_total - W_known_cargo;

x_additional_cargo = 4.25;

% Medevac configuration
W_stretcher = 22.7;             % 50 lb stretcher [kg]
W_patient = 90.7;               % 200 lb patient [kg]
W_litter_patient = W_stretcher + W_patient;
x_litter_patient = 4.45;

% 4 occupants in medevac configuration:
% 2 medical technicians, 1 patient escort, and 1 additional occupant/patient
% station as required by the RFP interpretation.
W_medevac_occupants = 4*90.7;
x_medevac_occupants = 4.05;

W_med_supplies = 136;           % 300 lb medical supplies [kg]
x_med_supplies = 4.70;

%% Helper Functions

calc_cg = @(W, x) sum(W .* x) / sum(W);
to_percent_MAC = @(x) 100*(x - x_LEMAC)/c_bar;

%% Configuration 1: Passenger Transport
% 2 pilots, full fuel, 9 passengers with baggage

W_config1 = [ ...
    W_empty, ...
    2*W_pilot_each, ...
    W_fuel_full, ...
    3*W_passenger_with_bag, ...
    3*W_passenger_with_bag, ...
    3*W_passenger_with_bag];

x_config1 = [ ...
    x_empty, ...
    x_pilots, ...
    x_fuel, ...
    x_row1, ...
    x_row2, ...
    x_row3];

W_total_1 = sum(W_config1);
x_CG_1 = calc_cg(W_config1, x_config1);
CG_percent_MAC_1 = to_percent_MAC(x_CG_1);

%% Configuration 2: Cargo Transport
% 2 pilots, full fuel, removable-seat equivalent, generator, diesel,
% container, and additional equivalent cargo.

W_config2 = [ ...
    W_empty, ...
    2*W_pilot_each, ...
    W_fuel_full, ...
    W_removed_seats, ...
    W_generator, ...
    W_diesel, ...
    W_drum, ...
    W_additional_cargo];

x_config2 = [ ...
    x_empty, ...
    x_pilots, ...
    x_fuel, ...
    x_removed_seats, ...
    x_generator, ...
    x_diesel, ...
    x_drum, ...
    x_additional_cargo];

W_total_2 = sum(W_config2);
x_CG_2 = calc_cg(W_config2, x_config2);
CG_percent_MAC_2 = to_percent_MAC(x_CG_2);

%% Configuration 3: Medevac
% 2 pilots, full fuel, litter/patient, 4 cabin occupants, and medical supplies

W_config3 = [ ...
    W_empty, ...
    2*W_pilot_each, ...
    W_fuel_full, ...
    W_litter_patient, ...
    W_medevac_occupants, ...
    W_med_supplies];

x_config3 = [ ...
    x_empty, ...
    x_pilots, ...
    x_fuel, ...
    x_litter_patient, ...
    x_medevac_occupants, ...
    x_med_supplies];

W_total_3 = sum(W_config3);
x_CG_3 = calc_cg(W_config3, x_config3);
CG_percent_MAC_3 = to_percent_MAC(x_CG_3);

%% Fuel Burn Cases
% Include full-fuel and zero-fuel points to show CG remains inside envelope
% throughout the mission.

% Configuration 1 zero fuel
W_config1_zf = W_config1;
W_config1_zf(3) = 0;
W_total_1_zf = sum(W_config1_zf);
x_CG_1_zf = calc_cg(W_config1_zf, x_config1);
CG_percent_MAC_1_zf = to_percent_MAC(x_CG_1_zf);

% Configuration 2 zero fuel
W_config2_zf = W_config2;
W_config2_zf(3) = 0;
W_total_2_zf = sum(W_config2_zf);
x_CG_2_zf = calc_cg(W_config2_zf, x_config2);
CG_percent_MAC_2_zf = to_percent_MAC(x_CG_2_zf);

% Configuration 3 zero fuel
W_config3_zf = W_config3;
W_config3_zf(3) = 0;
W_total_3_zf = sum(W_config3_zf);
x_CG_3_zf = calc_cg(W_config3_zf, x_config3);
CG_percent_MAC_3_zf = to_percent_MAC(x_CG_3_zf);

%% Print Results

fprintf('===== CG Envelope Reference =====\n')
fprintf('Mean aerodynamic chord, c_bar = %.3f m\n', c_bar)
fprintf('Wing AC location = %.3f m from nose\n', x_wing_AC)
fprintf('LEMAC location = %.3f m from nose\n', x_LEMAC)
fprintf('Forward CG limit = %.3f m = %.1f%% MAC\n', ...
    x_CG_forward, CG_forward_percent_MAC)
fprintf('Aft CG limit = %.3f m = %.1f%% MAC\n', ...
    x_CG_aft, CG_aft_percent_MAC)
fprintf('Approximate neutral point = %.3f m\n\n', x_NP_target)

fprintf('===== Full-Fuel Configuration CG Results =====\n')
fprintf('Config 1 Passenger: W = %.1f kg, x_CG = %.3f m, CG = %.1f%% MAC\n', ...
    W_total_1, x_CG_1, CG_percent_MAC_1)
fprintf('Config 2 Cargo:     W = %.1f kg, x_CG = %.3f m, CG = %.1f%% MAC\n', ...
    W_total_2, x_CG_2, CG_percent_MAC_2)
fprintf('Config 3 Medevac:   W = %.1f kg, x_CG = %.3f m, CG = %.1f%% MAC\n\n', ...
    W_total_3, x_CG_3, CG_percent_MAC_3)

fprintf('===== Zero-Fuel Configuration CG Results =====\n')
fprintf('Config 1 Passenger ZF: W = %.1f kg, x_CG = %.3f m, CG = %.1f%% MAC\n', ...
    W_total_1_zf, x_CG_1_zf, CG_percent_MAC_1_zf)
fprintf('Config 2 Cargo ZF:     W = %.1f kg, x_CG = %.3f m, CG = %.1f%% MAC\n', ...
    W_total_2_zf, x_CG_2_zf, CG_percent_MAC_2_zf)
fprintf('Config 3 Medevac ZF:   W = %.1f kg, x_CG = %.3f m, CG = %.1f%% MAC\n\n', ...
    W_total_3_zf, x_CG_3_zf, CG_percent_MAC_3_zf)

%% Pass / Fail Check

CG_forward_percent = CG_forward_percent_MAC;
CG_aft_percent = CG_aft_percent_MAC;

case_names = { ...
    'Config 1 Passenger, full fuel', ...
    'Config 2 Cargo, full fuel', ...
    'Config 3 Medevac, full fuel', ...
    'Config 1 Passenger, zero fuel', ...
    'Config 2 Cargo, zero fuel', ...
    'Config 3 Medevac, zero fuel'};

CG_all = [ ...
    CG_percent_MAC_1, ...
    CG_percent_MAC_2, ...
    CG_percent_MAC_3, ...
    CG_percent_MAC_1_zf, ...
    CG_percent_MAC_2_zf, ...
    CG_percent_MAC_3_zf];

fprintf('===== Envelope Check =====\n')
for i = 1:length(CG_all)
    if CG_all(i) >= CG_forward_percent && CG_all(i) <= CG_aft_percent
        fprintf('%s: PASS\n', case_names{i})
    else
        fprintf('%s: FAIL\n', case_names{i})
    end
end

%% Plot 1: CG Envelope by Configuration

figure('Name','CG Envelope by Configuration', ...
       'Units','normalized','Position',[0.10 0.15 0.78 0.55]);

hold on; grid on; box on;

y_positions_full = [1 2 3];
y_positions_zf   = [1.18 2.18 3.18];

% Allowable CG envelope shaded region
fill([CG_forward_percent CG_aft_percent CG_aft_percent CG_forward_percent], ...
     [0.5 0.5 3.6 3.6], ...
     [0.85 0.90 1.00], ...
     'EdgeColor','none', ...
     'FaceAlpha',0.55, ...
     'DisplayName','Allowable CG Envelope');

xline(CG_forward_percent, 'k--', 'Forward CG Limit', ...
      'LineWidth',1.4, ...
      'LabelVerticalAlignment','bottom');

xline(CG_aft_percent, 'k--', 'Aft CG Limit', ...
      'LineWidth',1.4, ...
      'LabelVerticalAlignment','bottom');

% Full-fuel points
p1 = plot(CG_percent_MAC_1, 1, 'o', 'MarkerSize',10, ...
    'MarkerFaceColor','b', 'MarkerEdgeColor','k', ...
    'DisplayName','Full Fuel');

plot(CG_percent_MAC_2, 2, 'o', 'MarkerSize',10, ...
    'MarkerFaceColor','b', 'MarkerEdgeColor','k', ...
    'HandleVisibility','off');

plot(CG_percent_MAC_3, 3, 'o', 'MarkerSize',10, ...
    'MarkerFaceColor','b', 'MarkerEdgeColor','k', ...
    'HandleVisibility','off');

% Zero-fuel points
p2 = plot(CG_percent_MAC_1_zf, 1.18, 's', 'MarkerSize',9, ...
    'MarkerFaceColor','r', 'MarkerEdgeColor','k', ...
    'DisplayName','Zero Fuel');

plot(CG_percent_MAC_2_zf, 2.18, 's', 'MarkerSize',9, ...
    'MarkerFaceColor','r', 'MarkerEdgeColor','k', ...
    'HandleVisibility','off');

plot(CG_percent_MAC_3_zf, 3.18, 's', 'MarkerSize',9, ...
    'MarkerFaceColor','r', 'MarkerEdgeColor','k', ...
    'HandleVisibility','off');

% Connect full-fuel to zero-fuel points
plot([CG_percent_MAC_1 CG_percent_MAC_1_zf], [1 1.18], 'k-', 'HandleVisibility','off')
plot([CG_percent_MAC_2 CG_percent_MAC_2_zf], [2 2.18], 'k-', 'HandleVisibility','off')
plot([CG_percent_MAC_3 CG_percent_MAC_3_zf], [3 3.18], 'k-', 'HandleVisibility','off')

yticks([1.09 2.09 3.09])
yticklabels({'Config 1: Passenger','Config 2: Cargo','Config 3: Medevac'})

xlabel('Center of Gravity Location [% MAC]')
ylabel('Mission Configuration')
title('Preliminary Center of Gravity Envelope Check')

xlim([CG_forward_percent - 3, CG_aft_percent + 3])
ylim([0.5 3.6])

legend('Location','eastoutside')

%% Plot 2: Weight vs CG Envelope

figure('Name','Weight vs CG Envelope', ...
       'Units','normalized','Position',[0.10 0.15 0.78 0.55]);

hold on; grid on; box on;

CG_plot = [ ...
    CG_percent_MAC_1, CG_percent_MAC_2, CG_percent_MAC_3, ...
    CG_percent_MAC_1_zf, CG_percent_MAC_2_zf, CG_percent_MAC_3_zf];

W_plot = [ ...
    W_total_1, W_total_2, W_total_3, ...
    W_total_1_zf, W_total_2_zf, W_total_3_zf];

W_min = min(W_plot) - 150;
W_max = max(W_plot) + 150;

% Envelope region
fill([CG_forward_percent CG_aft_percent CG_aft_percent CG_forward_percent], ...
     [W_min W_min W_max W_max], ...
     [0.85 0.90 1.00], ...
     'EdgeColor','none', ...
     'FaceAlpha',0.55, ...
     'DisplayName','Allowable CG Envelope');

xline(CG_forward_percent, 'k--', 'Forward CG Limit', 'LineWidth',1.4);
xline(CG_aft_percent, 'k--', 'Aft CG Limit', 'LineWidth',1.4);

% Full fuel points
plot(CG_percent_MAC_1, W_total_1, 'o', 'MarkerSize',10, ...
    'MarkerFaceColor','b', 'MarkerEdgeColor','k', ...
    'DisplayName','Config 1 Full Fuel');

plot(CG_percent_MAC_2, W_total_2, 'o', 'MarkerSize',10, ...
    'MarkerFaceColor','g', 'MarkerEdgeColor','k', ...
    'DisplayName','Config 2 Full Fuel');

plot(CG_percent_MAC_3, W_total_3, 'o', 'MarkerSize',10, ...
    'MarkerFaceColor','m', 'MarkerEdgeColor','k', ...
    'DisplayName','Config 3 Full Fuel');

% Zero fuel points
plot(CG_percent_MAC_1_zf, W_total_1_zf, 's', 'MarkerSize',9, ...
    'MarkerFaceColor','b', 'MarkerEdgeColor','k', ...
    'DisplayName','Config 1 Zero Fuel');

plot(CG_percent_MAC_2_zf, W_total_2_zf, 's', 'MarkerSize',9, ...
    'MarkerFaceColor','g', 'MarkerEdgeColor','k', ...
    'DisplayName','Config 2 Zero Fuel');

plot(CG_percent_MAC_3_zf, W_total_3_zf, 's', 'MarkerSize',9, ...
    'MarkerFaceColor','m', 'MarkerEdgeColor','k', ...
    'DisplayName','Config 3 Zero Fuel');

% Fuel burn lines
plot([CG_percent_MAC_1 CG_percent_MAC_1_zf], ...
     [W_total_1 W_total_1_zf], 'b-', 'LineWidth',1.4, ...
     'HandleVisibility','off');

plot([CG_percent_MAC_2 CG_percent_MAC_2_zf], ...
     [W_total_2 W_total_2_zf], 'g-', 'LineWidth',1.4, ...
     'HandleVisibility','off');

plot([CG_percent_MAC_3 CG_percent_MAC_3_zf], ...
     [W_total_3 W_total_3_zf], 'm-', 'LineWidth',1.4, ...
     'HandleVisibility','off');

xlabel('Center of Gravity Location [% MAC]')
ylabel('Aircraft Weight [kg]')
title('Aircraft Weight vs Center of Gravity Location')

xlim([CG_forward_percent - 3, CG_aft_percent + 3])
ylim([W_min W_max])

legend('Location','eastoutside')

%% Save Figures

print(figure(1), 'CG_Envelope_by_Configuration.png', '-dpng', '-r600')
print(figure(2), 'Weight_vs_CG_Envelope.png', '-dpng', '-r600')