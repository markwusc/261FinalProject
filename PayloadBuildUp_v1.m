%% PayloadBuildUp_v1.m
% =========================================================================
% Arctic STOL Aircraft — Payload Mass Build-Up
% Purpose: Itemized payload weight breakdown for all three RFP configurations.
%
% SCOPE OF THIS SCRIPT:
%   - Payload weights ONLY. OEW, fuel, and full MTOW closure are handled
%     in RaymerWeightEst_v3.m. This script feeds into that one.
%   - OEW is fixed at the Config 2 MTOW-converged value (40,000 N / 4,077 kg).
%   - No fuel fractions or fuel weights are computed here.
%   - Config 2 is the SIZING CASE (heaviest payload) — all structural and
%     fuel sizing is driven by Config 2. Configs 1 and 3 are lighter and
%     do not re-drive the structure.
%
% CONFIGURATIONS:
%   Config 1 — Passenger Transport
%     9 passengers + 1 flight attendant (FA, added beyond RFP minimum)
%     10 seats (9 pax + 1 FA jump seat)
%     Passenger baggage per pax, pilot baggage allowance for FA
%
%   Config 2 — Cargo Transport (SIZING CASE, heaviest payload)
%     Cargo payload mass-equivalent to 9-pax configuration PLUS seat weight
%     8,000 W diesel generator + 55 gal diesel fuel in drum
%     NOTE: No passengers in Config 2 — seats are REMOVED and replaced by cargo
%
%   Config 3 — Aerial Ambulance (Medevac), two loading conditions:
%     Outbound: 2 med techs + litter + medical supplies (no patient yet)
%     Inbound:  2 med techs + litter + medical supplies + 1 patient + 1 escort
%     No baggage for any occupant in Config 3 (emergency ops assumption)
%     Aircraft must carry fuel for both legs from the start (no refuel at site)
%
% ⚠ FLAGS are raised inline wherever a number is uncertain or assumed.
%
% DESIGN NOTE ON ITERATION:
%   Weight drives everything. As structural estimates improve (component
%   weight buildup), return here to verify payload fractions have not
%   shifted. Any change to seat mass, pax count, or med equipment mass
%   propagates directly into MTOW through RaymerWeightEst_v3.m.
%
% Authors:  AME 261 Design Team
% Date:     May 2026
% Version:  1.0
% Units:    SI (kg) throughout. lbf shown parenthetically for RFP traceability.
% =========================================================================

clear; clc;

%% =========================================================================
%  SECTION 1 — SHARED CONSTANTS
%  These values apply across all configurations. Change here only — do not
%  hardcode the same number in multiple places.
% =========================================================================

% --- Conversion factor (for RFP traceability printout only) ---
kg_per_lb = 0.453592;   % [kg/lb]

% --- Standard occupant body weight (RFP §1.2) ---
% RFP specifies 200 lb per person for all occupants (pax, crew, med staff).
W_person_kg = 200 * kg_per_lb;      % 90.72 kg  [kg]

% --- Passenger baggage allowance (RFP §1.2) ---
% 50 lb per passenger, checked baggage.
W_pax_bag_kg = 50 * kg_per_lb;      % 22.68 kg  [kg]

% --- Pilot / FA baggage allowance (RFP §1.2) ---
% RFP assigns 20 lb per pilot. FA uses same allowance per design decision.
W_pilot_bag_kg = 20 * kg_per_lb;    % 9.07 kg   [kg]

% --- Passenger seat mass ---
% Recaro BL 3530 lightweight unit: 10 kg per seat.
% Used for all 9 pax seats AND the FA seat.
% ⚠ FLAG: Seat mass is an estimate from manufacturer datasheet.
%   Confirm once seat selection is finalized.
W_seat_kg = 10.0;                   % [kg]  per seat

% --- Litter (NATO stretcher) mass (RFP §1.2) ---
% RFP: 50 lb. Using exact RFP value.
W_litter_kg = 50 * kg_per_lb;       % 22.68 kg  [kg]

% --- Medical supplies and equipment (RFP §1.2) ---
% RFP: 300 lb total, single line item. No sub-breakdown available yet.
% ⚠ FLAG: This is an RFP-specified aggregate mass. As equipment is selected
%   (monitor, defibrillator, O2 bottles, drug kit, etc.), replace with
%   itemized values. Mass may increase — recheck MTOW closure if it does.
W_med_supplies_kg = 300 * kg_per_lb; % 136.08 kg [kg]

% --- Diesel fuel properties (for Config 2 cargo fuel delivery) ---
% ⚠ NOTE: This is the CARGO diesel fuel delivered to remote sites,
%   NOT the aircraft's own propulsion fuel. Do not confuse with Wf in Raymer.
rho_diesel_kg_L  = 0.85;            % [kg/L]  density of diesel fuel
vol_diesel_gal   = 55;              % [US gal] volume of cargo diesel
L_per_gal        = 3.7854;          % [L/US gal]
vol_diesel_L     = vol_diesel_gal * L_per_gal;   % 208.2 L
W_diesel_fuel_kg = rho_diesel_kg_L * vol_diesel_L; % [kg]

% --- Cargo diesel fuel drum (55-gal plastic open-top drum) ---
% Source: Uline S-9945BLU product spec.
% ⚠ FLAG: Confirm final drum selection. Mass is manufacturer-quoted.
W_drum_kg = 11.4;                   % [kg]

% --- 8,000 W diesel generator ---
% Source: Generac unit from Amazon product link on team spreadsheet.
% Listed mass from product page: ~219 lb.
% ⚠ FLAG: Confirm this is the correct generator. The 99.3 kg value from the
%   spreadsheet is used here; the report draft earlier cited 250 kg for a
%   different unit. This MUST be reconciled — a 150 kg discrepancy is
%   not trivial at this weight class. Source the actual shipping weight
%   from the manufacturer spec sheet, not the Amazon listing.
W_generator_kg = 99.3;              % [kg]  ⚠ VERIFY

% --- MTOW anchor (Config 2 converged value from RaymerWeightEst_v3.m) ---
% OEW is frozen at this sizing point. All configs fly an aircraft
% sized for Config 2. Configs 1 and 3 will have payload margin to spare.
MTOW_N  = 40000;                    % [N]   design MTOW
g       = 9.81;                     % [m/s²]
MTOW_kg = MTOW_N / g;               % 4077.5 kg


%% =========================================================================
%  SECTION 2 — CONFIGURATION 1: PASSENGER TRANSPORT
%
%  RFP requirements:
%    - 9 passengers at 200 lb body + 50 lb baggage each
%    - Minimum 31-inch seat pitch
%  Design additions beyond RFP:
%    - 1 flight attendant (FA) added for operational safety and service
%      FA uses pilot body weight (200 lb) and pilot baggage allowance (20 lb)
%      FA seat: 10 kg (same as pax seat, mounted as jump seat forward of Row 1)
%    - 10th seat added for FA
%
%  Payload DOES include removable seats — when seats are removed for Config 2,
%  that seat mass becomes part of the cargo payload equivalence.
%  Payload does NOT include pilot seats (part of OEW/structure).
% =========================================================================

fprintf('=================================================================\n');
fprintf('  CONFIGURATION 1 — PASSENGER TRANSPORT\n');
fprintf('=================================================================\n\n');

n_pax_C1 = 9;   % number of revenue passengers

% --- Occupant weights ---
W_pax_body_C1   = n_pax_C1 * W_person_kg;       % 9 pax body weight
W_pax_bag_C1    = n_pax_C1 * W_pax_bag_kg;       % 9 pax baggage
W_FA_body_C1    = 1        * W_person_kg;         % 1 FA body weight
W_FA_bag_C1     = 1        * W_pilot_bag_kg;      % FA baggage (pilot allowance)

% --- Seat weights ---
% 9 pax seats + 1 FA seat = 10 seats total
n_seats_C1      = n_pax_C1 + 1;                   % 10 seats
W_seats_C1      = n_seats_C1 * W_seat_kg;          % total removable seat mass

% --- Config 1 total payload ---
W_payload_C1 = W_pax_body_C1 + W_pax_bag_C1 ...
             + W_FA_body_C1  + W_FA_bag_C1  ...
             + W_seats_C1;

% --- Print itemized breakdown ---
fprintf('  %-40s  %8s  %8s\n', 'Item', 'Mass (kg)', 'Mass (lb)');
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('%d passengers body weight (x%.0f kg)', n_pax_C1, W_person_kg), ...
    W_pax_body_C1,  W_pax_body_C1  / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('%d passengers baggage (x%.2f kg)', n_pax_C1, W_pax_bag_kg), ...
    W_pax_bag_C1,   W_pax_bag_C1   / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '1 flight attendant body weight', ...
    W_FA_body_C1,   W_FA_body_C1   / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '1 FA baggage (pilot allowance, 20 lb)', ...
    W_FA_bag_C1,    W_FA_bag_C1    / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('%d removable seats (x%.0f kg each)', n_seats_C1, W_seat_kg), ...
    W_seats_C1,     W_seats_C1     / kg_per_lb);
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', 'TOTAL CONFIG 1 PAYLOAD', ...
    W_payload_C1, W_payload_C1 / kg_per_lb);
fprintf('\n');
fprintf('  NOTE: Pilot seats NOT included (part of OEW/structure).\n');
fprintf('  NOTE: FA is a design addition beyond RFP minimum crew spec.\n');
fprintf('        Adds %.2f kg to payload vs RFP-only configuration.\n', ...
    W_FA_body_C1 + W_FA_bag_C1 + W_seat_kg);
fprintf('\n\n');


%% =========================================================================
%  SECTION 3 — CONFIGURATION 2: CARGO TRANSPORT (SIZING CASE)
%
%  RFP requirements:
%    - Cargo weight equivalent to the 9-passenger configuration PLUS
%      the weight of the removable seats
%    - Must transport an 8,000 W generator and 55 gal of diesel fuel
%      (in no more than 3 containers, preferably one)
%
%  DESIGN NOTE: Config 2 is the SIZING CASE because it is the heaviest
%  payload. The Raymer iteration in RaymerWeightEst_v3.m is driven by
%  this configuration. Do not reduce this payload without re-running
%  the Raymer convergence loop.
%
%  Payload equivalence logic:
%    RFP says cargo must equal "9-pax payload + seat weight."
%    9-pax payload = 9 × (200 lb body + 50 lb baggage) = 2,250 lb = 1,021 kg
%    Seat weight   = 9 seats × 10 kg = 90 kg  (pax seats only; FA seat
%                    is Config 1 specific and NOT present in Config 2)
%    Equivalence total = 1,021 + 90 = 1,111 kg
%    Generator + fuel + drum = 99.3 + 177.0 + 11.4 = 287.7 kg
%    Total Config 2 payload  = 1,111 + 287.7 = 1,398.7 kg ≈ 1,399 kg
%
%  ⚠ FLAG: Config 2 does NOT carry a flight attendant or FA seat.
%    The FA is a Config 1 design addition. Config 2 is pure cargo.
%    This means Config 1 (1,212 kg) is LIGHTER than Config 2 (1,399 kg),
%    which is correct and expected — Config 2 remains the sizing case.
% =========================================================================

fprintf('=================================================================\n');
fprintf('  CONFIGURATION 2 — CARGO TRANSPORT  (SIZING CASE)\n');
fprintf('=================================================================\n\n');

n_pax_C2    = 9;   % equivalent passenger count for mass equivalence calc
n_seats_C2  = 9;   % only the 9 pax removable seats (FA seat not in Config 2)

% --- Passenger mass equivalence (cargo replaces these people) ---
W_pax_equiv_C2  = n_pax_C2 * W_person_kg;       % body weight equivalent
W_bag_equiv_C2  = n_pax_C2 * W_pax_bag_kg;       % baggage equivalent
W_seats_C2      = n_seats_C2 * W_seat_kg;         % seat mass (9 seats removed)

% --- Cargo items ---
W_diesel_C2     = W_diesel_fuel_kg;               % 55 gal cargo diesel
W_drum_C2       = W_drum_kg;                      % fuel drum
W_gen_C2        = W_generator_kg;                 % 8,000 W generator

% --- Config 2 total payload ---
W_payload_C2 = W_pax_equiv_C2 + W_bag_equiv_C2 ...
             + W_seats_C2 ...
             + W_diesel_C2 + W_drum_C2 + W_gen_C2;

% --- Print itemized breakdown ---
fprintf('  %-40s  %8s  %8s\n', 'Item', 'Mass (kg)', 'Mass (lb)');
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '--- Passenger Mass Equivalence ---', 0, 0);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('%d pax body equiv. (x%.2f kg)', n_pax_C2, W_person_kg), ...
    W_pax_equiv_C2,  W_pax_equiv_C2  / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('%d pax baggage equiv. (x%.2f kg)', n_pax_C2, W_pax_bag_kg), ...
    W_bag_equiv_C2,  W_bag_equiv_C2  / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('%d removable seats (x%.0f kg each)', n_seats_C2, W_seat_kg), ...
    W_seats_C2,      W_seats_C2      / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '--- Cargo Items ---', 0, 0);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    sprintf('55 gal diesel (rho=%.2f kg/L)', rho_diesel_kg_L), ...
    W_diesel_C2,     W_diesel_C2     / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '55-gal plastic drum (Uline S-9945BLU)', ...
    W_drum_C2,       W_drum_C2       / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '8,000 W diesel generator [⚠ VERIFY]', ...
    W_gen_C2,        W_gen_C2        / kg_per_lb);
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', 'TOTAL CONFIG 2 PAYLOAD', ...
    W_payload_C2, W_payload_C2 / kg_per_lb);
fprintf('\n');
fprintf('  ⚠ FLAG: Generator mass (%.1f kg) conflicts with report draft\n', W_gen_C2);
fprintf('          which cited 250 kg for a different unit. Reconcile\n');
fprintf('          with manufacturer spec sheet before finalizing.\n');
fprintf('  NOTE:   Config 2 is the SIZING CASE. MTOW is driven by this\n');
fprintf('          payload. Do not reduce without re-running RaymerWeightEst_v3.m.\n');
fprintf('\n\n');


%% =========================================================================
%  SECTION 4 — CONFIGURATION 3: AERIAL AMBULANCE (MEDEVAC)
%
%  RFP requirements:
%    - 1 litter (50 lb stretcher)
%    - 4 occupants: 2 medical technicians, 1 patient, 1 patient escort
%      (all at 200 lb per RFP)
%    - 300 lb of medical supplies and equipment
%    - Total RFP payload: 1,150 lb = 521.6 kg
%    - NO refueling at remote site — aircraft carries fuel for both legs
%    - Round trip: outbound (no patient) + 45 min turnaround + inbound (with patient)
%
%  TWO LOADING CONDITIONS:
%    Outbound: aircraft departs with med techs + litter + supplies
%              Patient and escort are NOT yet on board
%    Inbound:  Patient and escort loaded at remote site
%              Heavier condition — this is the critical structural loading
%
%  Baggage assumption: NO baggage for any Config 3 occupant.
%    Med techs: personal gear subsumed in medical supplies line item.
%    Patient / escort: emergency evacuation — no baggage.
%
%  DESIGN NOTE ON FUEL:
%    This script does NOT compute fuel weight. However, note that Config 3
%    requires fuel for 2x cruise legs (350 nmi each) + 4x loiter segments.
%    This will yield a higher fuel fraction than Configs 1 and 2 despite
%    lower payload. Full fuel analysis is handled in RaymerWeightEst_v3.m.
% =========================================================================

fprintf('=================================================================\n');
fprintf('  CONFIGURATION 3 — AERIAL AMBULANCE (MEDEVAC)\n');
fprintf('=================================================================\n\n');

% --- Items present on BOTH outbound and inbound legs ---
W_med_techs_C3   = 2 * W_person_kg;     % 2 medical technicians (no baggage)
W_litter_C3      = W_litter_kg;          % NATO stretcher
W_med_equip_C3   = W_med_supplies_kg;    % 300 lb medical supplies (single line)

% --- Items added ONLY on inbound leg (loaded at remote site) ---
W_patient_C3     = 1 * W_person_kg;      % patient body weight (no baggage)
W_escort_C3      = 1 * W_person_kg;      % patient escort body weight (no baggage)

% --- Subtotals ---
W_payload_C3_out = W_med_techs_C3 + W_litter_C3 + W_med_equip_C3;
W_payload_C3_in  = W_payload_C3_out + W_patient_C3 + W_escort_C3;

% --- OUTBOUND breakdown ---
fprintf('  --- OUTBOUND LEG (no patient on board) ---\n\n');
fprintf('  %-40s  %8s  %8s\n', 'Item', 'Mass (kg)', 'Mass (lb)');
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '2 medical technicians (no baggage)', ...
    W_med_techs_C3,  W_med_techs_C3  / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    'NATO litter / stretcher (50 lb RFP)', ...
    W_litter_C3,     W_litter_C3     / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    'Medical supplies & equipment (300 lb RFP) [⚠]', ...
    W_med_equip_C3,  W_med_equip_C3  / kg_per_lb);
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', 'TOTAL OUTBOUND PAYLOAD', ...
    W_payload_C3_out, W_payload_C3_out / kg_per_lb);
fprintf('\n');

% --- INBOUND breakdown ---
fprintf('  --- INBOUND LEG (patient and escort loaded) ---\n\n');
fprintf('  %-40s  %8s  %8s\n', 'Item', 'Mass (kg)', 'Mass (lb)');
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '2 medical technicians (no baggage)', ...
    W_med_techs_C3,  W_med_techs_C3  / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    'NATO litter / stretcher (50 lb RFP)', ...
    W_litter_C3,     W_litter_C3     / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    'Medical supplies & equipment (300 lb RFP) [⚠]', ...
    W_med_equip_C3,  W_med_equip_C3  / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '1 patient body weight (no baggage)', ...
    W_patient_C3,    W_patient_C3    / kg_per_lb);
fprintf('  %-40s  %8.2f  %8.2f\n', ...
    '1 patient escort body weight (no baggage)', ...
    W_escort_C3,     W_escort_C3     / kg_per_lb);
fprintf('  %s\n', repmat('-', 1, 60));
fprintf('  %-40s  %8.2f  %8.2f\n', 'TOTAL INBOUND PAYLOAD', ...
    W_payload_C3_in, W_payload_C3_in / kg_per_lb);
fprintf('\n');
fprintf('  NOTE: No baggage for any Config 3 occupant (emergency ops).\n');
fprintf('  ⚠ FLAG: Medical supplies (%.2f kg) is an RFP aggregate.\n', W_med_equip_C3);
fprintf('          Replace with itemized list once equipment is selected.\n');
fprintf('\n\n');


%% =========================================================================
%  SECTION 5 — SUMMARY TABLE
%  Comparison of all configurations against the MTOW sizing anchor.
%  Helps verify payload fractions are consistent with RaymerWeightEst_v3.m.
% =========================================================================

fprintf('=================================================================\n');
fprintf('  PAYLOAD SUMMARY — ALL CONFIGURATIONS\n');
fprintf('=================================================================\n\n');

fprintf('  %-35s  %10s  %10s  %10s\n', 'Configuration', 'Payload(kg)', ...
    'Payload(lb)', '% of MTOW');
fprintf('  %s\n', repmat('-', 1, 70));
fprintf('  %-35s  %10.2f  %10.2f  %10.1f\n', ...
    'Config 1 — Pax Transport', ...
    W_payload_C1, W_payload_C1 / kg_per_lb, ...
    100 * W_payload_C1 / MTOW_kg);
fprintf('  %-35s  %10.2f  %10.2f  %10.1f\n', ...
    'Config 2 — Cargo (SIZING CASE)', ...
    W_payload_C2, W_payload_C2 / kg_per_lb, ...
    100 * W_payload_C2 / MTOW_kg);
fprintf('  %-35s  %10.2f  %10.2f  %10.1f\n', ...
    'Config 3 — Medevac Outbound', ...
    W_payload_C3_out, W_payload_C3_out / kg_per_lb, ...
    100 * W_payload_C3_out / MTOW_kg);
fprintf('  %-35s  %10.2f  %10.2f  %10.1f\n', ...
    'Config 3 — Medevac Inbound', ...
    W_payload_C3_in, W_payload_C3_in / kg_per_lb, ...
    100 * W_payload_C3_in / MTOW_kg);
fprintf('\n');
fprintf('  MTOW anchor (Config 2, converged): %.0f N  /  %.2f kg\n', ...
    MTOW_N, MTOW_kg);
fprintf('  OEW is fixed at this sizing point. Fuel sizing in RaymerWeightEst_v3.m.\n');
fprintf('\n\n');


%% =========================================================================
%  SECTION 6 — CSV EXPORT
%  Writes one table per configuration. All masses in kg and lb.
%  File: PayloadBuildUp_v1.csv
%  Intended to replace the manual spreadsheet (Sheet2) as the authoritative
%  payload record. Update this script, not the spreadsheet, going forward.
% =========================================================================

csv_path = 'PayloadBuildUp_v1.csv';
fid = fopen(csv_path, 'w');

% --- Header row convention ---
% Configuration, Item, Mass_kg, Mass_lb, Notes

fprintf(fid, 'Configuration,Item,Mass_kg,Mass_lb,Notes\n');

% --- Config 1 ---
fprintf(fid, 'Config 1 - Passenger Transport,');
fprintf(fid, '%d passengers body weight,%.2f,%.2f,RFP: 200 lb each\n', ...
    n_pax_C1, W_pax_body_C1, W_pax_body_C1/kg_per_lb);

fprintf(fid, 'Config 1 - Passenger Transport,');
fprintf(fid, '%d passengers baggage,%.2f,%.2f,RFP: 50 lb each\n', ...
    n_pax_C1, W_pax_bag_C1, W_pax_bag_C1/kg_per_lb);

fprintf(fid, 'Config 1 - Passenger Transport,');
fprintf(fid, '1 flight attendant body weight,%.2f,%.2f,Design addition - not in RFP\n', ...
    W_FA_body_C1, W_FA_body_C1/kg_per_lb);

fprintf(fid, 'Config 1 - Passenger Transport,');
fprintf(fid, '1 FA baggage (pilot allowance),%.2f,%.2f,20 lb pilot allowance per design decision\n', ...
    W_FA_bag_C1, W_FA_bag_C1/kg_per_lb);

fprintf(fid, 'Config 1 - Passenger Transport,');
fprintf(fid, '%d removable seats (10 kg each),%.2f,%.2f,9 pax + 1 FA seat\n', ...
    n_seats_C1, W_seats_C1, W_seats_C1/kg_per_lb);

fprintf(fid, 'Config 1 - Passenger Transport,');
fprintf(fid, 'TOTAL,%.2f,%.2f,\n', W_payload_C1, W_payload_C1/kg_per_lb);

fprintf(fid, '\n');

% --- Config 2 ---
fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, '%d pax body weight equivalent,%.2f,%.2f,Mass equivalence per RFP\n', ...
    n_pax_C2, W_pax_equiv_C2, W_pax_equiv_C2/kg_per_lb);

fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, '%d pax baggage equivalent,%.2f,%.2f,Mass equivalence per RFP\n', ...
    n_pax_C2, W_bag_equiv_C2, W_bag_equiv_C2/kg_per_lb);

fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, '%d removable seats (10 kg each),%.2f,%.2f,9 pax seats only - no FA seat in Config 2\n', ...
    n_seats_C2, W_seats_C2, W_seats_C2/kg_per_lb);

fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, '55 gal diesel cargo fuel (rho=%.2f kg/L),%.2f,%.2f,Delivered to remote site\n', ...
    rho_diesel_kg_L, W_diesel_C2, W_diesel_C2/kg_per_lb);

fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, '55-gal plastic drum (Uline S-9945BLU),%.2f,%.2f,VERIFY with final drum selection\n', ...
    W_drum_C2, W_drum_C2/kg_per_lb);

fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, '8000 W diesel generator,%.2f,%.2f,FLAG: VERIFY mass - conflict with 250 kg in report draft\n', ...
    W_gen_C2, W_gen_C2/kg_per_lb);

fprintf(fid, 'Config 2 - Cargo Transport (SIZING CASE),');
fprintf(fid, 'TOTAL,%.2f,%.2f,SIZING CASE - drives MTOW\n', ...
    W_payload_C2, W_payload_C2/kg_per_lb);

fprintf(fid, '\n');

% --- Config 3 Outbound ---
fprintf(fid, 'Config 3 - Medevac Outbound,');
fprintf(fid, '2 medical technicians (no baggage),%.2f,%.2f,Emergency ops - no personal baggage\n', ...
    W_med_techs_C3, W_med_techs_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Outbound,');
fprintf(fid, 'NATO litter / stretcher,%.2f,%.2f,RFP: 50 lb\n', ...
    W_litter_C3, W_litter_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Outbound,');
fprintf(fid, 'Medical supplies and equipment,%.2f,%.2f,RFP: 300 lb aggregate - itemize when equipment selected\n', ...
    W_med_equip_C3, W_med_equip_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Outbound,');
fprintf(fid, 'TOTAL,%.2f,%.2f,No patient on outbound leg\n', ...
    W_payload_C3_out, W_payload_C3_out/kg_per_lb);

fprintf(fid, '\n');

% --- Config 3 Inbound ---
fprintf(fid, 'Config 3 - Medevac Inbound,');
fprintf(fid, '2 medical technicians (no baggage),%.2f,%.2f,Emergency ops - no personal baggage\n', ...
    W_med_techs_C3, W_med_techs_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Inbound,');
fprintf(fid, 'NATO litter / stretcher,%.2f,%.2f,RFP: 50 lb\n', ...
    W_litter_C3, W_litter_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Inbound,');
fprintf(fid, 'Medical supplies and equipment,%.2f,%.2f,RFP: 300 lb aggregate\n', ...
    W_med_equip_C3, W_med_equip_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Inbound,');
fprintf(fid, '1 patient body weight (no baggage),%.2f,%.2f,Loaded at remote site\n', ...
    W_patient_C3, W_patient_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Inbound,');
fprintf(fid, '1 patient escort body weight (no baggage),%.2f,%.2f,Loaded at remote site\n', ...
    W_escort_C3, W_escort_C3/kg_per_lb);

fprintf(fid, 'Config 3 - Medevac Inbound,');
fprintf(fid, 'TOTAL,%.2f,%.2f,Heaviest Config 3 condition\n', ...
    W_payload_C3_in, W_payload_C3_in/kg_per_lb);

fprintf(fid, '\n');

% --- Summary ---
fprintf(fid, 'SUMMARY,Config 1 - Pax Transport,%.2f,%.2f,\n', ...
    W_payload_C1, W_payload_C1/kg_per_lb);
fprintf(fid, 'SUMMARY,Config 2 - Cargo (SIZING CASE),%.2f,%.2f,\n', ...
    W_payload_C2, W_payload_C2/kg_per_lb);
fprintf(fid, 'SUMMARY,Config 3 - Medevac Outbound,%.2f,%.2f,\n', ...
    W_payload_C3_out, W_payload_C3_out/kg_per_lb);
fprintf(fid, 'SUMMARY,Config 3 - Medevac Inbound,%.2f,%.2f,\n', ...
    W_payload_C3_in, W_payload_C3_in/kg_per_lb);
fprintf(fid, 'SUMMARY,MTOW anchor (Config 2 converged),%.2f,%.2f,40000 N / 4077.5 kg\n', ...
    MTOW_kg, MTOW_kg/kg_per_lb);

fclose(fid);

fprintf('=================================================================\n');
fprintf('  CSV EXPORT COMPLETE\n');
fprintf('  File written: %s\n', csv_path);
fprintf('=================================================================\n\n');
fprintf('  ⚠ DESIGN FLAGS SUMMARY:\n');
fprintf('    1. Generator mass: 99.3 kg used here vs 250 kg in report draft.\n');
fprintf('       Pull manufacturer spec sheet and reconcile before next iteration.\n');
fprintf('    2. Medical supplies (136 kg) is an RFP aggregate.\n');
fprintf('       Itemize once equipment manifest is finalized.\n');
fprintf('    3. Seat mass (10 kg each) is a Recaro BL 3530 estimate.\n');
fprintf('       Confirm once seat selection is locked.\n');
fprintf('    4. FA is a DESIGN ADDITION beyond the RFP. Adds %.2f kg\n', ...
    W_FA_body_C1 + W_FA_bag_C1 + W_seat_kg);
fprintf('       to Config 1 payload vs an RFP-only reading.\n');
fprintf('       If FA is removed, Config 1 payload drops to %.2f kg.\n', ...
    W_payload_C1 - W_FA_body_C1 - W_FA_bag_C1 - W_seat_kg);
fprintf('\n');
