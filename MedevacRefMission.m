% MedevacMission_v1.m
% =========================================================================
% PUDO 670 — Medevac Mission Performance Analysis
% Configuration 3  |  Round-Trip  |  350 nmi per leg (≥ 350 nmi RFP)
% =========================================================================
%
% MISSION PROFILE (per RFP §Medevac, 03/09/2026 v0):
%
%   OUTBOUND (segments 1-7, legs 3-5 range ≥ 350 nmi):
%     (1)  Warm-up & taxi          10 min,  Raymer ff = 0.990
%     (2)  Takeoff over 50 ft      STOL,    Raymer ff = 0.995
%     (3)  Climb  SL→h_cr          Raymer ff = 0.980 (fuel); kinematic sim (dist/time)
%     (4)  Cruise at best-endurance speed / altitude
%     (5)  Descent to sea level    Raymer ff = 0.990
%     (6)  Land over 50 ft ≤ 500ft Raymer ff = 0.992
%    [6a]  Climb to best-loiter alt  \
%    [6b]  Loiter 45 min at best CL  > reserve (NOT counted in mission time)
%    [6c]  Descend to sea level      /
%     (7)  10 min taxi & shutdown
%
%   TURNAROUND: 45 min (counted in total mission time)
%
%   INBOUND (segments 8-14, same range as outbound):
%     (8)  Warm-up & taxi          10 min,  ff = 0.990
%     (9)  Takeoff over 50 ft      STOL,    ff = 0.995
%     (10) Climb  SL→h_cr          ff = 0.980 (fuel); kinematic sim (dist/time)
%     (11) Cruise at best-endurance speed / altitude
%     (12) Descent to sea level    ff = 0.990
%     (13) Land over 50 ft ≤ 500ft ff = 0.992
%    [13a] Climb to best-loiter alt  \
%    [13b] Loiter 45 min at best CL  > reserve = SAME fuel as outbound 6a-6c
%    [13c] Descend to sea level      /
%     (14) 10 min taxi & shutdown
%
% TOTAL MISSION TIME = t_OB_air (excl. 6a-6c) + 45 min turnaround
%                    + t_IB_air (excl. 13a-13c) + ground ops (1,7,8,14)
%
% CRUISE SELECTION — BEST-ENDURANCE SPEED (user decision):
%   RFP states no speed/altitude requirement for medevac cruise.
%   Best-endurance speed for propeller aircraft:
%     V_BE(h) = sqrt(2W / (rho(h)*S)) * (k/(3*CD0))^(1/4)
%   Cruise altitude chosen so that V_BE(h, W_mid) is physically reasonable
%   (≥ stall speed with margin). Sea-level best-endurance is used here as the
%   most fuel-conservative option; altitude is iterated to find where V_BE
%   equals the stall speed floor (1.3*V_stall). A fixed cruise altitude of
%   h_cruise = 2000 m is adopted as a conservative default that keeps V_BE
%   above stall for the expected weight range. Update if weight changes.
%
% ITERATION STRATEGY:
%   Each leg (outbound / inbound) is iterated independently.
%   W6_target for each leg is pre-solved from the 45-min loiter constraint:
%     burn exactly to W_dry_leg in 45 min at sea level at best endurance.
%   W_fuel per leg is iterated via Newton step until W6_computed = W6_target.
%   Because inbound payload differs from outbound (patient + escort added),
%   W_dry_IB ≠ W_dry_OB, so W6_target_IB ≠ W6_target_OB.
%   Total fuel = W_fuel_OB + W_fuel_IB; total MTOW = max(W0_OB, W0_IB).
%
% KEY DESIGN DECISIONS:
%   - Climb TAS = 70 m/s (same as reference mission, team decision)
%   - Weight held CONSTANT during kinematic climb sim (no per-dt fuel burn)
%   - CD0 = 0.0288 for ALL phases (clean config)
%   - Power lapse: P_A(h) = sigma(h) * P_A_SL (linear, project convention)
%   - Loiter at sea level (conservative — max fuel burn rate)
%   - Cruise at best-endurance speed: V_BE = (4k/3*CD0)^(1/4)*sqrt(2W/(rho*S))
%   - L/D at best endurance = CL_BE / CD_BE (propeller endurance optimum)
%
% CALLS:   StdAtmos(h)  returns [P, rho, T, mu] for scalar h [m]
%
% OUTPUTS:
%   Command window — outbound + inbound weight statements, fuel fractions,
%                    distances, timing, total mission time
%   Figure 1       — Altitude & TAS vs mission time (outbound + inbound,
%                    separated by turnaround gap), colored by distance
%   Figure 2       — Throttle vs time: 6 subplots (climb/cruise/descent × 2 legs)
%
% FLAG INDEX:
%   FLAG-M1  Payload differences between legs — inbound heavier (patient added)
%   FLAG-M2  Cruise L/D = LD_BE; no speed constraint, best-endurance optimum
%   FLAG-M3  Loiter reserves excluded from mission time per RFP definition
%   FLAG-M4  Both legs carry independent loiter fuel; inbound reserve = outbound 6a-6c
%
% AUTHORS: PUDO 670 Design Team
% VERSION: 1.0  |  May 2026
% =========================================================================

clear; clc; close all;


%% =========================================================================
%  SECTION 1 — AIRCRAFT DESIGN CONSTANTS
%  Identical to ReferenceMission_v1.m. Update here if aircraft changes.
% =========================================================================

g      = 9.81;           % gravitational acceleration                [m/s^2]
rho_SL = 1.225;          % ISA sea-level air density                 [kg/m^3]

% --- Wing & Aerodynamics -------------------------------------------------
S   = 37.1;              % reference wing area                       [m^2]
AR  = 6.76;              % aspect ratio                              [-]
e   = 0.90;              % Oswald efficiency factor (clean)          [-]
CD0 = 0.0288;            % parasite drag, clean config (all phases)  [-]
k   = 1 / (pi * AR * e); % induced drag factor                       [-]

% --- Propulsion: PT6A-60A ------------------------------------------------
P_shp  = 1050;                         % rated shaft power           [shp]
P_SL_W = P_shp * 745.7;               % sea-level shaft power        [W]
eta_p  = 0.80;                         % propeller efficiency          [-]
P_A_SL = eta_p * P_SL_W;              % sea-level propulsive power    [W]

SFCp_imp = 0.548;                                   % [lb/(hp*hr)]
SFCp_SI  = SFCp_imp * 0.4536 / (745.7 * 3600);     % [kg/(W*s)]

% --- Best-endurance aerodynamics (CRUISE for this mission + loiter) ------
%   Propeller best-endurance: CL_BE = sqrt(3*CD0/k), CD_BE = 4*CD0
%   FLAG-M2: cruise and loiter share the same CL_BE / CD_BE.
CL_BE = sqrt(3 * CD0 / k);   % best-endurance / best-loiter CL      [-]
CD_BE = 4 * CD0;              % best-endurance CD                     [-]
LD_BE = CL_BE / CD_BE;        % best-endurance L/D                   [-]

if CL_BE > 2.4
    warning('PUDO670:MedevacLoiterStall', ...
        'CL_BE = %.4f exceeds clean CL_max = 2.4. Clamping.', CL_BE);
    CL_BE = 2.4;
    CD_BE = CD0 + k * CL_BE^2;
    LD_BE = CL_BE / CD_BE;
end

%   Breguet endurance constant K_loiter [N^0.5] — sea-level loiter
K_loiter = (eta_p / (g * SFCp_SI)) * (CL_BE^1.5 / CD_BE) * sqrt(2 * rho_SL * S);

% --- Cruise altitude (best-endurance cruise) -----------------------------
%   RFP: no speed/altitude requirement. Cruise at best-endurance speed.
%   V_BE(h,W) = (4k/(3*CD0))^(1/4) * sqrt(2W/(rho(h)*S))
%   We pick h_cruise = 2000 m as a conservative altitude where V_BE stays
%   above stall for the expected weight range (≈ 3200–3900 kg).
%   L/D at cruise = LD_BE (by definition of best-endurance condition).
h_cruise  = 2000.0;   % cruise altitude                              [m]
LD_cruise = LD_BE;    % cruise L/D = best-endurance L/D              [-]
%   V_BE computed at midpoint weight after convergence (see Section 5 output).
%   For throttle profiles we need a representative cruise TAS — computed
%   per-segment after iteration converges.


%% =========================================================================
%  SECTION 2 — MISSION PARAMETERS AND PAYLOAD
% =========================================================================

% --- Range per leg -------------------------------------------------------
R_leg_nmi = 350;                  % RFP minimum range per leg         [nmi]
R_leg_m   = R_leg_nmi * 1852;    %                                   [m]

% --- Raymer historical fuel fractions (same as reference mission) --------
ff_warmup  = 0.990;   % warm-up and taxi
ff_takeoff = 0.995;   % STOL takeoff over 50 ft obstacle
ff_climb   = 0.980;   % climb — Raymer historical, propeller aircraft
ff_descent = 0.990;   % descent — Raymer historical
ff_landing = 0.992;   % landing rollout and taxi-in

% --- OEW (fixed airframe property) --------------------------------------
%   Identical to ReferenceMission_v1.m — FLAG-M1.
W_OEW_kg  = 2150.0;              % OEW (structural + 2 pilots)       [kg]
W_crew_kg = 2 * 200 * 0.4536;   % 2 pilots bookkeeping              [kg]

% --- Configuration 3 payload breakdown ----------------------------------
%   OUTBOUND (segments 3-5): carry full medical kit + stretcher + 2 med techs
%     No patient or escort on the way OUT (picking up patient).
%     Per RFP: "outbound flight is flown with a full suite of medical
%     equipment and stretcher on board, as well as two medical technicians."
W_med_tech_kg  = 2 * 90.7;       % 2 medical technicians (200 lb ea) [kg]
W_med_equip_kg = 136.1;          % medical supplies (300 lb)          [kg]
W_stretcher_kg = 22.7;           % litter (50 lb)                    [kg]
W_pld_OB_kg    = W_med_tech_kg + W_med_equip_kg + W_stretcher_kg;
%   Outbound payload = 2 med techs + supplies + stretcher (no patient yet)

%   INBOUND (segments 10-12): patient + escort added.
%     Per RFP: "inbound flight is flown with a full suite of medical
%     equipment and stretcher on board, as well as two medical technicians,
%     a patient, and a patient escort."
W_patient_kg   = 90.7;           % patient (200 lb)                  [kg]
W_escort_kg    = 90.7;           % patient escort (200 lb)            [kg]
W_pld_IB_kg    = W_pld_OB_kg + W_patient_kg + W_escort_kg;
%   Inbound payload = outbound payload + patient + escort

% --- Ground timing -------------------------------------------------------
t_taxi_min     = 10.0;           % warm-up + taxi each end           [min]
t_turnaround_min = 45.0;         % patient loading turnaround        [min]
t_shutdown_min = 10.0;           % taxi + shutdown each end          [min]

% --- Loiter duration -----------------------------------------------------
t_loiter_s = 45 * 60;            % 45-min loiter reserve             [s]

% --- Climb simulation parameters -----------------------------------------
V_climb  = 70.0;    % fixed climb TAS (team decision)               [m/s]
dt_climb = 1.0;     % kinematic climb sim time step                 [s]

% --- Descent geometry (same convention as reference mission) -------------
ROD_fpm     = 1000;                       % rate of descent          [ft/min]
ROD_ms      = ROD_fpm * 0.3048 / 60;     % rate of descent          [m/s]
V_horiz_req = 67.0;                       % required horiz speed     [m/s]
V_descent   = sqrt(V_horiz_req^2 + ROD_ms^2);  % glide TAS          [m/s]
gamma_desc  = atan2(ROD_ms, V_horiz_req); % descent angle, nose down [rad]

t_desc_s    = h_cruise / ROD_ms;          % descent time             [s]
t_desc_min  = t_desc_s / 60;             %                           [min]
d_desc_m    = V_horiz_req * t_desc_s;    % horizontal descent dist   [m]

% --- Fuel fraction seed (from Config 2 reference) -----------------------
zeta_seed = 0.1313;


%% =========================================================================
%  SECTION 3 — LOITER RESERVE TARGETS (one per leg)
%
%  Each leg must carry fuel to sustain 45 min loiter after landing.
%  W_dry_OB = OEW + outbound payload  (no patient aboard on the way out)
%  W_dry_IB = OEW + inbound payload   (patient + escort added)
%  W6_target = weight after landing rollout = weight needed to start loiter
%
%  Breguet endurance (W in Newtons):
%    E = K_loiter * [W_dry^(-0.5) - W6^(-0.5)]
%  Solve for W6:
%    W6 = (W_dry^(-0.5) - E/K_loiter)^(-2)
% =========================================================================

% Outbound
W_dry_OB_kg     = W_OEW_kg + W_pld_OB_kg;
W_dry_OB_N      = W_dry_OB_kg * g;
W6_tgt_OB_N     = 1 / (W_dry_OB_N^(-0.5) - t_loiter_s / K_loiter)^2;
W6_tgt_OB_kg    = W6_tgt_OB_N / g;
W_loit_fuel_OB  = W6_tgt_OB_kg - W_dry_OB_kg;   % loiter reserve fuel [kg]
ff_loiter_OB    = W_dry_OB_N / W6_tgt_OB_N;

% Inbound
W_dry_IB_kg     = W_OEW_kg + W_pld_IB_kg;
W_dry_IB_N      = W_dry_IB_kg * g;
W6_tgt_IB_N     = 1 / (W_dry_IB_N^(-0.5) - t_loiter_s / K_loiter)^2;
W6_tgt_IB_kg    = W6_tgt_IB_N / g;
W_loit_fuel_IB  = W6_tgt_IB_kg - W_dry_IB_kg;   % loiter reserve fuel [kg]
ff_loiter_IB    = W_dry_IB_N / W6_tgt_IB_N;


%% =========================================================================
%  SECTION 4 — PRINT HEADER AND CONSTANTS
% =========================================================================

fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║       PUDO 670  —  MEDEVAC MISSION ANALYSIS                         ║\n');
fprintf('║       Config 3  |  Round-Trip  |  350 nmi per leg                   ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  AIRCRAFT CONSTANTS                                                  ║\n');
fprintf('║  %-40s : %8.2f m^2           ║\n', 'Wing area S', S);
fprintf('║  %-40s : %8.4f               ║\n', 'CD0 (all phases)', CD0);
fprintf('║  %-40s : %8.4f               ║\n', 'k (induced drag factor)', k);
fprintf('║  %-40s : %8.4f               ║\n', 'CL_BE (best endurance)', CL_BE);
fprintf('║  %-40s : %8.4f               ║\n', 'CD_BE', CD_BE);
fprintf('║  %-40s : %8.4f               ║\n', 'L/D at best endurance (cruise)', LD_BE);
fprintf('║  %-40s : %8.0f m  (%5.0f ft)║\n', 'Cruise altitude h_cr', h_cruise, h_cruise/0.3048);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  DESCENT GEOMETRY (both legs identical)                              ║\n');
fprintf('║  %-40s : %8.4f m/s (%.0f fpm)  ║\n', 'Rate of descent', ROD_ms, ROD_fpm);
fprintf('║  %-40s : %8.4f m/s           ║\n', 'Glide speed TAS', V_descent);
fprintf('║  %-40s : %8.4f deg           ║\n', 'Descent angle', rad2deg(gamma_desc));
fprintf('║  %-40s : %8.2f s  (%.2f min)║\n', 'Time to descend', t_desc_s, t_desc_min);
fprintf('║  %-40s : %8.2f km (%.2f nmi)║\n', 'Horiz distance descent', d_desc_m/1e3, d_desc_m/1852);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  PAYLOAD BREAKDOWN                                                   ║\n');
fprintf('║  %-40s : %8.2f kg           ║\n', 'OEW (structural + crew)', W_OEW_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'Outbound payload (OB)', W_pld_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  2 med techs', W_med_tech_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  medical supplies', W_med_equip_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  stretcher', W_stretcher_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'Inbound payload (IB = OB + patient)', W_pld_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  + patient', W_patient_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  + patient escort', W_escort_kg);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  LOITER RESERVE PARAMETERS                                           ║\n');
fprintf('║  %-40s : %8.4e [N^0.5]   ║\n', 'K_loiter', K_loiter);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W_dry_OB (OEW+pld_OB)', W_dry_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W6_target_OB (after OB landing)', W6_tgt_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'OB loiter fuel reserve', W_loit_fuel_OB);
fprintf('║  %-40s : %8.6f             ║\n', 'ff_loiter_OB', ff_loiter_OB);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W_dry_IB (OEW+pld_IB)', W_dry_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W6_target_IB (after IB landing)', W6_tgt_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'IB loiter fuel reserve', W_loit_fuel_IB);
fprintf('║  %-40s : %8.6f             ║\n', 'ff_loiter_IB', ff_loiter_IB);


%% =========================================================================
%  SECTION 5 — FUEL ITERATION: OUTBOUND LEG
%
%  Identical algorithm to ReferenceMission_v1.m Section 5, but:
%    - W_dry = W_dry_OB_kg
%    - W6_target = W6_tgt_OB_kg
%    - LD_cruise = LD_BE (best-endurance cruise)
%    - R_total = R_leg_m (350 nmi)
%  The kinematic climb sim uses W2_N = W2_kg*g (weight at base of climb).
% =========================================================================

tol_kg   = 0.01;
max_iter = 100;

W_fuel_OB_kg = zeta_seed * W_dry_OB_kg / (1 - zeta_seed);

fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  OUTBOUND FUEL ITERATION  (tol=%.3f kg, init=%.2f kg)           ║\n', tol_kg, W_fuel_OB_kg);
fprintf('║  %4s | %9s | %9s | %9s | %9s | %9s  ║\n', 'Iter','W_fuel','W0','W6_comp','W6_tgt','Resid');
fprintf('║  %s  ║\n', repmat('-',1,64));

converged_OB = false;
residual_OB  = NaN;
W0_OB_kg=0; W1_OB_kg=0; W2_OB_kg=0; W3_OB_kg=0;
W4_OB_kg=0; W5_OB_kg=0; W6_OB_kg=0;
d_climb_OB_m=0; t_climb_OB_s=0;
ROC_avg_OB_ms=0; ROC_avg_OB_fpm=0; t_climb_OB_min=0;
d_cruise_OB_m=0; d_cruise_OB_km=0; d_cruise_OB_nmi=0;
ff_cruise_OB=1; Vh_min_OB=inf;

for iter = 1:max_iter

    W0_OB_kg = W_dry_OB_kg + W_fuel_OB_kg;
    W1_OB_kg = W0_OB_kg * ff_warmup;
    W2_OB_kg = W1_OB_kg * ff_takeoff;
    W2_OB_N  = W2_OB_kg * g;

    % Kinematic climb simulation
    h_c = 0.0; d_climb_OB_m = 0.0; t_climb_OB_s = 0.0;
    ROC_sum = 0.0; n_steps = 0; Vh_min_OB = inf;

    while h_c < h_cruise
        [~, rho_c, ~, ~] = StdAtmos(h_c);
        sigma_c = rho_c / rho_SL;
        P_A_c   = sigma_c * P_A_SL;
        T_c     = P_A_c / V_climb;
        q_c     = 0.5 * rho_c * V_climb^2;
        CL_c    = W2_OB_N / (q_c * S);
        CD_c    = CD0 + k * CL_c^2;
        D_c     = q_c * S * CD_c;
        ROC_c   = V_climb * (T_c - D_c) / W2_OB_N;
        if ROC_c <= 0
            warning('PUDO670:MedevacOBCeiling','OB: ROC<=0 at h=%.1f m.',h_c);
            break;
        end
        gamma_c     = asin(min(ROC_c / V_climb, 1.0));
        V_horiz_c   = V_climb * cos(gamma_c);
        Vh_min_OB   = min(Vh_min_OB, V_horiz_c);
        dt_eff      = min(dt_climb, (h_cruise - h_c) / ROC_c);
        d_climb_OB_m = d_climb_OB_m + V_horiz_c * dt_eff;
        t_climb_OB_s = t_climb_OB_s + dt_eff;
        h_c          = h_c + ROC_c * dt_eff;
        ROC_sum      = ROC_sum + ROC_c;
        n_steps      = n_steps + 1;
    end

    ROC_avg_OB_ms  = ROC_sum / max(n_steps,1);
    ROC_avg_OB_fpm = ROC_avg_OB_ms * 196.85;
    t_climb_OB_min = t_climb_OB_s / 60;

    d_cruise_OB_m   = R_leg_m - d_climb_OB_m - d_desc_m;
    d_cruise_OB_km  = d_cruise_OB_m / 1e3;
    d_cruise_OB_nmi = d_cruise_OB_m / 1852;

    if d_cruise_OB_m <= 0
        error('PUDO670:MedevacOBRange','OB cruise range = %.1f m (negative).', d_cruise_OB_m);
    end

    ff_cruise_OB = exp(-d_cruise_OB_m * g * SFCp_SI / (eta_p * LD_cruise));

    W3_OB_kg = W2_OB_kg * ff_climb;
    W4_OB_kg = W3_OB_kg * ff_cruise_OB;
    W5_OB_kg = W4_OB_kg * ff_descent;
    W6_OB_kg = W5_OB_kg * ff_landing;

    residual_OB = W6_OB_kg - W6_tgt_OB_kg;

    fprintf('║  %4d | %9.3f | %9.3f | %9.3f | %9.3f | %9.4f  ║\n', ...
        iter, W_fuel_OB_kg, W0_OB_kg, W6_OB_kg, W6_tgt_OB_kg, residual_OB);

    if abs(residual_OB) < tol_kg
        converged_OB = true;
        break;
    end

    ff_prod_OB = ff_warmup * ff_takeoff * ff_climb * ff_cruise_OB * ff_descent * ff_landing;
    W_fuel_OB_kg = W_fuel_OB_kg - residual_OB / ff_prod_OB;
end

if ~converged_OB
    warning('PUDO670:MedevacOBNoConv','OB fuel iteration did not converge. Residual=%.4f kg', residual_OB);
end

% Outbound timing
t_cruise_OB_s   = d_cruise_OB_m / (W3_OB_kg/W3_OB_kg);  % placeholder — compute below
[~, rho_cr, ~, ~] = StdAtmos(h_cruise);
W_mid_OB_N     = 0.5 * (W3_OB_kg + W4_OB_kg) * g;
V_BE_OB        = (4*k / (3*CD0))^(0.25) * sqrt(2*W_mid_OB_N / (rho_cr * S));
t_cruise_OB_s  = d_cruise_OB_m / V_BE_OB;
t_cruise_OB_min = t_cruise_OB_s / 60;
t_air_OB_min   = t_climb_OB_min + t_cruise_OB_min + t_desc_min;  % excl. loiter


%% =========================================================================
%  SECTION 6 — FUEL ITERATION: INBOUND LEG
%
%  Same algorithm, but W_dry = W_dry_IB_kg, W6_target = W6_tgt_IB_kg.
%  Inbound range equals outbound range (RFP: "range of IB = range of OB").
% =========================================================================

W_fuel_IB_kg = zeta_seed * W_dry_IB_kg / (1 - zeta_seed);

fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  INBOUND FUEL ITERATION   (tol=%.3f kg, init=%.2f kg)           ║\n', tol_kg, W_fuel_IB_kg);
fprintf('║  %4s | %9s | %9s | %9s | %9s | %9s  ║\n', 'Iter','W_fuel','W0','W6_comp','W6_tgt','Resid');
fprintf('║  %s  ║\n', repmat('-',1,64));

converged_IB = false;
residual_IB  = NaN;
W0_IB_kg=0; W1_IB_kg=0; W2_IB_kg=0; W3_IB_kg=0;
W4_IB_kg=0; W5_IB_kg=0; W6_IB_kg=0;
d_climb_IB_m=0; t_climb_IB_s=0;
ROC_avg_IB_ms=0; ROC_avg_IB_fpm=0; t_climb_IB_min=0;
d_cruise_IB_m=0; d_cruise_IB_km=0; d_cruise_IB_nmi=0;
ff_cruise_IB=1; Vh_min_IB=inf;

for iter = 1:max_iter

    W0_IB_kg = W_dry_IB_kg + W_fuel_IB_kg;
    W1_IB_kg = W0_IB_kg * ff_warmup;
    W2_IB_kg = W1_IB_kg * ff_takeoff;
    W2_IB_N  = W2_IB_kg * g;

    % Kinematic climb simulation
    h_c = 0.0; d_climb_IB_m = 0.0; t_climb_IB_s = 0.0;
    ROC_sum = 0.0; n_steps = 0; Vh_min_IB = inf;

    while h_c < h_cruise
        [~, rho_c, ~, ~] = StdAtmos(h_c);
        sigma_c = rho_c / rho_SL;
        P_A_c   = sigma_c * P_A_SL;
        T_c     = P_A_c / V_climb;
        q_c     = 0.5 * rho_c * V_climb^2;
        CL_c    = W2_IB_N / (q_c * S);
        CD_c    = CD0 + k * CL_c^2;
        D_c     = q_c * S * CD_c;
        ROC_c   = V_climb * (T_c - D_c) / W2_IB_N;
        if ROC_c <= 0
            warning('PUDO670:MedevacIBCeiling','IB: ROC<=0 at h=%.1f m.',h_c);
            break;
        end
        gamma_c     = asin(min(ROC_c / V_climb, 1.0));
        V_horiz_c   = V_climb * cos(gamma_c);
        Vh_min_IB   = min(Vh_min_IB, V_horiz_c);
        dt_eff      = min(dt_climb, (h_cruise - h_c) / ROC_c);
        d_climb_IB_m = d_climb_IB_m + V_horiz_c * dt_eff;
        t_climb_IB_s = t_climb_IB_s + dt_eff;
        h_c          = h_c + ROC_c * dt_eff;
        ROC_sum      = ROC_sum + ROC_c;
        n_steps      = n_steps + 1;
    end

    ROC_avg_IB_ms  = ROC_sum / max(n_steps,1);
    ROC_avg_IB_fpm = ROC_avg_IB_ms * 196.85;
    t_climb_IB_min = t_climb_IB_s / 60;

    d_cruise_IB_m   = R_leg_m - d_climb_IB_m - d_desc_m;
    d_cruise_IB_km  = d_cruise_IB_m / 1e3;
    d_cruise_IB_nmi = d_cruise_IB_m / 1852;

    if d_cruise_IB_m <= 0
        error('PUDO670:MedevacIBRange','IB cruise range = %.1f m (negative).', d_cruise_IB_m);
    end

    ff_cruise_IB = exp(-d_cruise_IB_m * g * SFCp_SI / (eta_p * LD_cruise));

    W3_IB_kg = W2_IB_kg * ff_climb;
    W4_IB_kg = W3_IB_kg * ff_cruise_IB;
    W5_IB_kg = W4_IB_kg * ff_descent;
    W6_IB_kg = W5_IB_kg * ff_landing;

    residual_IB = W6_IB_kg - W6_tgt_IB_kg;

    fprintf('║  %4d | %9.3f | %9.3f | %9.3f | %9.3f | %9.4f  ║\n', ...
        iter, W_fuel_IB_kg, W0_IB_kg, W6_IB_kg, W6_tgt_IB_kg, residual_IB);

    if abs(residual_IB) < tol_kg
        converged_IB = true;
        break;
    end

    ff_prod_IB = ff_warmup * ff_takeoff * ff_climb * ff_cruise_IB * ff_descent * ff_landing;
    W_fuel_IB_kg = W_fuel_IB_kg - residual_IB / ff_prod_IB;
end

if ~converged_IB
    warning('PUDO670:MedevacIBNoConv','IB fuel iteration did not converge. Residual=%.4f kg', residual_IB);
end

% Inbound timing and cruise speed
W_mid_IB_N      = 0.5 * (W3_IB_kg + W4_IB_kg) * g;
V_BE_IB         = (4*k / (3*CD0))^(0.25) * sqrt(2*W_mid_IB_N / (rho_cr * S));
t_cruise_IB_s   = d_cruise_IB_m / V_BE_IB;
t_cruise_IB_min = t_cruise_IB_s / 60;
t_air_IB_min    = t_climb_IB_min + t_cruise_IB_min + t_desc_min;  % excl. loiter


%% =========================================================================
%  SECTION 7 — MISSION SUMMARY (command window)
% =========================================================================

% Total mission time (FLAG-M3: loiter segments excluded per RFP)
t_ground_OB_min = t_taxi_min + t_shutdown_min;        % segments 1+7 = 20 min
t_ground_IB_min = t_taxi_min + t_shutdown_min;        % segments 8+14 = 20 min
t_total_min = t_ground_OB_min + t_air_OB_min ...
            + t_turnaround_min ...
            + t_ground_IB_min + t_air_IB_min;

W_fuel_total_kg = W_fuel_OB_kg + W_fuel_IB_kg;
W0_OB_final     = W0_OB_kg;
W0_IB_final     = W0_IB_kg;
W0_max          = max(W0_OB_final, W0_IB_final);

fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  CONVERGED MISSION SUMMARY                                           ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  OUTBOUND LEG WEIGHT BREAKDOWN                                       ║\n');
fprintf('║  %-40s : %8.2f kg           ║\n', 'OEW (structural + crew)', W_OEW_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'Outbound payload', W_pld_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'OB fuel (incl. loiter reserve)', W_fuel_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  of which: loiter reserve', W_loit_fuel_OB);
fprintf('║  %-40s : %8.2f kg  (%6.0f N)║\n', 'OB MTOW (W0_OB)', W0_OB_final, W0_OB_final*g);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  OUTBOUND SEGMENT WEIGHTS                                            ║\n');
fprintf('║  %-40s : %8.2f kg           ║\n', 'W0  Takeoff', W0_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W1  After warm-up/taxi', W1_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W2  After takeoff (base of climb)', W2_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W3  After climb', W3_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W4  After cruise', W4_OB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W5  After descent', W5_OB_kg);
fprintf('║  %-40s : %8.2f kg  (=W6_tgt)║\n', 'W6  After landing rollout', W6_OB_kg);
fprintf('║  %-40s : %8.2f kg  (zero fuel)║\n','W7  After 45-min loiter', W_dry_OB_kg);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  OUTBOUND FUEL FRACTIONS                                             ║\n');
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_warmup',   ff_warmup);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_takeoff',  ff_takeoff);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_climb',    ff_climb);
fprintf('║  %-40s : %9.6f  (Breguet) ║\n', 'ff_cruise_OB',ff_cruise_OB);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_descent',  ff_descent);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_landing',  ff_landing);
fprintf('║  %-40s : %9.6f  (Breguet E)║\n','ff_loiter_OB',ff_loiter_OB);
fprintf('║  %-40s : %9.6f             ║\n', 'zeta_OB = W_fuel_OB/W0_OB', W_fuel_OB_kg/W0_OB_final);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  OUTBOUND DISTANCES & TIMING                                         ║\n');
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)║\n', 'Climb distance', d_climb_OB_m/1e3, d_climb_OB_m/1852);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)║\n', 'Cruise distance', d_cruise_OB_km, d_cruise_OB_nmi);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)║\n', 'Descent distance', d_desc_m/1e3, d_desc_m/1852);
fprintf('║  %-40s : %8.2f km  (%6.0f nmi)║\n', 'Total OB ground track', R_leg_m/1e3, R_leg_nmi);
fprintf('║  %-40s : %8.2f m/s            ║\n', 'V_BE cruise speed OB (mid-cruise)', V_BE_OB);
fprintf('║  %-40s : %8.2f min            ║\n', 'Climb time OB', t_climb_OB_min);
fprintf('║  %-40s : %8.2f min            ║\n', 'Cruise time OB', t_cruise_OB_min);
fprintf('║  %-40s : %8.2f min            ║\n', 'Descent time OB', t_desc_min);
fprintf('║  %-40s : %8.2f min  (excl. loiter)║\n','OB air time', t_air_OB_min);
if Vh_min_OB >= V_horiz_req
    fprintf('║  CHECK V_horiz OB: PASS  (>= 67 m/s throughout climb)            ║\n');
else
    fprintf('║  CHECK V_horiz OB: FAIL  (< 67 m/s — increase V_climb!)          ║\n');
end
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  INBOUND LEG WEIGHT BREAKDOWN                                        ║\n');
fprintf('║  %-40s : %8.2f kg           ║\n', 'OEW (structural + crew)', W_OEW_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'Inbound payload (+ patient + escort)', W_pld_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'IB fuel (incl. loiter reserve)', W_fuel_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', '  of which: loiter reserve', W_loit_fuel_IB);
fprintf('║  %-40s : %8.2f kg  (%6.0f N)║\n', 'IB MTOW (W0_IB)', W0_IB_final, W0_IB_final*g);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  INBOUND SEGMENT WEIGHTS                                             ║\n');
fprintf('║  %-40s : %8.2f kg           ║\n', 'W0  Takeoff', W0_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W1  After warm-up/taxi', W1_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W2  After takeoff (base of climb)', W2_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W3  After climb', W3_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W4  After cruise', W4_IB_kg);
fprintf('║  %-40s : %8.2f kg           ║\n', 'W5  After descent', W5_IB_kg);
fprintf('║  %-40s : %8.2f kg  (=W6_tgt)║\n', 'W6  After landing rollout', W6_IB_kg);
fprintf('║  %-40s : %8.2f kg  (zero fuel)║\n','W7  After 45-min loiter', W_dry_IB_kg);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  INBOUND FUEL FRACTIONS                                              ║\n');
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_warmup',   ff_warmup);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_takeoff',  ff_takeoff);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_climb',    ff_climb);
fprintf('║  %-40s : %9.6f  (Breguet) ║\n', 'ff_cruise_IB',ff_cruise_IB);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_descent',  ff_descent);
fprintf('║  %-40s : %9.6f  (Raymer)  ║\n', 'ff_landing',  ff_landing);
fprintf('║  %-40s : %9.6f  (Breguet E)║\n','ff_loiter_IB',ff_loiter_IB);
fprintf('║  %-40s : %9.6f             ║\n', 'zeta_IB = W_fuel_IB/W0_IB', W_fuel_IB_kg/W0_IB_final);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  INBOUND DISTANCES & TIMING                                          ║\n');
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)║\n', 'Climb distance', d_climb_IB_m/1e3, d_climb_IB_m/1852);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)║\n', 'Cruise distance', d_cruise_IB_km, d_cruise_IB_nmi);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)║\n', 'Descent distance', d_desc_m/1e3, d_desc_m/1852);
fprintf('║  %-40s : %8.2f km  (%6.0f nmi)║\n', 'Total IB ground track', R_leg_m/1e3, R_leg_nmi);
fprintf('║  %-40s : %8.2f m/s            ║\n', 'V_BE cruise speed IB (mid-cruise)', V_BE_IB);
fprintf('║  %-40s : %8.2f min            ║\n', 'Climb time IB', t_climb_IB_min);
fprintf('║  %-40s : %8.2f min            ║\n', 'Cruise time IB', t_cruise_IB_min);
fprintf('║  %-40s : %8.2f min            ║\n', 'Descent time IB', t_desc_min);
fprintf('║  %-40s : %8.2f min  (excl. loiter)║\n','IB air time', t_air_IB_min);
if Vh_min_IB >= V_horiz_req
    fprintf('║  CHECK V_horiz IB: PASS  (>= 67 m/s throughout climb)            ║\n');
else
    fprintf('║  CHECK V_horiz IB: FAIL  (< 67 m/s — increase V_climb!)          ║\n');
end
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  TOTAL ROUND-TRIP SUMMARY                                            ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  %-40s : %8.2f kg           ║\n', 'Total fuel (OB + IB)', W_fuel_total_kg);
fprintf('║  %-40s : %8.2f kg  (%6.0f N)║\n', 'Max MTOW (governing leg)', W0_max, W0_max*g);
fprintf('║  %-40s : %8.2f kg  (%6.0f N)║\n', 'Config 2 sizing MTOW (ref)', 4077.0, 4077.0*g);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  TOTAL MISSION TIME BUDGET  (FLAG-M3: loiter excl. per RFP)         ║\n');
fprintf('║  %-40s : %8.2f min           ║\n', 'OB ground ops (seg 1+7)', t_ground_OB_min);
fprintf('║  %-40s : %8.2f min           ║\n', 'OB air time (seg 3-5)', t_air_OB_min);
fprintf('║  %-40s : %8.2f min           ║\n', 'Turnaround (patient loading)', t_turnaround_min);
fprintf('║  %-40s : %8.2f min           ║\n', 'IB ground ops (seg 8+14)', t_ground_IB_min);
fprintf('║  %-40s : %8.2f min           ║\n', 'IB air time (seg 10-12)', t_air_IB_min);
fprintf('║  %-40s : %8.2f min  (%5.2f hr)║\n', 'TOTAL MISSION TIME', t_total_min, t_total_min/60);
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n\n');


%% =========================================================================
%  SECTION 8 — FINAL CLIMB SIMULATIONS (store vectors for plotting)
%  Outbound and Inbound climbs re-run with converged weights, saving all steps.
% =========================================================================

% --- Outbound climb -------------------------------------------------------
W2_OB_N_final = W2_OB_kg * g;
max_pts = 7000;
t_clmb_OB_vec  = zeros(1, max_pts);
h_clmb_OB_vec  = zeros(1, max_pts);
dx_clmb_OB_vec = zeros(1, max_pts);
V_clmb_OB_vec  = V_climb * ones(1, max_pts);

h_c = 0.0; d_c = 0.0; t_c = 0.0; step = 1;
t_clmb_OB_vec(1) = 0; h_clmb_OB_vec(1) = 0; dx_clmb_OB_vec(1) = 0;

while h_c < h_cruise
    [~, rho_c, ~, ~] = StdAtmos(h_c);
    sigma_c = rho_c / rho_SL;
    P_A_c   = sigma_c * P_A_SL;
    T_c     = P_A_c / V_climb;
    q_c     = 0.5 * rho_c * V_climb^2;
    CL_c    = W2_OB_N_final / (q_c * S);
    CD_c    = CD0 + k * CL_c^2;
    D_c     = q_c * S * CD_c;
    ROC_c   = V_climb * (T_c - D_c) / W2_OB_N_final;
    if ROC_c <= 0, break; end
    gamma_c = asin(min(ROC_c / V_climb, 1.0));
    dt_eff  = min(dt_climb, (h_cruise - h_c) / ROC_c);
    d_c = d_c + V_climb * cos(gamma_c) * dt_eff;
    t_c = t_c + dt_eff;
    h_c = h_c + ROC_c * dt_eff;
    step = step + 1;
    if step > max_pts, break; end
    t_clmb_OB_vec(step)  = t_c;
    h_clmb_OB_vec(step)  = h_c;
    dx_clmb_OB_vec(step) = d_c;
end
n_clmb_OB    = step;
t_clmb_OB    = t_clmb_OB_vec(1:n_clmb_OB) / 60;    % [min]
h_clmb_OB    = h_clmb_OB_vec(1:n_clmb_OB);           % [m]
dx_clmb_OB   = dx_clmb_OB_vec(1:n_clmb_OB);          % [m]
V_clmb_OB    = V_clmb_OB_vec(1:n_clmb_OB);           % [m/s]

% --- Inbound climb --------------------------------------------------------
W2_IB_N_final = W2_IB_kg * g;
t_clmb_IB_vec  = zeros(1, max_pts);
h_clmb_IB_vec  = zeros(1, max_pts);
dx_clmb_IB_vec = zeros(1, max_pts);
V_clmb_IB_vec  = V_climb * ones(1, max_pts);

h_c = 0.0; d_c = 0.0; t_c = 0.0; step = 1;
t_clmb_IB_vec(1) = 0; h_clmb_IB_vec(1) = 0; dx_clmb_IB_vec(1) = 0;

while h_c < h_cruise
    [~, rho_c, ~, ~] = StdAtmos(h_c);
    sigma_c = rho_c / rho_SL;
    P_A_c   = sigma_c * P_A_SL;
    T_c     = P_A_c / V_climb;
    q_c     = 0.5 * rho_c * V_climb^2;
    CL_c    = W2_IB_N_final / (q_c * S);
    CD_c    = CD0 + k * CL_c^2;
    D_c     = q_c * S * CD_c;
    ROC_c   = V_climb * (T_c - D_c) / W2_IB_N_final;
    if ROC_c <= 0, break; end
    gamma_c = asin(min(ROC_c / V_climb, 1.0));
    dt_eff  = min(dt_climb, (h_cruise - h_c) / ROC_c);
    d_c = d_c + V_climb * cos(gamma_c) * dt_eff;
    t_c = t_c + dt_eff;
    h_c = h_c + ROC_c * dt_eff;
    step = step + 1;
    if step > max_pts, break; end
    t_clmb_IB_vec(step)  = t_c;
    h_clmb_IB_vec(step)  = h_c;
    dx_clmb_IB_vec(step) = d_c;
end
n_clmb_IB    = step;
t_clmb_IB    = t_clmb_IB_vec(1:n_clmb_IB) / 60;    % [min]
h_clmb_IB    = h_clmb_IB_vec(1:n_clmb_IB);           % [m]
dx_clmb_IB   = dx_clmb_IB_vec(1:n_clmb_IB);          % [m]
V_clmb_IB    = V_clmb_IB_vec(1:n_clmb_IB);           % [m/s]


%% =========================================================================
%  SECTION 9 — THROTTLE PROFILES
%
%  OB climb  : 100% throughout
%  OB cruise : T = D at V_BE; weight drops W3_OB → W4_OB; rho varies with W
%              At best-endurance speed: V_BE(W) = (4k/3CD0)^0.25*sqrt(2W/(rho*S))
%              Speed varies with weight so we compute T at each weight step.
%  OB descent: along-path force balance (same as reference)
%  IB: repeat for inbound weights
% =========================================================================

n_pts = 300;

% --- OB Climb throttle ---------------------------------------------------
thr_clmb_OB_pct = 100 * ones(1, n_clmb_OB);

% --- OB Cruise throttle --------------------------------------------------
t_cr_OB_rel = linspace(0, t_cruise_OB_min, n_pts);
W_cr_OB_N   = linspace(W3_OB_kg, W4_OB_kg, n_pts) * g;
thr_cr_OB_pct = zeros(1, n_pts);
for i = 1:n_pts
    V_BE_i  = (4*k / (3*CD0))^(0.25) * sqrt(2*W_cr_OB_N(i) / (rho_cr * S));
    q_i     = 0.5 * rho_cr * V_BE_i^2;
    CL_i    = W_cr_OB_N(i) / (q_i * S);
    CD_i    = CD0 + k * CL_i^2;
    T_i     = q_i * S * CD_i;            % T = D for level cruise
    sigma_cr = rho_cr / rho_SL;
    P_A_i   = sigma_cr * P_A_SL;
    thr_cr_OB_pct(i) = 100 * (T_i * V_BE_i) / P_A_i;
end

% --- OB Descent throttle -------------------------------------------------
n_dc = n_pts;
h_dc_vec    = linspace(h_cruise, 0, n_dc);
t_dc_OB_rel = linspace(0, t_desc_min, n_dc);
W_dc_OB_N   = linspace(W5_OB_kg, W6_OB_kg, n_dc) * g;
thr_dc_OB_pct = zeros(1, n_dc);
for i = 1:n_dc
    [~, rho_d, ~, ~] = StdAtmos(h_dc_vec(i));
    sigma_d = rho_d / rho_SL;
    P_A_d   = sigma_d * P_A_SL;
    q_d     = 0.5 * rho_d * V_descent^2;
    CL_d    = W_dc_OB_N(i) / (q_d * S);
    CD_d    = CD0 + k * CL_d^2;
    D_d     = q_d * S * CD_d;
    T_d     = D_d - W_dc_OB_N(i) * sin(gamma_desc);
    if T_d > 0
        thr_dc_OB_pct(i) = 100 * (T_d * V_descent) / P_A_d;
    else
        thr_dc_OB_pct(i) = 0;
    end
end

% --- IB Climb throttle ---------------------------------------------------
thr_clmb_IB_pct = 100 * ones(1, n_clmb_IB);

% --- IB Cruise throttle --------------------------------------------------
t_cr_IB_rel = linspace(0, t_cruise_IB_min, n_pts);
W_cr_IB_N   = linspace(W3_IB_kg, W4_IB_kg, n_pts) * g;
thr_cr_IB_pct = zeros(1, n_pts);
for i = 1:n_pts
    V_BE_i  = (4*k / (3*CD0))^(0.25) * sqrt(2*W_cr_IB_N(i) / (rho_cr * S));
    q_i     = 0.5 * rho_cr * V_BE_i^2;
    CL_i    = W_cr_IB_N(i) / (q_i * S);
    CD_i    = CD0 + k * CL_i^2;
    T_i     = q_i * S * CD_i;
    sigma_cr = rho_cr / rho_SL;
    P_A_i   = sigma_cr * P_A_SL;
    thr_cr_IB_pct(i) = 100 * (T_i * V_BE_i) / P_A_i;
end

% --- IB Descent throttle -------------------------------------------------
t_dc_IB_rel = linspace(0, t_desc_min, n_dc);
W_dc_IB_N   = linspace(W5_IB_kg, W6_IB_kg, n_dc) * g;
thr_dc_IB_pct = zeros(1, n_dc);
for i = 1:n_dc
    [~, rho_d, ~, ~] = StdAtmos(h_dc_vec(i));
    sigma_d = rho_d / rho_SL;
    P_A_d   = sigma_d * P_A_SL;
    q_d     = 0.5 * rho_d * V_descent^2;
    CL_d    = W_dc_IB_N(i) / (q_d * S);
    CD_d    = CD0 + k * CL_d^2;
    D_d     = q_d * S * CD_d;
    T_d     = D_d - W_dc_IB_N(i) * sin(gamma_desc);
    if T_d > 0
        thr_dc_IB_pct(i) = 100 * (T_d * V_descent) / P_A_d;
    else
        thr_dc_IB_pct(i) = 0;
    end
end


%% =========================================================================
%  SECTION 10 — BUILD GLOBAL MISSION TIME VECTORS
%
%  Global time axis: OB phases run 0 → t_air_OB_min.
%  A turnaround gap (t_turnaround_min) separates the legs on the time axis.
%  IB phases then run from (t_air_OB_min + t_turnaround_min).
%  Ground taxi segments (10 min each end) are shown as shaded annotations,
%  not as separate flight-path segments.
%
%  Cumulative distance resets to 0 at the start of the inbound leg
%  (distance is measured from the departure airport for each leg).
% =========================================================================

t_gap_start = t_air_OB_min;
t_gap_end   = t_air_OB_min + t_turnaround_min;
t_IB_offset = t_gap_end;     % inbound time zero on global axis

% --- OB global mission vectors -------------------------------------------
t_OB_clmb = t_clmb_OB;
h_OB_clmb = h_clmb_OB;
V_OB_clmb = V_clmb_OB;
d_OB_clmb = dx_clmb_OB / 1e3;    % [km] cumul. from OB departure

t_OB_cr   = t_climb_OB_min + t_cr_OB_rel;
h_OB_cr   = h_cruise * ones(1, n_pts);
V_OB_cr   = zeros(1, n_pts);
for i = 1:n_pts
    V_OB_cr(i) = (4*k/(3*CD0))^(0.25) * sqrt(2*W_cr_OB_N(i)/(rho_cr*S));
end
d_OB_cr   = (d_climb_OB_m + linspace(0, d_cruise_OB_m, n_pts)) / 1e3;

t_OB_dc   = t_climb_OB_min + t_cruise_OB_min + t_dc_OB_rel;
h_OB_dc   = h_dc_vec;
V_OB_dc   = V_descent * ones(1, n_dc);
d_OB_dc   = (d_climb_OB_m + d_cruise_OB_m + linspace(0, d_desc_m, n_dc)) / 1e3;

t_OB_all = [t_OB_clmb, t_OB_cr, t_OB_dc];
h_OB_all = [h_OB_clmb, h_OB_cr, h_OB_dc];
V_OB_all = [V_OB_clmb, V_OB_cr, V_OB_dc];
d_OB_all = [d_OB_clmb, d_OB_cr, d_OB_dc];

% --- IB global mission vectors -------------------------------------------
t_IB_clmb = t_IB_offset + t_clmb_IB;
h_IB_clmb = h_clmb_IB;
V_IB_clmb = V_clmb_IB;
d_IB_clmb = dx_clmb_IB / 1e3;    % [km] cumul. from IB departure (resets)

t_IB_cr   = t_IB_offset + t_climb_IB_min + t_cr_IB_rel;
h_IB_cr   = h_cruise * ones(1, n_pts);
V_IB_cr   = zeros(1, n_pts);
for i = 1:n_pts
    V_IB_cr(i) = (4*k/(3*CD0))^(0.25) * sqrt(2*W_cr_IB_N(i)/(rho_cr*S));
end
d_IB_cr   = (d_climb_IB_m + linspace(0, d_cruise_IB_m, n_pts)) / 1e3;

t_IB_dc   = t_IB_offset + t_climb_IB_min + t_cruise_IB_min + t_dc_IB_rel;
h_IB_dc   = h_dc_vec;
V_IB_dc   = V_descent * ones(1, n_dc);
d_IB_dc   = (d_climb_IB_m + d_cruise_IB_m + linspace(0, d_desc_m, n_dc)) / 1e3;

t_IB_all = [t_IB_clmb, t_IB_cr, t_IB_dc];
h_IB_all = [h_IB_clmb, h_IB_cr, h_IB_dc];
V_IB_all = [V_IB_clmb, V_IB_cr, V_IB_dc];
d_IB_all = [d_IB_clmb, d_IB_cr, d_IB_dc];


%% =========================================================================
%  SECTION 11 — FIGURE 1: Altitude & TAS vs Mission Time
%
%  Both legs plotted on the same axes. Turnaround gap shown as a shaded
%  rectangle. OB coloured by distance from OB departure (solid altitude,
%  dashed TAS). IB coloured by distance from IB departure using the same
%  colour table but with circle markers at segment boundaries to distinguish
%  from OB.
% =========================================================================

band_km        = 100;
total_dist_one_leg = R_leg_m / 1e3;   % [km]
n_bands_needed = ceil(total_dist_one_leg / band_km);
colour_base = [
    0.50, 0.00, 0.75;
    0.00, 0.20, 0.90;
    0.00, 0.80, 0.90;
    0.00, 0.65, 0.10;
    0.95, 0.85, 0.00;
    1.00, 0.50, 0.00;
    0.90, 0.00, 0.00;
];
if n_bands_needed <= size(colour_base,1)
    colour_rgb = colour_base(1:n_bands_needed, :);
else
    colour_rgb = colour_base(mod((0:n_bands_needed-1), size(colour_base,1))+1, :);
end
n_bands = n_bands_needed;

figure('Name','PUDO 670 Medevac Mission: Airspeed & Altitude', ...
    'Units','normalized','Position',[0.02 0.10 0.94 0.68]);

yyaxis left
hold on; box on; grid on;

% OB altitude (solid lines, coloured by OB cumulative distance)
for b = 1:n_bands
    d_lo = (b-1)*band_km; d_hi = b*band_km;
    in_b = d_OB_all >= d_lo & d_OB_all <= d_hi;
    idx  = find(in_b);
    if isempty(idx), continue; end
    idx_ext = unique([max(1,idx(1)-1), idx(:)', min(numel(d_OB_all),idx(end)+1)]);
    plot(t_OB_all(idx_ext), h_OB_all(idx_ext), '-', ...
        'Color', colour_rgb(b,:), 'LineWidth', 2.8);
end

% IB altitude (dashed lines, coloured by IB cumulative distance)
for b = 1:n_bands
    d_lo = (b-1)*band_km; d_hi = b*band_km;
    in_b = d_IB_all >= d_lo & d_IB_all <= d_hi;
    idx  = find(in_b);
    if isempty(idx), continue; end
    idx_ext = unique([max(1,idx(1)-1), idx(:)', min(numel(d_IB_all),idx(end)+1)]);
    plot(t_IB_all(idx_ext), h_IB_all(idx_ext), '--', ...
        'Color', colour_rgb(b,:), 'LineWidth', 2.8);
end
ylabel('Altitude  [m]', 'FontSize', 12, 'Color', 'k');
ylim([0, h_cruise * 2.2]);
set(gca, 'YColor', 'k');

yyaxis right
hold on;

% OB TAS (solid, thinner)
for b = 1:n_bands
    d_lo = (b-1)*band_km; d_hi = b*band_km;
    in_b = d_OB_all >= d_lo & d_OB_all <= d_hi;
    idx  = find(in_b);
    if isempty(idx), continue; end
    idx_ext = unique([max(1,idx(1)-1), idx(:)', min(numel(d_OB_all),idx(end)+1)]);
    plot(t_OB_all(idx_ext), V_OB_all(idx_ext), '-', ...
        'Color', colour_rgb(b,:), 'LineWidth', 1.4);
end

% IB TAS (dashed, thinner)
for b = 1:n_bands
    d_lo = (b-1)*band_km; d_hi = b*band_km;
    in_b = d_IB_all >= d_lo & d_IB_all <= d_hi;
    idx  = find(in_b);
    if isempty(idx), continue; end
    idx_ext = unique([max(1,idx(1)-1), idx(:)', min(numel(d_IB_all),idx(end)+1)]);
    plot(t_IB_all(idx_ext), V_IB_all(idx_ext), '--', ...
        'Color', colour_rgb(b,:), 'LineWidth', 1.4);
end
ylabel('True Airspeed  [m/s]', 'FontSize', 12, 'Color', 'k');
ylim([0, V_climb * 2.0]);
set(gca, 'YColor', 'k');

% Turnaround shaded region
yyaxis left
yl = ylim;
patch([t_gap_start t_gap_end t_gap_end t_gap_start], ...
      [yl(1) yl(1) yl(2) yl(2)], [0.85 0.85 0.85], ...
      'FaceAlpha', 0.45, 'EdgeColor', 'none', 'HandleVisibility', 'off');
text(mean([t_gap_start t_gap_end]), h_cruise*1.55, ...
    sprintf('45-min\nturnaround'), 'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', [0.3 0.3 0.3], 'HorizontalAlignment', 'center');

% Phase boundary lines and labels — Outbound
xline(t_climb_OB_min,                         'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
xline(t_climb_OB_min + t_cruise_OB_min,       'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
text(t_climb_OB_min*0.42,               h_cruise*0.45, 'OB CLIMB', ...
    'FontSize',8,'FontWeight','bold','Color',[0.2 0.2 0.6],'HorizontalAlignment','center');
text(t_climb_OB_min + t_cruise_OB_min*0.50, h_cruise*1.88, 'OB CRUISE', ...
    'FontSize',8,'FontWeight','bold','Color',[0.2 0.2 0.6],'HorizontalAlignment','center');
text(t_climb_OB_min + t_cruise_OB_min + t_desc_min*0.5, h_cruise*0.45, 'OB DESC', ...
    'FontSize',8,'FontWeight','bold','Color',[0.2 0.2 0.6],'HorizontalAlignment','center');

% Phase boundary lines and labels — Inbound
xline(t_IB_offset + t_climb_IB_min,                         'k--', 'LineWidth', 1.0, 'HandleVisibility','off');
xline(t_IB_offset + t_climb_IB_min + t_cruise_IB_min,       'k--', 'LineWidth', 1.0, 'HandleVisibility','off');
text(t_IB_offset + t_climb_IB_min*0.42,               h_cruise*0.45, 'IB CLIMB', ...
    'FontSize',8,'FontWeight','bold','Color',[0.6 0.1 0.1],'HorizontalAlignment','center');
text(t_IB_offset + t_climb_IB_min + t_cruise_IB_min*0.50, h_cruise*1.88, 'IB CRUISE', ...
    'FontSize',8,'FontWeight','bold','Color',[0.6 0.1 0.1],'HorizontalAlignment','center');
text(t_IB_offset + t_climb_IB_min + t_cruise_IB_min + t_desc_min*0.5, h_cruise*0.45, 'IB DESC', ...
    'FontSize',8,'FontWeight','bold','Color',[0.6 0.1 0.1],'HorizontalAlignment','center');

xlabel('Mission Time  [min]', 'FontSize', 12);
xlim([0, t_IB_all(end) * 1.02]);
title({'PUDO 670 — Medevac Mission: True Airspeed & Altitude vs Time', ...
    sprintf('Config 3 | Round-Trip 350 nmi/leg | Cruise at V_{BE} | Solid=OB  Dashed=IB  | Thick=Altitude  Thin=TAS')}, ...
    'FontSize', 10, 'FontWeight', 'bold');

% Colour legend
yyaxis left
leg_h   = gobjects(n_bands,1);
leg_lbl = cell(n_bands,1);
for b = 1:n_bands
    d_lo = (b-1)*band_km;
    if d_lo >= total_dist_one_leg, break; end
    leg_h(b)   = plot(nan, nan, '-', 'Color', colour_rgb(b,:), 'LineWidth', 3.5);
    leg_lbl{b} = sprintf('%d - %d km', d_lo, b*band_km);
end
valid_idx = arrayfun(@(x) isgraphics(x,'line'), leg_h);
lg = legend(leg_h(valid_idx), leg_lbl(valid_idx), ...
    'Location','northeast', 'FontSize', 9);
lg.Title.String = 'Cumul. dist (per leg)';


%% =========================================================================
%  SECTION 12 — FIGURE 2: Throttle vs Time (6 subplots)
%
%  Row 1: OB Climb   Row 2: OB Cruise   Row 3: OB Descent
%  Row 4: IB Climb   Row 5: IB Cruise   Row 6: IB Descent
%  Each subplot uses global mission time on the x-axis so the overall
%  structure of the flight is immediately readable.
% =========================================================================

t_OB_clmb_global = t_clmb_OB;
t_OB_cr_global   = t_climb_OB_min + t_cr_OB_rel;
t_OB_dc_global   = t_climb_OB_min + t_cruise_OB_min + t_dc_OB_rel;

t_IB_clmb_global = t_IB_offset + t_clmb_IB;
t_IB_cr_global   = t_IB_offset + t_climb_IB_min + t_cr_IB_rel;
t_IB_dc_global   = t_IB_offset + t_climb_IB_min + t_cruise_IB_min + t_dc_IB_rel;

col_OB = [0.10 0.30 0.85];   % blue family for OB
col_OB_cr = [0.82 0.15 0.08];
col_OB_dc = [0.05 0.60 0.10];
col_IB = [0.55 0.00 0.75];   % purple family for IB
col_IB_cr = [0.90 0.55 0.00];
col_IB_dc = [0.00 0.65 0.55];

figure('Name','PUDO 670 Medevac Mission: Throttle Profile', ...
    'Units','normalized','Position',[0.02 0.03 0.94 0.92]);

% --- OB Climb ---
subplot(6,1,1);
plot(t_OB_clmb_global, thr_clmb_OB_pct, '-', 'Color', col_OB, 'LineWidth', 2.2);
hold on;
yline(100,'k--','LineWidth',0.8,'HandleVisibility','off');
ylabel('Thr [%]','FontSize',9);
title(sprintf('OB Climb | V_{TAS}=%.0f m/s | Full throttle | Avg ROC=%.1f m/s (%.0f fpm)', ...
    V_climb, ROC_avg_OB_ms, ROC_avg_OB_fpm),'FontSize',9);
ylim([0,125]); xlim([t_OB_clmb_global(1), t_OB_clmb_global(end)]); grid on; box on;
text(mean(t_OB_clmb_global), 50, ...
    sprintf('t_{climb}=%.1f min  |  d_{climb}=%.1f km  (%.1f nmi)', ...
    t_climb_OB_min, d_climb_OB_m/1e3, d_climb_OB_m/1852), ...
    'FontSize',8,'HorizontalAlignment','center','BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);

% --- OB Cruise ---
subplot(6,1,2);
plot(t_OB_cr_global, thr_cr_OB_pct, '-', 'Color', col_OB_cr, 'LineWidth', 2.2);
ylabel('Thr [%]','FontSize',9);
title(sprintf('OB Cruise | V_{BE}=%.1f–%.1f m/s | h=%.0f m (%.0f ft) | L/D=%.2f', ...
    V_BE_OB, (4*k/(3*CD0))^.25*sqrt(2*W4_OB_kg*g/(rho_cr*S)), h_cruise, h_cruise/0.3048, LD_BE),'FontSize',9);
y_cr = max(thr_cr_OB_pct)*1.35;
ylim([0, y_cr]); xlim([t_OB_cr_global(1), t_OB_cr_global(end)]); grid on; box on;
text(mean(t_OB_cr_global), max(thr_cr_OB_pct)*0.4, ...
    sprintf('W_{start}=%.0f kg → W_{end}=%.0f kg | d_{cr}=%.0f km (%.0f nmi)', ...
    W3_OB_kg, W4_OB_kg, d_cruise_OB_km, d_cruise_OB_nmi), ...
    'FontSize',8,'HorizontalAlignment','center','BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);

% --- OB Descent ---
subplot(6,1,3);
plot(t_OB_dc_global, thr_dc_OB_pct, '-', 'Color', col_OB_dc, 'LineWidth', 2.2);
ylabel('Thr [%]','FontSize',9);
title(sprintf('OB Descent | V_{TAS}=%.2f m/s | ROD=%.0f fpm', V_descent, ROD_fpm),'FontSize',9);
y_dc = max(max(thr_dc_OB_pct)*1.35, 5.0);
ylim([0, y_dc]); xlim([t_OB_dc_global(1), t_OB_dc_global(end)]); grid on; box on;
if max(thr_dc_OB_pct) < 0.10
    text(mean(t_OB_dc_global), y_dc*0.5, 'Power-off glide — gravity sustains descent', ...
        'FontSize',8,'HorizontalAlignment','center','BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);
end

% --- IB Climb ---
subplot(6,1,4);
plot(t_IB_clmb_global, thr_clmb_IB_pct, '-', 'Color', col_IB, 'LineWidth', 2.2);
hold on;
yline(100,'k--','LineWidth',0.8,'HandleVisibility','off');
ylabel('Thr [%]','FontSize',9);
title(sprintf('IB Climb | V_{TAS}=%.0f m/s | Full throttle | Avg ROC=%.1f m/s (%.0f fpm)', ...
    V_climb, ROC_avg_IB_ms, ROC_avg_IB_fpm),'FontSize',9);
ylim([0,125]); xlim([t_IB_clmb_global(1), t_IB_clmb_global(end)]); grid on; box on;
text(mean(t_IB_clmb_global), 50, ...
    sprintf('t_{climb}=%.1f min  |  d_{climb}=%.1f km  (%.1f nmi)', ...
    t_climb_IB_min, d_climb_IB_m/1e3, d_climb_IB_m/1852), ...
    'FontSize',8,'HorizontalAlignment','center','BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);

% --- IB Cruise ---
subplot(6,1,5);
plot(t_IB_cr_global, thr_cr_IB_pct, '-', 'Color', col_IB_cr, 'LineWidth', 2.2);
ylabel('Thr [%]','FontSize',9);
title(sprintf('IB Cruise | V_{BE}=%.1f–%.1f m/s | h=%.0f m (%.0f ft) | L/D=%.2f', ...
    V_BE_IB, (4*k/(3*CD0))^.25*sqrt(2*W4_IB_kg*g/(rho_cr*S)), h_cruise, h_cruise/0.3048, LD_BE),'FontSize',9);
y_cr_ib = max(thr_cr_IB_pct)*1.35;
ylim([0, y_cr_ib]); xlim([t_IB_cr_global(1), t_IB_cr_global(end)]); grid on; box on;
text(mean(t_IB_cr_global), max(thr_cr_IB_pct)*0.4, ...
    sprintf('W_{start}=%.0f kg → W_{end}=%.0f kg | d_{cr}=%.0f km (%.0f nmi)', ...
    W3_IB_kg, W4_IB_kg, d_cruise_IB_km, d_cruise_IB_nmi), ...
    'FontSize',8,'HorizontalAlignment','center','BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);

% --- IB Descent ---
subplot(6,1,6);
plot(t_IB_dc_global, thr_dc_IB_pct, '-', 'Color', col_IB_dc, 'LineWidth', 2.2);
xlabel('Mission Time  [min]','FontSize',10);
ylabel('Thr [%]','FontSize',9);
title(sprintf('IB Descent | V_{TAS}=%.2f m/s | ROD=%.0f fpm', V_descent, ROD_fpm),'FontSize',9);
y_dc_ib = max(max(thr_dc_IB_pct)*1.35, 5.0);
ylim([0, y_dc_ib]); xlim([t_IB_dc_global(1), t_IB_dc_global(end)]); grid on; box on;
if max(thr_dc_IB_pct) < 0.10
    text(mean(t_IB_dc_global), y_dc_ib*0.5, 'Power-off glide — gravity sustains descent', ...
        'FontSize',8,'HorizontalAlignment','center','BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);
end

sgtitle(sprintf('PUDO 670 — Medevac Mission Throttle Profile  |  Total mission time: %.1f min (%.2f hr)', ...
    t_total_min, t_total_min/60), 'FontSize', 12, 'FontWeight', 'bold');