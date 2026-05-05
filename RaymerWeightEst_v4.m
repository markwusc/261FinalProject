%% RaymerWeightEst_v4.m
% =========================================================================
% Arctic STOL Aircraft — MTOW Estimation with Physics-Based Mission Simulation
% Method: Raymer EWF iteration (SINGLE-ENGINE ONLY), with numerically
%         integrated climb and descent replacing fixed Raymer fuel fractions.
%
% ── KEY CHANGES FROM v3 ───────────────────────────────────────────────────
%
%  1. SINGLE-ENGINE ONLY.
%     Twin-engine configuration removed. All analysis targets a single
%     PT6A-60A turboprop per the current design direction.
%
%  2. PHYSICS-BASED CLIMB (replaces ff_climb = 0.980).
%     Aircraft climbs on full throttle. At each 5-second timestep:
%       V_climb = max(67 m/s,  V_Pmin(h, W))
%     where V_Pmin = sqrt(2Wg / (rho·S·CL_Pmin)) is the speed for minimum
%     power required — the theoretical maximum-ROC speed.
%     67 m/s is enforced as a hard lower bound on airspeed.
%     ROC = (P_A - P_R) / (W·g) is computed at every step.
%     Fuel burn: dW/dt = -P_shaft × SFCp_SI (shaft power basis).
%
%  3. PHYSICS-BASED DESCENT (replaces ff_descent = 0.990).
%     Target: ROD = 1000 fpm = 5.080 m/s, V ≥ 67 m/s.
%     At each altitude step, the descent speed is found by bisection:
%       Find V such that  (P_R(V) - P_A_min) / (W·g) = ROD_target
%     where P_A_min = 2% throttle × P_A_sl × sigma.
%     If V_opt ≥ 67 m/s → fly at V_opt (no extra throttle).
%     If V_opt < 67 m/s → fly at 67 m/s, throttle up to cap ROD:
%       P_A = P_R(67) - W·g·ROD_target
%     In practice for this aircraft, V_opt < 67 m/s throughout the descent
%     because P_R(67) already exceeds the ROD power budget at all altitudes.
%     Expected throttle: ~9% at 15,000 ft, increasing to ~19% at sea level.
%     This is a DESIGN CRITIQUE item: the original "1–5% throttle" assumption
%     significantly underestimates descent fuel. See Section 7 for details.
%
%  4. TIME-BASED 450 NMI BUDGET (replaces a fixed 450 nmi cruise range).
%     The 450 nmi is the TOTAL ground distance including climb and descent:
%       d_cruise = 450 nmi - d_climb - d_descent
%     Both d_climb and d_descent are outputs of the physics simulations.
%     The cruise distance is therefore shorter than in v3.
%
%  5. FUEL CONSUMPTION FORMULA.
%     All segments use:  dW/dt = -P_shaft × SFCp_SI  [kg/s]
%     where SFCp_SI = SFC in [kg / (W_shaft · s)], computed from the
%     rated PT6A-60A value of 0.548 lb/(hp·hr).
%     Relationships:
%       P_A = eta_p × P_shaft          (propulsive = shaft × efficiency)
%       Climb:   P_shaft = P_A_sl × sigma       (full throttle)
%       Cruise:  P_shaft = P_R(V) / eta_p      (T = D, steady level)
%       Descent: P_shaft = P_A_needed / eta_p  (throttle for ROD target)
%
%  6. LOITER: 50 MINUTES AT SEA LEVEL.
%     50 min (not 45 per the v3 default) using the exact Breguet endurance
%     formula from v3. Sea level = highest density = minimum power required
%     for a propeller aircraft = maximum endurance. This is the reserve.
%
%  7. RESERVES: 1% STUCK FUEL (trapped/unusable in tanks/lines).
%     The 50-min loiter IS the operational reserve. No separate 5% margin.
%     Total fuel = mission_fuel / (1 - 0.01)
%     (The 1% is of total tank capacity, including itself — solved algebraically.)
%
%  8. L/D COMPARISON: Historical single-engine STOL L/D vs design-point L/D.
%     Shows fuel-weight sensitivity to L/D assumption.
%
% ── PT6A-60A ENGINE ──────────────────────────────────────────────────────
%   Rated shaft power:    1050 shp  (= 782,985 W)
%   Propeller efficiency: 0.80      (constant-speed prop, typical)
%   Available power at SL: P_A_sl = eta_p × P_shp_SI = 626,388 W
%   SFC at rated power:   0.548 lb/(hp·hr)  → 9.261×10⁻⁸ kg/(W·s)
%   Power vs altitude:    P_A(h) = sigma(h) × P_A_sl   [simple ISA model]
%   NOTE: A real PT6A-60A has torque-limited flat-rating below ~10,000 ft.
%   We use the simple sigma model for consistency with CruiseOptimization_v3,
%   TradestudyTakeoff_S_AR, and ConstraintAnalysis_TO_Land_v2. This is
%   CONSERVATIVE — the real engine delivers more power at intermediate
%   altitudes than sigma scaling implies, so actual climb is better.
%
% ── UNITS ────────────────────────────────────────────────────────────────
%   SI throughout (kg, m, N, W, s). Raymer EWF formula requires lb — the
%   conversion is applied locally and labelled explicitly every time.
%
% Authors: AME 261 Design Team
% Date:    May 2026
% Version: 4.0
% =========================================================================

clear; clc; close all;

%% =========================================================================
%  >>>  USER INPUTS — EDIT THIS BLOCK EACH DESIGN ITERATION  <<<
% =========================================================================

% --- Wing geometry (current design point) ---
S   = 44.9;     % Wing reference area          [m²]
AR  = 7.2;      % Aspect ratio                 [-]

% --- Aerodynamics (current design point) ---
CD0 = 0.030;    % Parasite drag coefficient, clean config   [-]
e   = 0.90;     % Oswald span efficiency (RFP §4.3: 0.9 clean)  [-]

% =========================================================================
%  END OF USER INPUT BLOCK
% =========================================================================


%% =========================================================================
%  SECTION 1 — REFERENCE AIRCRAFT SURVEY
%
%  Single-engine 9-pax STOL turboprops: our most direct comparators.
%  Pilatus PC-12 is less STOL but is the upper bound for single-engine size.
%  Historical cruise L/D values are estimated from published performance data:
%    L/D_cruise ≈ W_cruise × V_cruise / (P_engine × eta_p × (1 - climb_frac))
%  These are ORDER-OF-MAGNITUDE estimates suitable for sizing comparison.
%  Actual values depend on specific operating conditions.
% =========================================================================

% Each entry: name, MTOW [kg], OEW [kg], estimated cruise L/D
ref(1).name = 'Cessna 208B Grand Caravan'; ref(1).MTOW = 3969; ref(1).OEW = 2145; ref(1).LD = 11.0;
ref(2).name = 'Quest/Daher Kodiak 100';    ref(2).MTOW = 3062; ref(2).OEW = 1934; ref(2).LD = 10.5;
ref(3).name = 'Pilatus PC-12 NGX';         ref(3).MTOW = 4740; ref(3).OEW = 2812; ref(3).LD = 13.0;
% STOL L/D notes:
%   Caravan:  straight wing, fixed-gear, large chord → L/D ~11 at cruise
%   Kodiak:   optimized STOL wing with large Fowler flaps; slightly lower
%             L/D than Caravan due to shorter span for STOL compatibility
%   PC-12:    higher AR wing, retractable gear, pressurized → L/D ~13
%             (NOT a true STOL aircraft — upper bound for our design)

fprintf('=== SECTION 1: REFERENCE AIRCRAFT SURVEY ===\n');
fprintf('%-30s | %9s | %8s | %5s | %10s\n', ...
    'Aircraft', 'MTOW (kg)', 'OEW (kg)', 'EWF', 'L/D cruise');
fprintf('%s\n', repmat('-', 1, 72));
for i = 1:length(ref)
    ref(i).EWF = ref(i).OEW / ref(i).MTOW;
    fprintf('%-30s | %9.0f | %8.0f | %5.3f | %10.1f\n', ...
        ref(i).name, ref(i).MTOW, ref(i).OEW, ref(i).EWF, ref(i).LD);
end
SE_EWF     = [ref.EWF];
LD_hist    = [ref.LD];
LD_hist_mn = mean(LD_hist);
fprintf('\nSingle-engine EWF:  %.3f – %.3f  (mean %.3f)\n', ...
    min(SE_EWF), max(SE_EWF), mean(SE_EWF));
fprintf('Historical L/D:     %.1f – %.1f  (mean %.1f)\n\n', ...
    min(LD_hist), max(LD_hist), LD_hist_mn);


%% =========================================================================
%  SECTION 2 — FIXED DESIGN PARAMETERS
% =========================================================================

% ── Payload and crew ──────────────────────────────────────────────────────
W_payload_kg = 1530.0;              % [kg]  9 pax + baggage (no crew)
W_crew_kg    = 2 * 200 * 0.4536;   % [kg]  2 pilots × 200 lb = 181.4 kg (in OEW)
W_fixed_kg   = W_payload_kg + W_crew_kg;   % drives the sizing denominator

% ── PT6A-60A engine specs ─────────────────────────────────────────────────
P_shp_rated = 1050;                         % rated shaft horsepower     [shp]
P_shp_SI    = P_shp_rated * 745.7;          % rated shaft power          [W]
eta_p       = 0.80;                          % propeller efficiency       [-]
P_A_sl      = eta_p * P_shp_SI;            % available propulsive power at SL [W]
SFCp_imp    = 0.548;                         % specific fuel consumption  [lb/(hp·hr)]
% Convert SFCp to SI [kg / (W_shaft · s)]:
%   SFCp_SI = [lb/(hp·hr)] × [0.4536 kg/lb] / [745.7 W/hp × 3600 s/hr]
SFCp_SI     = SFCp_imp * 0.4536 / (745.7 * 3600);   % 9.261×10⁻⁸ kg/(W·s)

% ── Mission parameters ────────────────────────────────────────────────────
R_total_m   = 450 * 1852;           % total ground distance [m]   (450 nmi)
h_cruise_m  = 15000 * 0.3048;       % cruise altitude       [m]   (15,000 ft = 4572.0 m)
V_min_ms    = 67.0;                  % minimum airspeed      [m/s] (= 130 KTAS)
ROD_ms      = 1000 * 0.3048 / 60;   % target descent ROD    [m/s] (1000 fpm = 5.080 m/s)
t_loiter_s  = 50 * 60;              % loiter reserve        [s]   (50 min)
throttle_desc = 0.02;                % baseline descent throttle (low end of 1–5%)

% ── Physical constants ────────────────────────────────────────────────────
g         = 9.81;      % gravitational acceleration [m/s²]
rho_SL    = 1.225;     % ISA sea-level density      [kg/m³]
lb_per_kg = 2.20462;   % unit conversion constant   [lb/kg]

% ── Derived aerodynamic parameters ───────────────────────────────────────
k_ind = 1 / (pi * AR * e);           % induced drag factor  [-]
%   Parabolic drag polar: CD = CD0 + k_ind × CL²
%   L/D_max: d/dCL[CL/CD] = 0  →  CL_opt = sqrt(CD0/k), CD_opt = 2×CD0
E_m   = sqrt(1 / (4 * CD0 * k_ind)); % maximum L/D          [-]

% ── Raymer weight coefficients (single-engine prop, Table 3.1) ───────────
A_ray = 2.36;    % Raymer A coefficient (fitted to GA prop data)
C_ray = -0.18;   % Raymer C exponent    (negative: heavier → lower EWF)
STOL_penalty = 1.04;   % +4% on EWF for STOL structure (see v3 header)
% STOL structural additions not in Raymer's GA dataset:
%   - Double-slotted Fowler flap system
%   - Reinforced gravel-rated undercarriage (higher gear loads + mud guards)
%   - Full ice-protection (FIKI per RFP)
%   - Structural reinforcement at gravel-runway stress concentrations

% ── Reserves ──────────────────────────────────────────────────────────────
frac_stuck = 0.01;   % 1% trapped/unusable fuel in tanks and lines

% ── Raymer fuel fractions for short ground segments (Table 3.2) ──────────
%   These are kept for warmup and takeoff because these segments are short
%   enough that numerical integration would add noise without improving accuracy.
%   All other segments (climb, cruise, descent) are replaced by physics sim.
ff_warmup  = 0.990;   % engine start, warmup, taxi to runway
ff_takeoff = 0.995;   % full-power takeoff roll (brief, captured by fraction)

% ── Raymer iteration settings ─────────────────────────────────────────────
tol_kg   = 0.5;    % convergence tolerance [kg]
max_iter = 250;    % maximum iterations

fprintf('=== SECTION 2: DESIGN INPUTS ===\n');
fprintf('  Wing area S          : %.1f m²\n',    S);
fprintf('  Aspect ratio AR      : %.1f\n',       AR);
fprintf('  CD0 (clean)          : %.4f\n',       CD0);
fprintf('  Oswald e             : %.2f\n',        e);
fprintf('  k_ind                : %.5f\n',       k_ind);
fprintf('  L/D_max (design)     : %.2f\n',       E_m);
fprintf('  Historical L/D mean  : %.2f\n',       LD_hist_mn);
fprintf('  Payload (no crew)    : %.1f kg\n',    W_payload_kg);
fprintf('  Crew (in OEW)        : %.1f kg\n',    W_crew_kg);
fprintf('  W_fixed              : %.1f kg\n',    W_fixed_kg);
fprintf('  PT6A-60A P_shp       : %.0f shp  (%.0f W)\n', P_shp_rated, P_shp_SI);
fprintf('  P_A_sl (prop output) : %.0f W  (%.1f kW)\n',  P_A_sl, P_A_sl/1e3);
fprintf('  SFCp                 : %.3f lb/(hp·hr)  →  %.4e kg/(W·s)\n', SFCp_imp, SFCp_SI);
fprintf('  eta_p                : %.2f\n',        eta_p);
fprintf('  Cruise altitude      : %.0f m  (%.0f ft)\n', h_cruise_m, h_cruise_m/0.3048);
fprintf('  Min airspeed V_min   : %.1f m/s  (%.0f KTAS)\n', V_min_ms, V_min_ms/0.5144);
fprintf('  Target ROD           : %.3f m/s  (%.0f fpm)\n', ROD_ms, ROD_ms/0.3048*60);
fprintf('  Loiter duration      : %.0f min\n',   t_loiter_s/60);
fprintf('  Stuck fuel reserve   : %.0f%%\n',     frac_stuck*100);
fprintf('  Raymer A, C          : %.2f, %.2f  (single-engine GA)\n', A_ray, C_ray);
fprintf('  STOL EWF penalty     : %.2fx\n\n',    STOL_penalty);


%% =========================================================================
%  SECTION 3 — LOITER AERODYNAMICS: BEST-ENDURANCE OPERATING POINT
%
%  For maximum propeller-aircraft endurance, maximise CL^(3/2) / CD.
%  Using parabolic polar CD = CD0 + k·CL²:
%    d/dCL [CL^(3/2) / CD] = 0
%    → CL_BE = sqrt(3 × CD0 / k)    (best-endurance lift coefficient)
%       CD_BE = 4 × CD0              (3× induced drag + 1× parasite at this CL)
%
%  Exact Breguet endurance rearranged for fuel fraction (see v3 for derivation):
%    W1 = [E/K + W0^(-1/2)]^(-2)       where E = loiter time [s]
%    K  = (eta_p / (g × SFCp)) × (CL^1.5/CD) × sqrt(2 × rho × S)   [N^(1/2)]
%
%  Loiter is computed at SEA LEVEL (rho_SL = 1.225 kg/m³).
%  For a prop aircraft: lower altitude → higher density → lower airspeed
%  for same CL → lower power required → MAXIMUM endurance at sea level.
% =========================================================================

CL_BE    = sqrt(3 * CD0 / k_ind);     % best-endurance lift coeff   [-]
CD_BE    = 4 * CD0;                    % best-endurance drag coeff   [-]
BE_param = CL_BE^1.5 / CD_BE;         % endurance figure of merit   [-]

% Stall check: CL_BE must not exceed maximum achievable lift coefficient
CL_max = 2.4;   % typical STOL CL_max with Fowler flap deployed
if CL_BE > CL_max
    warning(['CL_BE = %.3f exceeds CL_max = %.3f.\n' ...
             'Loiter is stall-limited. Capping CL_BE at CL_max.'], CL_BE, CL_max);
    CL_BE    = CL_max;
    CD_BE    = CD0 + k_ind * CL_BE^2;
    BE_param = CL_BE^1.5 / CD_BE;
end

% K_loiter constant [N^(1/2)]:  weight-and-time-independent part of Breguet
K_loiter = (eta_p / (g * SFCp_SI)) * BE_param * sqrt(2 * rho_SL * S);

fprintf('=== SECTION 3: LOITER AERODYNAMICS (sea level) ===\n');
fprintf('  CL_BE        : %.4f\n',  CL_BE);
fprintf('  CD_BE        : %.4f  (= 4 × CD0 = %.4f — check)\n', CD_BE, 4*CD0);
fprintf('  CL^1.5 / CD  : %.4f\n',  BE_param);
fprintf('  K_loiter     : %.4e  [N^(1/2)]\n\n', K_loiter);


%% =========================================================================
%  SECTION 4 — PRE-LOOP DESCENT DISTANCE ESTIMATE
%
%  The cruise distance is:  d_cruise = 450 nmi - d_climb - d_descent
%  d_climb is computed fresh each Raymer iteration (weight-dependent).
%  d_descent is nearly constant across iterations: ROD is fixed at 5.08 m/s
%  and the descent takes h_cruise/ROD ≈ 900 s regardless of weight.
%  Airspeed during descent is ≥ 67 m/s (typically ≈ 67 m/s for our aircraft).
%
%  Pre-compute a reference descent distance using an assumed mid-mission weight
%  to avoid running a full sim_descent ghost call inside every Raymer iteration.
%  This estimate is refined after the converged solution is found.
%
%  Note: throughout the descent, V_opt (speed for ROD = 1000 fpm at 2% throttle)
%  is found by bisection at each altitude step. Analysis shows V_opt < 67 m/s
%  throughout the descent for this aircraft, so the aircraft always flies at
%  exactly V_min = 67 m/s with throttle varied to achieve the target ROD.
%  The ground distance is therefore:
%    d_descent ≈ V_min × t_descent = 67 × (h_cruise / ROD_ms) = 67 × 900 = 60,300 m
%  The actual simulation is run per-iteration for accuracy (weight affects P_R,
%  which affects required throttle, which affects fuel — but NOT d_descent much).
% =========================================================================

d_descent_ref_m = V_min_ms * (h_cruise_m / ROD_ms);   % [m]  ≈ 60,300 m ≈ 32.6 nmi
fprintf('=== SECTION 4: DESCENT DISTANCE PRE-ESTIMATE ===\n');
fprintf('  t_descent = h_cruise / ROD = %.0f / %.3f = %.0f s  (%.1f min)\n', ...
    h_cruise_m, ROD_ms, h_cruise_m/ROD_ms, h_cruise_m/ROD_ms/60);
fprintf('  d_descent_ref = V_min × t_descent = %.0f m  (%.1f nmi)\n', ...
    d_descent_ref_m, d_descent_ref_m/1852);
fprintf('  (Refined by actual simulation inside Raymer loop)\n\n');


%% =========================================================================
%  SECTION 5 — RAYMER ITERATIVE WEIGHT SIZING
%
%  Iteration sequence per step:
%    1. Warmup + takeoff fuel fractions (Raymer Table 3.2 — brief events)
%    2. sim_climb:   SL → 15,000 ft, V = max(67, V_Pmin), full throttle
%    3. d_cruise = 450 nmi - d_climb - d_descent_ref  (pre-estimate used)
%    4. sim_cruise:  constant altitude 15,000 ft, V = 67 m/s, T = D
%    5. sim_descent: 15,000 ft → SL, ROD = 1000 fpm, V ≥ 67 m/s
%    6. Loiter reserve: 50 min at sea level, exact Breguet formula
%    7. Total fuel = (sum of all) / (1 - 0.01 stuck fraction)
%    8. EWF from Raymer × STOL penalty
%    9. W0_new = (W_fixed + W_fuel) / (1 - EWF)
%   10. Blend and check convergence
%
%  Note: The cruise distance uses d_descent_ref (pre-computed). After
%  convergence, the actual d_descent from the final sim is compared.
%  If the range error exceeds 0.5 nmi, a one-pass correction is applied.
% =========================================================================

fprintf('=== SECTION 5: RAYMER ITERATIVE SIZING ===\n\n');
fprintf('  Single-engine only. Raymer A=%.2f, C=%.2f, STOL penalty=%.2fx.\n\n', ...
    A_ray, C_ray, STOL_penalty);
fprintf('  %-4s | %-10s | %-8s | %-9s | %-9s | %-10s | %-7s\n', ...
    'Iter', 'W0 [kg]', 'EWF', 'Wfuel[kg]', 'Wclimb[kg]', 'Wcruise[kg]', 'Δ [kg]');
fprintf('  %s\n', repmat('-', 1, 75));

W0_guess = 4000;    % initial MTOW guess [kg]
converged = false;

% Pre-allocate storage for final-iteration segment values
seg.fuel_warmup  = NaN;  seg.fuel_takeoff = NaN;
seg.fuel_climb   = NaN;  seg.fuel_cruise  = NaN;
seg.fuel_descent = NaN;  seg.fuel_loiter  = NaN;
seg.fuel_stuck   = NaN;  seg.fuel_total   = NaN;
seg.d_climb   = NaN;  seg.d_cruise  = NaN;  seg.d_descent = NaN;
seg.t_climb   = NaN;  seg.t_cruise  = NaN;  seg.t_descent = NaN;

EWF_raw_final = NaN;  EWF_final = NaN;

for iter = 1:max_iter

    W = W0_guess;   % track current aircraft weight [kg] through the mission

    %% ── (1) Warmup and taxi ─────────────────────────────────────────────
    %   Raymer Table 3.2 fraction: captures engine start, runup, taxi.
    fw = W * (1 - ff_warmup);   % fuel burned [kg]
    W  = W * ff_warmup;         % weight after warmup

    %% ── (2) Takeoff roll ────────────────────────────────────────────────
    %   Short high-power event; fraction adequate for sizing accuracy.
    ft = W * (1 - ff_takeoff);  % fuel burned [kg]
    W  = W * ff_takeoff;        % weight after takeoff

    W_start_climb = W;   % save for potential cruise re-run

    %% ── (3) Climb: sea level → 15,000 ft ───────────────────────────────
    %   Full throttle. V = max(V_min, V_Pmin). See sim_climb for details.
    [fc, d_cl, t_cl, W] = sim_climb(W, S, k_ind, CD0, eta_p, ...
                                     P_shp_SI, SFCp_SI, V_min_ms, ...
                                     h_cruise_m, g, rho_SL);

    W_end_climb = W;   % weight at top of climb [kg]

    %% ── (4) Cruise distance allocation ──────────────────────────────────
    %   Use pre-computed descent reference. This avoids running a ghost
    %   descent call every iteration (saves ~180 bisection steps per iter).
    %   The cruise distance adjusts once after convergence if needed.
    d_cr = R_total_m - d_cl - d_descent_ref_m;    % cruise distance [m]

    if d_cr < 0
        % FATAL: climb + descent alone exceeds 450 nmi. Mission geometry broken.
        % This should not occur for 450 nmi / 15,000 ft cruise altitude.
        error(['Mission geometry error at iter %d:\n' ...
               '  d_climb = %.1f km, d_descent_ref = %.1f km\n' ...
               '  Sum = %.1f km > 450 nmi = %.1f km\n' ...
               'The aircraft climbs and descends more distance than the total range.\n' ...
               'Consider reducing cruise altitude or increasing range.'], ...
               iter, d_cl/1e3, d_descent_ref_m/1e3, ...
               (d_cl+d_descent_ref_m)/1e3, R_total_m/1e3);
    end

    %% ── (5) Cruise: 15,000 ft, V = 67 m/s, T = D ───────────────────────
    %   Throttle is continuously adjusted to maintain T = D (level flight).
    %   CL and CD are updated each timestep as W decreases.
    [fc2, t_cr, W] = sim_cruise(W, S, k_ind, CD0, eta_p, SFCp_SI, ...
                                 V_min_ms, h_cruise_m, d_cr, g);

    W_end_cruise = W;   % weight at start of descent [kg]

    %% ── (6) Descent: 15,000 ft → SL, ROD = 1000 fpm ───────────────────
    %   See sim_descent for detailed logic and design critique.
    [fd, d_de, t_de, W] = sim_descent(W, S, k_ind, CD0, eta_p, ...
                                       P_shp_SI, SFCp_SI, P_A_sl, ...
                                       V_min_ms, h_cruise_m, ROD_ms, ...
                                       throttle_desc, g, rho_SL);

    W_end_descent = W;   % weight at touchdown [kg]

    %% ── (7) Loiter reserve: 50 min at sea level ─────────────────────────
    %   Exact Breguet endurance formula.
    %   W is in Newtons for the formula (multiply by g):
    %     W1 = [t_loiter / K_loiter + W0^(-1/2)]^(-2)
    %   where K_loiter contains eta_p, SFCp, CL^1.5/CD, and sqrt(rho×S).
    W0_lot_N = W_end_descent * g;                              % [N]
    W1_lot_N = 1 / (t_loiter_s / K_loiter + W0_lot_N^(-0.5))^2;  % [N]
    fl       = (W0_lot_N - W1_lot_N) / g;   % loiter fuel burned [kg]

    %% ── (8) Total fuel ──────────────────────────────────────────────────
    %   Mission fuel = sum of all burned segments.
    %   Stuck fuel is ADDITIONAL — 1% of total tank capacity.
    %   Since stuck fuel is part of what we load, not what we burn:
    %     W_total_fuel × (1 - 0.01) = W_mission_fuel
    %     W_total_fuel = W_mission_fuel / 0.99
    fuel_mission = fw + ft + fc + fc2 + fd + fl;   % [kg]
    W_fuel_total = fuel_mission / (1 - frac_stuck); % [kg]
    fuel_stuck   = W_fuel_total - fuel_mission;     % [kg]

    %% ── (9) Raymer EWF ──────────────────────────────────────────────────
    %   IMPORTANT: Raymer formula uses W0 in POUNDS (imperial fit).
    %   Convert W0_guess to lb before applying the formula.
    W0_lbs  = W0_guess * lb_per_kg;               % [lb]
    EWF_raw = A_ray * (W0_lbs ^ C_ray);           % [-] Raymer prediction
    EWF     = EWF_raw * STOL_penalty;             % [-] with STOL correction

    %% ── (10) New MTOW estimate ──────────────────────────────────────────
    %   Weight breakdown: W0 = W_empty + W_crew + W_payload + W_fuel
    %   W_empty = EWF × W0   →   W0 × (1 - EWF) = W_fixed + W_fuel
    %   W0_new = (W_fixed + W_fuel_total) / (1 - EWF)
    denom  = 1 - EWF;
    if denom <= 0
        warning('EWF ≥ 1 at iter %d (EWF=%.4f). Structure alone fills W0. Increasing guess.', iter, EWF);
        W0_guess = W0_guess * 1.2;
        continue;
    end
    W0_new = (W_fixed_kg + W_fuel_total) / denom;
    delta  = abs(W0_new - W0_guess);

    % Print progress: always first 5 iters, every 10 after, and near convergence
    if iter <= 5 || mod(iter, 10) == 0 || delta < 2.0
        fprintf('  %4d | %10.2f | %8.5f | %9.2f | %9.2f | %10.2f | %7.2f\n', ...
            iter, W0_guess, EWF, W_fuel_total, fc, fc2, delta);
    end

    %% ── Convergence check ───────────────────────────────────────────────
    if delta < tol_kg
        converged = true;
        % Store all final-iteration values for reporting
        seg.fuel_warmup  = fw;    seg.fuel_takeoff = ft;
        seg.fuel_climb   = fc;    seg.fuel_cruise  = fc2;
        seg.fuel_descent = fd;    seg.fuel_loiter  = fl;
        seg.fuel_stuck   = fuel_stuck;
        seg.fuel_total   = W_fuel_total;
        seg.d_climb   = d_cl;  seg.d_cruise  = d_cr;  seg.d_descent = d_de;
        seg.t_climb   = t_cl;  seg.t_cruise  = t_cr;  seg.t_descent = t_de;
        EWF_raw_final = EWF_raw;
        EWF_final     = EWF;
        break;
    end

    % Damped update to prevent oscillation around the fixed point
    W0_guess = 0.5 * W0_guess + 0.5 * W0_new;
end

if ~converged
    warning('Raymer loop did NOT converge after %d iterations! Final Δ = %.2f kg.', ...
        max_iter, delta);
    % Store whatever the last iteration gave us
    seg.fuel_warmup  = fw;    seg.fuel_takeoff = ft;
    seg.fuel_climb   = fc;    seg.fuel_cruise  = fc2;
    seg.fuel_descent = fd;    seg.fuel_loiter  = fl;
    seg.fuel_stuck   = fuel_stuck;
    seg.fuel_total   = W_fuel_total;
    seg.d_climb   = d_cl;  seg.d_cruise  = d_cr;  seg.d_descent = d_de;
    seg.t_climb   = t_cl;  seg.t_cruise  = t_cr;  seg.t_descent = t_de;
    EWF_raw_final = EWF_raw;
    EWF_final     = EWF;
end

W0_final = W0_guess;

%% ── Range closure correction ─────────────────────────────────────────────
%   Now that the descent is physically simulated, check whether the actual
%   d_descent differs meaningfully from the pre-estimate used for d_cruise.
%   Field names match the struct defined inside the Raymer loop:
%     seg.d_climb, seg.d_cruise, seg.d_descent  (NOT d_cl / d_cr / d_de)
range_error_nmi = abs(seg.d_climb + seg.d_cruise + seg.d_descent - R_total_m) / 1852;
 
if range_error_nmi > 0.5
    fprintf('\n  [RANGE CORRECTION] Actual range = %.2f nmi (error %.2f nmi).\n', ...
        (seg.d_climb + seg.d_cruise + seg.d_descent)/1852, range_error_nmi);
    fprintf('  Re-running cruise with corrected d_cruise...\n\n');
 
    d_cr_corrected = R_total_m - seg.d_climb - seg.d_descent;
 
    if d_cr_corrected > 0
        % Re-run cruise from the saved end-of-climb weight with corrected distance
        [fc2_corr, t_cr_corr, W_end_cruise_corr] = sim_cruise( ...
            W_end_climb, S, k_ind, CD0, eta_p, SFCp_SI, ...
            V_min_ms, h_cruise_m, d_cr_corrected, g);
 
        % Re-run descent from corrected end-of-cruise weight
        [fd_corr, d_de_corr, t_de_corr, ~] = sim_descent( ...
            W_end_cruise_corr, S, k_ind, CD0, eta_p, P_shp_SI, SFCp_SI, P_A_sl, ...
            V_min_ms, h_cruise_m, ROD_ms, throttle_desc, g, rho_SL);
 
        % Update stored segment values with corrected results
        seg.fuel_cruise  = fc2_corr;
        seg.fuel_descent = fd_corr;
        seg.d_cruise     = d_cr_corrected;
        seg.d_descent    = d_de_corr;
        seg.t_cruise     = t_cr_corr;
        seg.t_descent    = t_de_corr;
 
        fprintf('  Corrected d_cruise = %.1f nmi, new range error = %.2f nmi.\n\n', ...
            d_cr_corrected/1852, ...
            abs(seg.d_climb + d_cr_corrected + d_de_corr - R_total_m)/1852);
    end
else
    fprintf('  Range closure OK: %.2f nmi (error %.2f nmi < 0.5 nmi threshold).\n\n', ...
        (seg.d_climb + seg.d_cruise + seg.d_descent)/1852, range_error_nmi);
end
 
 
%% =========================================================================
%  SECTION 6 — FINAL WEIGHT STATEMENT
% =========================================================================
 
W_struct_kg = EWF_final * W0_final;
W_OEW_kg    = W_struct_kg + W_crew_kg;     % OEW = structure + crew
fuel_mission_final = seg.fuel_warmup  + seg.fuel_takeoff + seg.fuel_climb + ...
                     seg.fuel_cruise  + seg.fuel_descent  + seg.fuel_loiter;
 
fprintf('\n=== SECTION 6: CONVERGED WEIGHT STATEMENT ===\n\n');
fprintf('  Converged: %s   |   Iterations to convergence: %d\n\n', ...
    mat2str(converged), iter);
 
fprintf('┌─────────────────────────────────────────────────────────────────────\n');
fprintf('│  CONFIG: Single-Engine Prop — PT6A-60A (%.0f shp, %.0f W)\n', ...
    P_shp_rated, P_shp_SI);
fprintf('├─────────────────────────────────────────────────────────────────────\n');
fprintf('│  WEIGHTS:\n');
fprintf('│    Max Takeoff Weight W0    : %7.1f kg  (%7.1f lb)  (%8.0f N)\n', ...
    W0_final, W0_final*lb_per_kg, W0_final*g);
fprintf('│    Structural Empty (Raymer): %6.1f kg  EWF_raw=%.4f, +STOL=%.4f\n', ...
    W_struct_kg, EWF_raw_final, EWF_final);
fprintf('│    Crew (2 pilots, in OEW)  : %6.1f kg\n', W_crew_kg);
fprintf('│    OEW total                : %6.1f kg  (%.1f%% of W0)\n', ...
    W_OEW_kg, 100*W_OEW_kg/W0_final);
fprintf('│    Payload (no crew)        : %6.1f kg  (%.1f%% of W0)\n', ...
    W_payload_kg, 100*W_payload_kg/W0_final);
fprintf('│    Total fuel (incl. stuck) : %6.1f kg  (%.1f%% of W0)\n', ...
    seg.fuel_total, 100*seg.fuel_total/W0_final);
fprintf('│\n');
fprintf('│  FUEL BREAKDOWN BY SEGMENT:\n');
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    'Warm-up & taxi', seg.fuel_warmup, ...
    100*seg.fuel_warmup/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    'Takeoff', seg.fuel_takeoff, ...
    100*seg.fuel_takeoff/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    sprintf('Climb  (%.0f m, %.1f min)', h_cruise_m, seg.t_climb/60), ...
    seg.fuel_climb, 100*seg.fuel_climb/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    sprintf('Cruise (%.1f nmi, %.1f min)', seg.d_cruise/1852, seg.t_cruise/60), ...
    seg.fuel_cruise, 100*seg.fuel_cruise/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    sprintf('Descent(%.0f m, %.1f min)', h_cruise_m, seg.t_descent/60), ...
    seg.fuel_descent, 100*seg.fuel_descent/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    'Loiter (50 min, sea level)', ...
    seg.fuel_loiter, 100*seg.fuel_loiter/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg  (%4.1f%% of fuel)\n', ...
    'Stuck/trapped (1%)', seg.fuel_stuck, ...
    100*seg.fuel_stuck/seg.fuel_total);
fprintf('│    %-30s: %6.2f kg\n', ...
    'SUBTOTAL (mission, before stuck)', fuel_mission_final);
fprintf('│    %-30s: %6.2f kg\n', 'TOTAL FUEL', seg.fuel_total);
fprintf('│\n');
fprintf('│  RANGE ACCOUNTING:\n');
fprintf('│    d_climb   = %6.1f km  (%5.1f nmi)  [%4.1f%% of 450 nmi]\n', ...
    seg.d_climb/1e3,   seg.d_climb/1852,   100*seg.d_climb/R_total_m);
fprintf('│    d_cruise  = %6.1f km  (%5.1f nmi)  [%4.1f%% of 450 nmi]\n', ...
    seg.d_cruise/1e3,  seg.d_cruise/1852,  100*seg.d_cruise/R_total_m);
fprintf('│    d_descent = %6.1f km  (%5.1f nmi)  [%4.1f%% of 450 nmi]\n', ...
    seg.d_descent/1e3, seg.d_descent/1852, 100*seg.d_descent/R_total_m);
d_total_check = seg.d_climb + seg.d_cruise + seg.d_descent;
fprintf('│    TOTAL     = %6.1f km  (%5.1f nmi)  [target: 450.0 nmi]\n', ...
    d_total_check/1e3, d_total_check/1852);
fprintf('│\n');
W_check = W_OEW_kg + W_payload_kg + seg.fuel_total;
fprintf('│  CLOSURE CHECK: W_OEW+W_payload+W_fuel = %.1f kg  (W0=%.1f kg, Δ=%.2f kg)\n', ...
    W_check, W0_final, abs(W_check - W0_final));
fprintf('└─────────────────────────────────────────────────────────────────────\n\n');
 
% Reference aircraft comparison
fprintf('─── Comparison with reference aircraft ──────────────────────────────\n');
for i = 1:length(ref)
    fprintf('  %-35s: %5.0f kg\n', ref(i).name, ref(i).MTOW);
end
fprintf('  %-35s: %5.0f kg  ← THIS DESIGN (v4, Raymer+STOL)\n', ...
    'This Design — Single Engine', W0_final);
EWF_hist_check = mean(SE_EWF);
W0_hist_check  = (W_fixed_kg + seg.fuel_total) / (1 - EWF_hist_check);
fprintf('  %-35s: %5.0f kg  ← HISTORICAL EWF CHECK (EWF=%.3f)\n\n', ...
    'Historical EWF estimate', W0_hist_check, EWF_hist_check);
fprintf('  NOTE: Raymer EWF = %.4f vs historical mean = %.3f.\n', ...
    EWF_final, EWF_hist_check);
fprintf('  Raymer+STOL is a LOWER bound — historical data implies W0 may be %.0f kg higher.\n\n', ...
    W0_hist_check - W0_final);
 
 
%% =========================================================================
%  SECTION 7 — L/D ANALYSIS: HISTORICAL vs DESIGN POINT
%
%  Three L/D cases are compared:
%   (A) Historical L/D band for similar single-engine STOL aircraft (10.5–13)
%   (B) Design-point L/D computed per-timestep at V=67 m/s, 15,000 ft
%   (C) L/D_max from the design polar (best achievable, lower speed)
%
%  For the sensitivity sweep, d_climb and d_descent are fixed to the
%  converged values (they do not change with L/D — climb uses full throttle,
%  descent uses fixed ROD). Only cruise fuel is varied via Breguet.
%
%  Breguet range (propeller, SI):
%    ff_cruise = exp( -d_cruise × g × SFCp_SI / (eta_p × L/D) )
% =========================================================================
 
% ── Design-point cruise L/D from converged simulation ────────────────────
%   Evaluate at mid-cruise weight for a representative single L/D value.
[~, rho_cruise] = StdAtmos(h_cruise_m);
W_start_cruise_ld = W0_final * ff_warmup * ff_takeoff - seg.fuel_climb;
W_mid_cruise      = W_start_cruise_ld - seg.fuel_cruise / 2;
CL_cruise         = W_mid_cruise * g / (0.5 * rho_cruise * V_min_ms^2 * S);
CD_cruise         = CD0 + k_ind * CL_cruise^2;
LD_design         = CL_cruise / CD_cruise;
 
fprintf('=== SECTION 7: L/D ANALYSIS ===\n\n');
fprintf('  Design polar (S=%.1f m², AR=%.1f, CD0=%.4f, e=%.2f):\n', ...
    S, AR, CD0, e);
fprintf('    L/D_max (theoretical)                  : %.2f\n', E_m);
fprintf('    L/D at cruise (67 m/s, 15000 ft, W_mid): %.2f\n', LD_design);
fprintf('    CL at cruise mid                       : %.4f\n', CL_cruise);
fprintf('    CD at cruise mid                       : %.4f\n', CD_cruise);
fprintf('      (induced = %.4f,  parasite = %.4f)\n', k_ind*CL_cruise^2, CD0);
fprintf('\n  Historical comparators:\n');
for i = 1:length(ref)
    fprintf('    %-30s: L/D_cruise ≈ %.1f\n', ref(i).name, ref(i).LD);
end
fprintf('    Historical mean                        : %.2f\n', LD_hist_mn);
 
if LD_design >= LD_hist_mn
    fprintf('\n  [OK] Design L/D (%.2f) meets or exceeds historical mean (%.2f).\n', ...
        LD_design, LD_hist_mn);
    fprintf('  Confirm CD0=%.4f with the component drag buildup in Appendix A.\n', CD0);
else
    fprintf('\n  [WARNING] Design L/D (%.2f) is below historical mean (%.2f).\n', ...
        LD_design, LD_hist_mn);
    fprintf('  Review AR, CD0, and wing area. Consider a AR/S trade study.\n');
end
 
% ── Check whether 67 m/s is above or below V_LDmax at cruise altitude ────
CL_opt  = sqrt(CD0 / k_ind);          % CL at L/D_max (min drag)
V_LDmax = sqrt(2 * W0_final * g / (rho_cruise * S * CL_opt));   % [m/s]
fprintf('\n  V at L/D_max (MTOW, 15000 ft): %.1f m/s  (%.0f KTAS)\n', ...
    V_LDmax, V_LDmax/0.5144);
if V_LDmax > V_min_ms
    fprintf('  V_LDmax > 67 m/s: cruise speed is BELOW L/D_max speed.\n');
    fprintf('  The aircraft is in the induced-drag-dominated (low-speed) regime.\n');
    fprintf('  Increasing cruise speed toward %.0f m/s would improve L/D.\n', V_LDmax);
else
    fprintf('  V_LDmax < 67 m/s: cruise speed is ABOVE L/D_max speed.\n');
    fprintf('  The aircraft is in the parasite-drag-dominated (high-speed) regime.\n');
    fprintf('  Increasing AR or reducing CD0 would raise V_LDmax toward 67 m/s.\n');
end
 
% ── L/D sensitivity sweep ─────────────────────────────────────────────────
%   Run Raymer iteration for each L/D using Breguet cruise (fast).
%   Climb and descent are re-simulated per W0 guess for physical consistency.
LD_labels = {'Hist min (Kodiak)', 'Hist mean', 'Hist max (PC-12)', ...
             'Design pt', 'L/D_max'};
LD_vals   = [min(LD_hist), LD_hist_mn, max(LD_hist), LD_design, E_m];
nLD       = length(LD_vals);
 
W0_ld = zeros(1, nLD);
Wf_ld = zeros(1, nLD);
 
fprintf('\n  L/D Sensitivity (Breguet cruise, physics climb/descent):\n');
fprintf('  %-22s | %6s | %10s | %10s | %+10s\n', ...
    'Case', 'L/D', 'Wfuel [kg]', 'MTOW [kg]', 'ΔW0 [kg]');
fprintf('  %s\n', repmat('-', 1, 68));
 
for ii = 1:nLD
    LD_t = LD_vals(ii);
    W0_g = W0_final;   % warm-start from converged value
 
    for it = 1:max_iter
        W_g = W0_g;
 
        % Warmup and takeoff fractions
        W_g     = W_g * ff_warmup * ff_takeoff;
        fw_ft_t = W0_g * (1 - ff_warmup * ff_takeoff);
 
        % Climb (physics — weight-dependent)
        [fc_t, d_cl_t, ~, W_g] = sim_climb(W_g, S, k_ind, CD0, eta_p, ...
            P_shp_SI, SFCp_SI, V_min_ms, h_cruise_m, g, rho_SL);
 
        % Cruise distance: total minus climb minus descent (use converged d_descent)
        d_cr_t = R_total_m - d_cl_t - seg.d_descent;
        if d_cr_t <= 0; d_cr_t = 100; end   % floor to prevent log(0)
 
        % Cruise fuel: Breguet at this L/D
        ff_cr_t = exp(-d_cr_t * g * SFCp_SI / (eta_p * LD_t));
        fc2_t   = W_g * (1 - ff_cr_t);
        W_g     = W_g * ff_cr_t;
 
        % Descent (physics — weight-dependent)
        [fd_t, ~, ~, W_g] = sim_descent(W_g, S, k_ind, CD0, eta_p, ...
            P_shp_SI, SFCp_SI, P_A_sl, V_min_ms, h_cruise_m, ROD_ms, ...
            throttle_desc, g, rho_SL);
 
        % Loiter: exact Breguet endurance at sea level
        W0_lt = W_g * g;
        W1_lt = 1 / (t_loiter_s / K_loiter + W0_lt^(-0.5))^2;
        fl_t  = (W0_lt - W1_lt) / g;
 
        % Total fuel
        Wfm_t = fw_ft_t + fc_t + fc2_t + fd_t + fl_t;
        Wft_t = Wfm_t / (1 - frac_stuck);
 
        % Raymer EWF at this W0 guess
        EWF_t = A_ray * ((W0_g * lb_per_kg) ^ C_ray) * STOL_penalty;
 
        % New MTOW
        W0_new_t = (W_fixed_kg + Wft_t) / (1 - EWF_t);
        if abs(W0_new_t - W0_g) < tol_kg; break; end
        W0_g = 0.5 * W0_g + 0.5 * W0_new_t;
    end
 
    W0_ld(ii) = W0_g;
    Wf_ld(ii) = Wft_t;
 
    dW0 = W0_g - W0_final;
    fprintf('  %-22s | %6.2f | %10.1f | %10.1f | %+10.1f\n', ...
        LD_labels{ii}, LD_t, Wft_t, W0_g, dW0);
end
 
% ── Delta summary ─────────────────────────────────────────────────────────
fprintf('\n  Fuel and MTOW delta vs physics-based design-point result:\n');
for ii = 1:nLD
    dWf  = Wf_ld(ii) - seg.fuel_total;
    dW0_ = W0_ld(ii) - W0_final;
    fprintf('    %-22s (L/D=%5.2f): fuel %+6.1f kg  →  MTOW %+6.1f kg\n', ...
        LD_labels{ii}, LD_vals(ii), dWf, dW0_);
end
fprintf('\n');
 
 
%% =========================================================================
%  SECTION 8 — DESIGN NOTES AND CRITIQUES
% =========================================================================
 
fprintf('=== SECTION 8: DESIGN NOTES AND CRITIQUES ===\n\n');
 
fprintf(['NOTE 1 — DESCENT THROTTLE IS HIGHER THAN INITIALLY ASSUMED.\n' ...
    '  The brief assumed 1-5%% throttle during descent. Physics shows:\n' ...
    '    P_A = P_R(67 m/s) - W*g*ROD_target\n' ...
    '  At 15,000 ft: P_R ~ 182 kW, P_A_needed ~ 32 kW  → ~9%%  throttle\n' ...
    '  At sea level: P_R ~ 271 kW, P_A_needed ~ 121 kW → ~19%% throttle\n' ...
    '  As the aircraft descends into denser air, drag at 67 m/s increases.\n' ...
    '  The pilot ADDS throttle to prevent ROD from exceeding 1000 fpm.\n' ...
    '  Descent fuel is therefore higher than a near-idle model suggests,\n' ...
    '  though it remains a small fraction of total fuel.\n\n']);
 
fprintf(['NOTE 2 — CONSTANT SFCp UNDERESTIMATES DESCENT FUEL BURN.\n' ...
    '  dW/dt = -P_shaft * SFCp_SI assumes SFCp = %.3f lb/(hp*hr) everywhere.\n' ...
    '  Real turboprop SFC rises sharply at partial throttle:\n' ...
    '    ~100%% power: 0.548 lb/(hp*hr)  (rated, used here)\n' ...
    '    ~50%%  power: ~0.62-0.65        (estimated)\n' ...
    '    ~15%%  power: ~0.80-0.90        (estimated)\n' ...
    '  At 9-19%% throttle, actual SFC may be 1.5-2x rated.\n' ...
    '  Descent fuel at rated SFC = %.1f kg; true value could be %.1f-%.1f kg.\n' ...
    '  ACTION: Obtain PT6A-60A SFC-vs-power map. Implement interpolation in\n' ...
    '  sim_descent for v5. Impact on total MTOW is small but documentable.\n\n'], ...
    SFCp_imp, seg.fuel_descent, seg.fuel_descent*1.5, seg.fuel_descent*2.0);
 
fprintf(['NOTE 3 — CLIMB SPEED TRANSITIONS FROM 67 m/s TO V_Pmin WITH ALTITUDE.\n' ...
    '  V_Pmin = sqrt(2Wg/(rho*S*CL_Pmin)) increases as rho drops with altitude.\n' ...
    '  At low altitude V_Pmin < 67 m/s: aircraft constrained to 67 m/s (suboptimal).\n' ...
    '  At high altitude V_Pmin > 67 m/s: aircraft uses V_Pmin for best ROC.\n' ...
    '  This causes the climb to cover more ground distance at altitude than\n' ...
    '  a fixed-67 m/s model would predict, reducing the remaining cruise range.\n\n']);
 
fprintf(['NOTE 4 — 450 NMI BUDGET IS SHARED ACROSS ALL FLIGHT PHASES.\n' ...
    '  v3 used 450 nmi as cruise range (climb/descent were separate).\n' ...
    '  v4 distributes the budget: d_cruise = 450 nmi - d_climb - d_descent.\n' ...
    '    d_climb   = %.1f nmi  (%.1f%% of budget)\n' ...
    '    d_cruise  = %.1f nmi  (%.1f%% of budget)\n' ...
    '    d_descent = %.1f nmi  (%.1f%% of budget)\n' ...
    '  The effective cruise segment is shorter in v4. Net fuel impact\n' ...
    '  depends on the ratio of climb vs cruise fuel burn rates.\n\n'], ...
    seg.d_climb/1852,   100*seg.d_climb/R_total_m, ...
    seg.d_cruise/1852,  100*seg.d_cruise/R_total_m, ...
    seg.d_descent/1852, 100*seg.d_descent/R_total_m);
 
fprintf(['NOTE 5 — RAYMER EWF IS A LOWER BOUND FOR THIS AIRCRAFT.\n' ...
    '  Raymer+STOL EWF = %.4f  vs  historical mean = %.3f.\n' ...
    '  Gap implies MTOW may be ~%.0f kg higher than Raymer predicts.\n' ...
    '  Raymer''s GA database excludes gravel gear, FIKI, and heavy STOL flaps.\n' ...
    '  Treat Raymer as lower bound, historical EWF check (~%.0f kg) as upper.\n\n'], ...
    EWF_final, mean(SE_EWF), W0_hist_check - W0_final, W0_hist_check);
 
fprintf(['NOTE 6 — LOITER AT SEA LEVEL IS CORRECT FOR MAX PROP ENDURANCE.\n' ...
    '  Higher density -> lower airspeed at same CL -> lower power required.\n' ...
    '  The 50-min loiter burns %.1f kg at CL_BE=%.4f, CD_BE=%.4f, L/D=%.2f.\n\n'], ...
    seg.fuel_loiter, CL_BE, CD_BE, CL_BE/CD_BE);
 
fprintf(['NOTE 7 — UPDATE PROCEDURE FOR NEXT DESIGN ITERATION.\n' ...
    '  Edit ONLY the user input block at the top of this script:\n' ...
    '    S, AR, CD0, e\n' ...
    '  All derived quantities recompute automatically. Suggested priorities:\n' ...
    '    1. Update S and AR from constraint analysis results\n' ...
    '    2. Refine CD0 from component drag buildup (Appendix A)\n' ...
    '    3. Add PT6A-60A partial-power SFC map (see Note 2)\n\n']);
 
 
%% =========================================================================
%  LOCAL FUNCTIONS
%  Must appear at the end of the script. Requires MATLAB R2016b or later.
%  Each function ends with 'end'. No code may appear after the first function.
% =========================================================================
 
 
% ─────────────────────────────────────────────────────────────────────────
function [fuel_kg, d_m, t_s, W_end] = sim_climb(W0, S, k, CD0, eta_p, ...
         P_shp_SI, SFCp_SI, V_min, h_target, g, rho_SL)
% SIM_CLIMB  Numerically integrates the climb from sea level to h_target.
%
%  SPEED LOGIC:
%    CL_Pmin = sqrt(3*CD0/k)             minimises power required
%    V_Pmin  = sqrt(2Wg / (rho*S*CL_Pmin))  maximises ROC
%    V       = max(V_min, V_Pmin)         enforce 67 m/s floor
%
%  POWER:  P_shaft = sigma * P_shp_SI  (full throttle)
%          P_A     = eta_p * P_shaft   (propulsive power)
%  ROC   = (P_A - P_R) / (W*g)
%  FUEL:   dW/dt = -P_shaft * SFCp_SI
%
%  ANGLE APPROXIMATION:
%    Ground distance = V * dt  (cos(gamma) ~ 1 for gamma < 10 deg).
%    At ROC~3 m/s and V~80 m/s: gamma~2.2 deg, cos=0.9993. Negligible error.
 
    dt    = 5;      % timestep [s]
    W     = W0;
    h     = 0;
    d     = 0;
    t     = 0;
    fuel  = 0;
 
    CL_Pmin = sqrt(3 * CD0 / k);   % constant — independent of altitude
 
    while h < h_target
 
        [~, rho, ~, ~] = StdAtmos(h);
        sigma   = rho / rho_SL;
 
        % Engine power at altitude (full throttle, simple sigma model)
        P_shaft = P_shp_SI * sigma;      % [W] shaft power
        P_A     = eta_p * P_shaft;       % [W] propulsive thrust power
 
        % Speed for best ROC at this altitude and weight
        V_Pmin = sqrt(2 * W * g / (rho * S * CL_Pmin));
        V      = max(V_min, V_Pmin);     % enforce minimum airspeed
 
        % Aerodynamics (L ~ W*g for small climb angles)
        CL  = W * g / (0.5 * rho * V^2 * S);
        CD  = CD0 + k * CL^2;
        D   = 0.5 * rho * V^2 * S * CD;
        P_R = D * V;
 
        % Rate of climb
        ROC = (P_A - P_R) / (W * g);    % [m/s]
 
        if ROC <= 0
            warning('sim_climb: ceiling at h=%.0f m (%.0f ft). Check W0 and engine power.', ...
                h, h/0.3048);
            break;
        end
 
        % Adaptive final step — do not overshoot target altitude
        dt_step = min(dt, (h_target - h) / ROC);
        dt_step = max(dt_step, 1e-4);
 
        dW_fuel = P_shaft * SFCp_SI * dt_step;   % [kg] per step
 
        h    = h    + ROC * dt_step;
        d    = d    + V   * dt_step;   % ground distance (cos gamma ~ 1)
        t    = t    + dt_step;
        W    = W    - dW_fuel;
        fuel = fuel + dW_fuel;
 
    end
 
    fuel_kg = fuel;
    d_m     = d;
    t_s     = t;
    W_end   = W;
 
end
 
 
% ─────────────────────────────────────────────────────────────────────────
function [fuel_kg, t_s, W_end] = sim_cruise(W0, S, k, CD0, eta_p, ...
         SFCp_SI, V, h, d_target, g)
% SIM_CRUISE  Numerically integrates cruise at constant altitude and speed.
%
%  Steady level flight: L = W*g, T = D.
%  Throttle is set each step so T = D (not tracked explicitly).
%  P_shaft = D*V / eta_p   (from T=D and P_A = eta_p*P_shaft = T*V)
%  FUEL: dW/dt = -P_shaft * SFCp_SI
%
%  This is the numerical equivalent of the Breguet range equation.
%  As W decreases, CL decreases, CD decreases, D decreases, and fuel
%  burn rate decreases. Agreement with Breguet < 0.1% at dt=10 s.
 
    dt    = 10;     % timestep [s]
    W     = W0;
    d     = 0;
    t     = 0;
    fuel  = 0;
 
    [~, rho, ~, ~] = StdAtmos(h);   % atmosphere constant at cruise altitude
 
    while d < d_target
 
        % Aerodynamics at current weight (L = W*g, level flight)
        CL  = W * g / (0.5 * rho * V^2 * S);
        CD  = CD0 + k * CL^2;
        D   = 0.5 * rho * V^2 * S * CD;    % drag = thrust [N]
 
        % Shaft power to maintain T = D
        P_shaft = D * V / eta_p;            % [W]
 
        % Adaptive final step
        dt_step = min(dt, (d_target - d) / V);
        dt_step = max(dt_step, 1e-4);
 
        dW_fuel = P_shaft * SFCp_SI * dt_step;  % [kg]
 
        d    = d    + V       * dt_step;
        t    = t    + dt_step;
        W    = W    - dW_fuel;
        fuel = fuel + dW_fuel;
 
    end
 
    fuel_kg = fuel;
    t_s     = t;
    W_end   = W;
 
end
 
 
% ─────────────────────────────────────────────────────────────────────────
function [fuel_kg, d_m, t_s, W_end] = sim_descent(W0, S, k, CD0, eta_p, ...
         P_shp_SI, SFCp_SI, P_A_sl, V_min, h_start, ROD_target, ...
         throttle_min, g, rho_SL)
% SIM_DESCENT  Numerically integrates descent at a fixed ROD target.
%
%  TARGET: ROD = ROD_target (5.080 m/s = 1000 fpm), V >= V_min (67 m/s).
%
%  CASE A — V_opt >= V_min  (fly faster, min throttle):
%    Find V_opt by bisection: P_R(V_opt) = W*g*ROD_target + P_A_min
%    Use V_opt with throttle = throttle_min.
%
%  CASE B — V_opt < V_min  (constrain to 67 m/s, throttle up):
%    Lock V = V_min = 67 m/s.
%    P_A = P_R(V_min) - W*g*ROD_target  (positive — caps ROD at target)
%    Throttle = P_A / (P_A_sl * sigma) > throttle_min.
%
%  For this aircraft, Case B applies throughout the entire descent:
%    P_R(67 m/s) > W*g*ROD_target at all altitudes and weights.
%    Throttle ranges from ~9% at 15,000 ft to ~19% at sea level.
%
%  FUEL: P_shaft = P_A / eta_p,   dW/dt = -P_shaft * SFCp_SI
%
%  CRITIQUE: SFC is higher at partial throttle (see Section 8 Note 2).
%  This model uses rated SFC throughout — descent fuel is underestimated.
 
    dt    = 5;      % timestep [s]
    W     = W0;
    h     = h_start;
    d     = 0;
    t     = 0;
    fuel  = 0;
 
    CL_Pmin = sqrt(3 * CD0 / k);   % constant — for V_Pmin bounds in bisection
 
    while h > 0
 
        [~, rho, ~, ~] = StdAtmos(h);
        sigma = rho / rho_SL;
 
        % Minimum propulsive power at baseline descent throttle
        P_A_min = throttle_min * P_A_sl * sigma;   % [W]
 
        % Power required target at V_opt with min throttle
        % ROD = (P_R - P_A_min)/(W*g) = ROD_target  →  P_R = W*g*ROD_target + P_A_min
        P_R_need = W * g * ROD_target + P_A_min;   % [W]
 
        % ── Bisection for V_opt on high-speed branch of P_R curve ────────
        %   P_R has a minimum at V_Pmin. We search V > V_Pmin (safer, faster).
        V_Pmin_loc = sqrt(2 * W * g / (rho * S * CL_Pmin));
        V_lo = V_Pmin_loc;
        V_hi = 300;            % [m/s] generous upper bound
 
        for bi = 1:60          % converges to < 1e-15 relative error
            V_mid = (V_lo + V_hi) / 2;
            CL_m  = W * g / (0.5 * rho * V_mid^2 * S);
            CD_m  = CD0 + k * CL_m^2;
            P_R_m = 0.5 * rho * V_mid^3 * S * CD_m;
            if P_R_m < P_R_need
                V_lo = V_mid;
            else
                V_hi = V_mid;
            end
        end
        V_opt = V_mid;
 
        % ── Apply speed constraint ────────────────────────────────────────
        if V_opt >= V_min
            % Case A: glide speed is above minimum — use it with min throttle
            V   = V_opt;
            P_A = P_A_min;
        else
            % Case B: glide speed too low — lock to V_min, throttle up
            V    = V_min;
            CL_v = W * g / (0.5 * rho * V^2 * S);
            CD_v = CD0 + k * CL_v^2;
            P_R_v = 0.5 * rho * V^3 * S * CD_v;
            P_A   = P_R_v - W * g * ROD_target;
            if P_A < 0; P_A = 0; end   % no thrust reverser
        end
 
        % Shaft power from propulsive power
        P_shaft = P_A / eta_p;          % [W]
 
        % Adaptive final step — do not descend below h = 0
        if h <= ROD_target * dt
            dt_step = h / ROD_target;
        else
            dt_step = dt;
        end
        dt_step = max(dt_step, 1e-4);
 
        dW_fuel = P_shaft * SFCp_SI * dt_step;   % [kg]
 
        h    = h    - ROD_target * dt_step;   % altitude decreases at fixed ROD
        d    = d    + V          * dt_step;   % ground distance at airspeed V
        t    = t    + dt_step;
        W    = W    - dW_fuel;
        fuel = fuel + dW_fuel;
 
    end
 
    fuel_kg = fuel;
    d_m     = d;
    t_s     = t;
    W_end   = W;
 
end