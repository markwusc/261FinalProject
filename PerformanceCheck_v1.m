%% PerformanceCheck_v1.m
% =========================================================================
% Arctic STOL Aircraft — RFP Performance Requirements Check
% Version: 1.0
%
% PURPOSE:
%   Evaluate three RFP performance requirements at the current design point
%   and produce a summary pass/fail table in metric units.
%
% RFP REQUIREMENTS CHECKED (from Design Brief screenshot):
%   1. Service Ceiling  >= 18,000 ft  (5,486 m)
%   2. Rate of Climb    >= 800 ft/min (4.064 m/s) at MTOW, ISA+5, sea level
%   3. Maximum Velocity (VNE proxy)   >= 180 KTAS (92.59 m/s) at sea level
%
% CALLS: StdAtmos(), RocCheck(), CeilingCheck(), IncomDrag()
%
% DESIGN CRITIQUE NOTES — READ BEFORE INTERPRETING RESULTS:
%
%   [C1] RocCheck() LACKS ISA+5 SUPPORT
%        The existing function hardcodes rho_sl = 1.225 (standard ISA).
%        The RFP explicitly requires ROC evaluated at ISA+5 (+5°C above
%        standard), which reduces density, increases V_Pmin, increases P_R,
%        and reduces P_A for a turboprop. Ignoring ISA+5 OVERestimates ROC
%        by roughly 1–2%. This script computes the correct ISA+5 values
%        manually and calls RocCheck() only as a secondary ISA-standard
%        sanity check.
%
%   [C2] CeilingCheck() USES AN INCORRECT SERVICE CEILING DEFINITION
%        The function checks whether P_A >= P_R at V_Dmin at exactly
%        18,000 ft. This only confirms the aircraft can sustain level flight
%        at that altitude — it does NOT locate the service ceiling.
%        By definition, service ceiling = altitude at which maximum ROC
%        falls to 100 ft/min (0.508 m/s). An aircraft barely sustaining
%        level flight at 18,000 ft would pass the existing check while
%        having effectively zero climb margin. This script uses the proper
%        sweep-and-interpolate method instead.
%
%   [C3] WING AREA INCONSISTENCY BETWEEN FILES
%        RaymerWeightEst_v3.m: S = 37.1 m²
%        CruiseOptimization_v3.m: S = 37.3 m²
%        These must be identical across all scripts. A 0.2 m² difference
%        propagates into weight, fuel burn, stall speed, and all
%        performance calculations. This script uses S = 37.3 m² (most
%        recent aerodynamics file) but this MUST be reconciled immediately.
%
%   [C4] TURBOPROP POWER SCALING WITH ISA+5
%        This script uses P_A = sigma * P_A_sl as the hot-day power
%        approximation (same method as the existing RocCheck function).
%        For a real PT6A-60A, power derates with both pressure ratio AND
%        temperature ratio. A more accurate derate would use the engine's
%        flat-rating temperature (approx. ISA+15 for most PT6 variants) —
%        below this, the engine is flat-rated and ISA+5 has minimal effect.
%        Verify with PT6A-60A performance charts before finalising ROC calc.
%
%   [C5] AR = 6.76 IS BELOW THE TYPICAL STOL RANGE
%        STOL aircraft typically use AR = 8–12 to maximise CL at low speed
%        and reduce induced drag during the approach. At AR = 6.76 the 
%        induced drag factor k = 1/(pi*e*AR) = 0.0527, which is noticeably
%        higher than at AR = 10 (k = 0.0354). This penalises ROC and 
%        service ceiling. Consider whether structural weight savings from a
%        shorter wing justify the aerodynamic penalty.
%
% UNITS: All internal calculations and table output in SI (metric).
%        Imperial equivalents shown parenthetically for reference only.
%
% Authors: AME 261 Design Team
% Date:    May 2026
% =========================================================================

clear; clc; close all;

%% =========================================================================
%  SECTION 1 — CURRENT DESIGN POINT
%  Source: CruiseOptimization_v3.m (most recent aerodynamic sizing file).
%  Update this block at the start of every design iteration so that all
%  performance checks automatically reflect the latest design.
% =========================================================================

W_To  = 40000;                    % Max takeoff weight (MTOW)        [N]
S     = 37.3;                     % Wing reference area              [m²]
AR    = 6.76;                     % Aspect ratio                     [-]
b     = sqrt(S * AR);             % Wing span                        [m]
CD0   = 0.0288;                   % Parasite drag coeff, clean       [-]
e     = 0.9;                      % Oswald efficiency factor         [-]

% Engine / propulsion
P_shp  = 1050;                    % Rated shaft power, PT6A-60A      [shp]
eta_p  = 0.80;                    % Propeller efficiency             [-]
% Available shaft power delivered to air at sea level, standard ISA.
% eta_p applied once here; all P_A values downstream inherit this.
P_A_sl = eta_p * P_shp * 745.7;  % Available propulsive power, SL   [W]

% Physical constants
g      = 9.81;                    % Gravitational acceleration       [m/s²]
R_gas  = 287;                     % Specific gas constant, air       [J/(kg·K)]
rho_sl = 1.225;                   % ISA sea level density            [kg/m³]
P_sl   = 101325;                  % ISA sea level pressure           [Pa]

% Derived aerodynamic constants — these do NOT change with altitude
k        = 1 / (pi * e * AR);    % Induced drag factor              [-]
% CL and CD at the minimum-power-required operating point.
% ROC is maximised when flying at V_Pmin (not V_LDmax), because maximum
% excess POWER (not minimum drag) produces maximum rate of climb.
CL_Pmin  = sqrt(3 * CD0 / k);    % CL at min power required         [-]
CD_Pmin  = 4 * CD0;              % CD at min power required = 4*CD0 [-]

fprintf('=== SECTION 1: CURRENT DESIGN POINT ===\n')
fprintf('  W_To    = %6.0f N  (%5.1f kg)\n',  W_To, W_To/g)
fprintf('  S       = %6.2f m²\n', S)
fprintf('  AR      = %6.2f  |  b = %.2f m\n', AR, b)
fprintf('  CD0     = %6.4f  |  e = %.2f\n', CD0, e)
fprintf('  k       = %6.5f  (induced drag factor)\n', k)
fprintf('  CL_Pmin = %6.4f  |  CD_Pmin = %.4f\n', CL_Pmin, CD_Pmin)
fprintf('  P_A_sl  = %6.1f kW  (%d shp × %.2f eta_p)\n\n', ...
    P_A_sl/1e3, P_shp, eta_p)

%% =========================================================================
%  SECTION 2 — RFP THRESHOLDS (metric)
%  All limits converted to SI here so downstream comparisons are clean.
%  Imperial equivalents retained parenthetically for traceability.
% =========================================================================

h_svc_req    = 18000 * 0.3048;   % Service ceiling requirement      [m]  (18,000 ft)
ROC_req_ms   =   800 * 0.00508;  % ROC requirement                  [m/s] (800 ft/min)
V_VNE_req_ms =   180 * 0.5144;  % VNE requirement                  [m/s] (180 KTAS)
ROC_svc_def  =   100 * 0.00508;  % Service ceiling ROC threshold    [m/s] (100 ft/min)
% Note: 1 ft/min = 0.3048/60 m/s = 0.00508 m/s

fprintf('=== SECTION 2: RFP THRESHOLDS ===\n')
fprintf('  Service Ceiling  >= %.0f m  (18,000 ft)\n',  h_svc_req)
fprintf('  Rate of Climb    >= %.4f m/s  (800 ft/min), MTOW ISA+5\n', ROC_req_ms)
fprintf('  VNE              >= %.3f m/s  (180 KTAS)\n\n', V_VNE_req_ms)

%% =========================================================================
%  SECTION 3 — ISA+5 DENSITY CORRECTION AT SEA LEVEL
%
%  ISA+5 (also written ISA+5°C or delta-ISA = +5): the ambient temperature
%  at every altitude is 5 K higher than the standard ISA value.
%  Pressure is assumed to follow the same ISA profile (i.e., the hydrostatic
%  column is unchanged — standard practice in preliminary performance work).
%
%  At sea level:
%    T_ISA5 = 288.15 + 5 = 293.15 K
%    rho_ISA5 = P_sl / (R * T_ISA5)                     [kg/m³]
%
%  This lower density has two adverse effects on ROC:
%    (a) V_Pmin increases (heavier equivalent air — more speed required)
%    (b) P_A decreases (turboprop ingests less mass flow per unit time)
%  Both degrade ROC. The RFP uses ISA+5 precisely to capture hot-day
%  performance — the design must pass even on a warm day.
% =========================================================================

T_ISA_sl    = 288.15;                          % ISA sea level temperature   [K]
T_ISA5_sl   = T_ISA_sl + 5;                   % ISA+5 sea level temperature [K]
rho_ISA5_sl = P_sl / (R_gas * T_ISA5_sl);     % ISA+5 sea level density     [kg/m³]
sigma_ISA5  = rho_ISA5_sl / rho_sl;           % ISA+5 density ratio         [-]

fprintf('=== SECTION 3: ISA+5 CONDITIONS AT SEA LEVEL ===\n')
fprintf('  T_ISA (SL)      = %.2f K  (%.2f °C)\n', T_ISA_sl,  T_ISA_sl  - 273.15)
fprintf('  T_ISA+5 (SL)    = %.2f K  (%.2f °C)\n', T_ISA5_sl, T_ISA5_sl - 273.15)
fprintf('  rho_ISA5        = %.4f kg/m³  (ISA: %.4f kg/m³, delta = %.4f)\n', ...
    rho_ISA5_sl, rho_sl, rho_ISA5_sl - rho_sl)
fprintf('  sigma_ISA5      = %.5f  (%.3f%% of ISA density)\n\n', ...
    sigma_ISA5, sigma_ISA5 * 100)

%% =========================================================================
%  SECTION 4 — RATE OF CLIMB AT MTOW, ISA+5, SEA LEVEL
%
%  Method: ROC_max = (P_A - P_R_min) / W
%  This is the standard ROC equation for a propeller aircraft, where P_R_min
%  is the minimum power required (evaluated at V_Pmin). Flying at V_Pmin
%  gives the maximum rate of climb — this is what a pilot targets for best
%  rate of climb in an underpowered or climb-critical scenario.
%
%  The existing RocCheck() function implements the same method but at ISA
%  standard conditions. We call it below for reference, then compute the
%  correct ISA+5 result manually.
% =========================================================================

% --- ISA+5 sea level quantities ---
% V_Pmin increases with lower density (same CL_Pmin, but lower rho means
% the aircraft must fly faster to generate the same lift)
V_Pmin_ISA5 = sqrt(2 * W_To / (rho_ISA5_sl * S * CL_Pmin));  % [m/s]
q_Pmin_ISA5 = 0.5 * rho_ISA5_sl * V_Pmin_ISA5^2;             % [Pa]
D_Pmin_ISA5 = q_Pmin_ISA5 * S * CD_Pmin;                      % [N]
P_R_ISA5    = D_Pmin_ISA5 * V_Pmin_ISA5;                      % [W]

% Turboprop power output scales with sigma (density ratio).
% At ISA+5 sea level the engine ingests ~1.7% less air by mass → ~1.7% less power.
P_A_ISA5    = sigma_ISA5 * P_A_sl;                            % [W]

ROC_ISA5_ms  = (P_A_ISA5 - P_R_ISA5) / W_To;                 % [m/s]
ROC_ISA5_fpm = ROC_ISA5_ms / 0.00508;                         % [ft/min]

% --- Call the existing RocCheck() for ISA standard (h=0) ---
% This gives a boolean check at sea level ISA. Note: does NOT apply ISA+5.
roc_bool_ISA = RocCheck(P_A_sl, W_To, S, AR, CD0, 0);

fprintf('=== SECTION 4: RATE OF CLIMB — MTOW, ISA+5, SEA LEVEL ===\n')
fprintf('  V_Pmin (ISA+5 SL) = %.2f m/s  (%.1f KTAS)\n', ...
    V_Pmin_ISA5, V_Pmin_ISA5/0.5144)
fprintf('  P_R at V_Pmin     = %.2f kW\n', P_R_ISA5/1e3)
fprintf('  P_A (ISA+5 SL)    = %.2f kW  (%.2f kW × sigma=%.5f)\n', ...
    P_A_ISA5/1e3, P_A_sl/1e3, sigma_ISA5)
fprintf('  ROC (ISA+5, SL)   = %.3f m/s  = %.1f ft/min\n', ...
    ROC_ISA5_ms, ROC_ISA5_fpm)
fprintf('  RFP requirement   = %.3f m/s  = 800 ft/min\n', ROC_req_ms)
fprintf('  Margin            = %.3f m/s  = %.1f ft/min\n', ...
    ROC_ISA5_ms - ROC_req_ms, ROC_ISA5_fpm - 800)
fprintf('  RocCheck() ISA std (ref only, no ISA+5) : %s\n\n', ...
    boolStr(roc_bool_ISA))

%% =========================================================================
%  SECTION 5 — SERVICE CEILING
%
%  Service ceiling is the altitude at which the aircraft's maximum rate of
%  climb (ROC_max = (P_A - P_R_min)/W) falls to 100 ft/min (0.508 m/s).
%  This is the FAA/ICAO standard definition and what "Service Ceiling" means
%  in the RFP. It is found by sweeping altitude and interpolating.
%
%  Note: ISA standard atmosphere is used for the ceiling sweep (the RFP
%  does not specify ISA+5 for the ceiling condition). W = MTOW throughout
%  (most conservative — lightest weight would give a higher ceiling).
%
%  The existing CeilingCheck() is called for reference but its result is a
%  boolean at exactly 18,000 ft with the level-flight check (see [C2] above).
%  The true service ceiling from the sweep may differ from this.
% =========================================================================

% Sweep altitudes from 0 to 12,000 m in ISA standard
% 12,000 m (39,370 ft) is a safe upper bound for a STOL turboprop
n_h      = 5000;
h_vec    = linspace(0, 12000, n_h);    % [m]
ROC_vec  = zeros(1, n_h);             % ROC_max at each altitude [m/s]

for i = 1:n_h
    [~, rho_i, ~, ~] = StdAtmos(h_vec(i));
    sigma_i = rho_i / rho_sl;

    % V at minimum power required — where ROC is maximised at each altitude
    V_Pmin_i = sqrt(2 * W_To / (rho_i * S * CL_Pmin));   % [m/s]
    q_Pmin_i = 0.5 * rho_i * V_Pmin_i^2;                 % [Pa]
    D_Pmin_i = q_Pmin_i * S * CD_Pmin;                    % [N]
    P_R_i    = D_Pmin_i * V_Pmin_i;                       % [W]

    % Turboprop available power decreases with density ratio
    P_A_i    = sigma_i * P_A_sl;                          % [W]

    % Maximum ROC at this altitude
    ROC_vec(i) = (P_A_i - P_R_i) / W_To;                 % [m/s]
end

% Interpolate to find service ceiling (ROC = 100 ft/min = 0.508 m/s)
% and absolute ceiling (ROC = 0)
% interp1 needs a monotonically decreasing ROC — the sweep must be
% descending (ROC decreases with altitude) which it will be for a
% well-designed aircraft. Reverse for interp1 compatibility.
h_svc_ceil  = interp1(flip(ROC_vec), flip(h_vec), ROC_svc_def, 'linear', NaN);
h_abs_ceil  = interp1(flip(ROC_vec), flip(h_vec), 0,            'linear', NaN);

if isnan(h_svc_ceil)
    warning('[PerformanceCheck] Service ceiling > 12,000 m — extend h_vec sweep range.')
    h_svc_ceil = h_vec(end);
end
if isnan(h_abs_ceil)
    warning('[PerformanceCheck] Absolute ceiling > 12,000 m — extend h_vec sweep range.')
    h_abs_ceil = h_vec(end);
end

% Call existing CeilingCheck() for boolean reference (not used in table)
ceil_bool = CeilingCheck(P_A_sl, W_To, S, AR, CD0);

fprintf('=== SECTION 5: SERVICE CEILING ===\n')
fprintf('  Service ceiling  (ROC=%.3f m/s) = %6.0f m  (%.0f ft)\n', ...
    ROC_svc_def, h_svc_ceil, h_svc_ceil/0.3048)
fprintf('  Absolute ceiling (ROC=0)         = %6.0f m  (%.0f ft)\n', ...
    h_abs_ceil, h_abs_ceil/0.3048)
fprintf('  RFP requirement                  = %6.0f m  (18,000 ft)\n', h_svc_req)
fprintf('  Margin                           = %+6.0f m  (%+.0f ft)\n', ...
    h_svc_ceil - h_svc_req, (h_svc_ceil - h_svc_req)/0.3048)
fprintf('  CeilingCheck() pass/fail (ref)   = %s\n\n', boolStr(ceil_bool))

%% =========================================================================
%  SECTION 6 — MAXIMUM LEVEL FLIGHT SPEED (VNE PROXY)
%
%  Maximum level flight speed V_max is found at sea level (sigma=1) where
%  P_A is maximum. This gives the highest achievable indicated airspeed and
%  therefore the highest structural loads — the appropriate condition for
%  setting or validating VNE.
%
%  Method: sweep velocity, compute P_R = IncomDrag() × V, find upper
%  crossing of P_A = P_R (power equilibrium at high speed).
%
%  IMPORTANT CAVEAT: Formal VNE is a structural limit, not a performance one.
%  In FARs, VNE is set at 0.9 × V_D (dive speed), where V_D is established
%  through flight test. This analysis finds the aerodynamic V_max, which
%  provides the necessary condition — the airframe must be structurally
%  cleared to at least 180 KTAS.
%
%  IncomDrag() uses an incompressible drag polar. For V > ~250 KTAS
%  compressibility effects become significant. For this STOL turboprop
%  operating below Mach 0.4, incompressible drag is an acceptable model.
% =========================================================================

% Sweep from a low speed (above stall) up to 200 m/s
V_sweep   = linspace(20, 200, 5000);   % [m/s]
P_R_sweep = zeros(1, length(V_sweep)); % [W]

% P_A at sea level is constant (sigma = 1)
P_A_SL = P_A_sl;   % [W]

for i = 1:length(V_sweep)
    % IncomDrag() returns total drag force [N] given speed, geometry, density
    [D_i, ~, ~, ~] = IncomDrag(V_sweep(i), b, S, e, CD0, W_To, rho_sl);
    P_R_sweep(i)   = D_i * V_sweep(i);   % Power required [W]
end

% Excess power: positive when P_A > P_R (aircraft can still accelerate)
P_excess = P_A_SL - P_R_sweep;

% The maximum level flight speed is the LAST velocity at which the aircraft
% can still sustain level flight (upper crossing, P_excess goes + → -)
sign_changes = diff(sign(P_excess));
upper_idx    = find(sign_changes < 0, 1, 'last');   % Last + to - crossing

if isempty(upper_idx)
    warning('[PerformanceCheck] V_max crossing not found — P_A > P_R across entire sweep. Increase upper velocity limit.')
    V_max = V_sweep(end);
else
    % Linear interpolation between the two bracketing points for precision
    V_max = interp1(P_excess(upper_idx:upper_idx+1), ...
                    V_sweep(upper_idx:upper_idx+1), 0, 'linear');
end

V_max_ktas = V_max / 0.5144;   % [KTAS]

fprintf('=== SECTION 6: MAXIMUM LEVEL FLIGHT SPEED — SEA LEVEL, MTOW ===\n')
fprintf('  V_max (level flight, SL) = %.2f m/s  =  %.1f KTAS\n', V_max, V_max_ktas)
fprintf('  VNE requirement          = %.2f m/s  = 180.0 KTAS\n', V_VNE_req_ms)
fprintf('  Margin                   = %+.2f m/s  (%+.1f KTAS)\n', ...
    V_max - V_VNE_req_ms, V_max_ktas - 180)

%% =========================================================================
%  SECTION 7 — PERFORMANCE SUMMARY TABLE
%  Columns: | Condition | RFP Requirement | Current Value | Status |
%  All primary values in metric; imperial in parentheses.
% =========================================================================

fprintf('\n')
fprintf('%s\n', repmat('=', 1, 120))
fprintf('  PERFORMANCE REQUIREMENTS SUMMARY\n')
fprintf('%s\n', repmat('=', 1, 120))
fprintf('  %-38s | %-32s | %-32s | %s\n', ...
    'Condition', 'RFP Requirement', 'Current Value', 'Status')
fprintf('  %s\n', repmat('-', 1, 114))

% --- Row 1: Service Ceiling ---
req1 = sprintf('>= %.0f m  (>= 18,000 ft)',     h_svc_req);
cur1 = sprintf('%.0f m  (%.0f ft)',              h_svc_ceil, h_svc_ceil/0.3048);
pass1 = h_svc_ceil >= h_svc_req;
fprintf('  %-38s | %-32s | %-32s | %s\n', ...
    'Service Ceiling [ISA, ROC=0.508 m/s]', req1, cur1, passFailStr(pass1))

% --- Row 2: Rate of Climb at MTOW, ISA+5 ---
req2 = sprintf('>= %.3f m/s  (>= 800 ft/min)',  ROC_req_ms);
cur2 = sprintf('%.3f m/s  (%.0f ft/min)',        ROC_ISA5_ms, ROC_ISA5_fpm);
pass2 = ROC_ISA5_ms >= ROC_req_ms;
fprintf('  %-38s | %-32s | %-32s | %s\n', ...
    'ROC [MTOW, ISA+5, sea level]', req2, cur2, passFailStr(pass2))

% --- Row 3: VNE (Maximum Level Flight Speed) ---
req3 = sprintf('>= %.2f m/s  (>= 180 KTAS)',    V_VNE_req_ms);
cur3 = sprintf('%.2f m/s  (%.1f KTAS)',          V_max, V_max_ktas);
pass3 = V_max >= V_VNE_req_ms;
fprintf('  %-38s | %-32s | %-32s | %s\n', ...
    'Max Level Speed / VNE proxy [SL, MTOW]', req3, cur3, passFailStr(pass3))

fprintf('  %s\n', repmat('-', 1, 114))
fprintf('%s\n\n', repmat('=', 1, 120))

% --- Overall result ---
all_pass = pass1 && pass2 && pass3;
if all_pass
    fprintf('  OVERALL RESULT: ALL REQUIREMENTS MET\n\n')
else
    fprintf('  OVERALL RESULT: ONE OR MORE REQUIREMENTS NOT MET — DESIGN ITERATION REQUIRED\n\n')
end

fprintf('NOTES:\n')
fprintf('  [1] Service ceiling found by sweep-and-interpolate (ROC drops to 100 ft/min).\n')
fprintf('      CeilingCheck() (pass/fail at 18,000 ft level flight) is a necessary\n')
fprintf('      but not sufficient condition — do not rely on it alone.\n')
fprintf('  [2] ROC evaluated at ISA+5 per RFP. RocCheck() uses ISA standard only.\n')
fprintf('      See design critique [C1] and [C4] in file header.\n')
fprintf('  [3] VNE is formally a structural limit. V_max (level flight, SL) is used\n')
fprintf('      as a proxy. True VNE requires dive speed analysis in structural sizing.\n')
fprintf('  [4] Wing area inconsistency: RaymerWeightEst_v3 uses S=37.1 m²; this\n')
fprintf('      script uses S=37.3 m² from CruiseOptimization_v3. Reconcile before\n')
fprintf('      submitting final performance values.\n')

%% =========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================

function s = passFailStr(tf)
    % Returns formatted pass/fail string for table output
    if tf
        s = 'PASS ✓';
    else
        s = 'FAIL ✗';
    end
end

function s = boolStr(tf)
    % Returns simple boolean string for diagnostic output
    if tf
        s = 'true  (PASS)';
    else
        s = 'false (FAIL)';
    end
end
