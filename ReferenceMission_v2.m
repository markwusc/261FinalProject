% ReferenceMission_v1.m
% =========================================================================
% PUDO 670 — Reference Mission Performance Analysis
% Configuration 1  |  90% Payload  |  450 nmi Total Range
% =========================================================================
%
% MISSION PROFILE (climb, cruise, descent plotted; ground ops excluded):
%   (1) Warm-up & taxi        Raymer ff = 0.990
%   (2) Takeoff               Raymer ff = 0.995
%   (3) Climb  SL to 4572 m   Raymer ff = 0.980 for FUEL
%                              Kinematic ROC sim  for DISTANCE / TIME only
%   (4) Cruise 67 m/s 4572 m  Breguet ff from required cruise range
%   (5) Descent 1000 fpm       Raymer ff = 0.990 for FUEL
%                              Drag balance       for THROTTLE CHART only
%   (6) Landing                Raymer ff = 0.992
%   [Reserve] 45-min loiter    Breguet endurance at sea level, best CL
%
% ITERATION STRATEGY:
%   W6_target (weight after landing rollout) is pre-solved from the loiter
%   constraint: burn exactly to W_dry in 45 min at sea level.
%   W_fuel is iterated via first-order Newton step until W6_computed = W6_target.
%   ff_cruise updates each iteration: cruise range = 450 nmi - d_climb - d_descent,
%   and d_climb depends on W2 = W0*ff_warmup*ff_takeoff which depends on W0.
%
% KEY DESIGN DECISIONS:
%   - Climb TAS = 70 m/s fixed (team decision)
%   - Weight held CONSTANT during kinematic climb sim (no per-dt fuel burn)
%   - CD0 = 0.0288 for ALL phases (clean config assumed after liftoff)
%   - Power lapse: P_A(h) = sigma(h) * P_A_SL (linear, project convention)
%   - Loiter at sea level (conservative — max fuel burn rate)
%   - Cruise L/D = 12.50 (from CruiseOptimization_v3.m at 130 KTAS / 4572 m)
%
% CALLS:   StdAtmos(h)  returns [P, rho, T, mu] for scalar h [m]
%
% OUTPUTS:
%   Command window — weight statement, fuel fractions, distances, timing
%   Figure 1       — Altitude & TAS vs mission time, colored by distance
%   Figure 2       — Throttle vs time: climb / cruise / descent subplots
%
% FLAG INDEX (search flag number):
%   FLAG-1  OEW back-derived from Raymer; update if design changes
%   FLAG-2  ff_climb and kinematic sim are intentionally decoupled
%   FLAG-3  V_horiz constraint checked; warning printed if violated
%
% AUTHORS: PUDO 670 Design Team
% VERSION: 1.0  |  May 2026
% =========================================================================

clear; clc; close all;


%% =========================================================================
%  SECTION 1 — AIRCRAFT DESIGN CONSTANTS
%  Source: RaymerWeightEst_v3.m and CruiseOptimization_v3.m (converged).
%  Update this block whenever S, AR, CD0, e, or engine change.
% =========================================================================

g       = 9.81;            % gravitational acceleration                [m/s^2]
rho_SL  = 1.225;           % ISA sea-level air density                 [kg/m^3]

% --- Wing & Aerodynamics -------------------------------------------------
S    = 37.1;               % reference wing area                       [m^2]
AR   = 6.76;               % aspect ratio                              [-]
e    = 0.90;               % Oswald efficiency factor (clean)          [-]
CD0  = 0.0288;             % parasite drag, clean config (all phases)  [-]
%   FLAG-2: CD0 = 0.0288 applied to ALL flight phases per team decision.
%   Flaps assumed retracted immediately after liftoff. Revisit if a
%   climb-flap CD0 is adopted later.
k    = 1 / (pi * AR * e); % induced drag factor                        [-]

% --- Propulsion: PT6A-60A ------------------------------------------------
P_shp   = 1050;                           % rated shaft power          [shp]
P_SL_W  = P_shp * 745.7;                  % sea-level shaft power      [W]
eta_p   = 0.80;                           % propeller efficiency        [-]
P_A_SL  = eta_p * P_SL_W;                 % sea-level propulsive power  [W]
%   Thrust:      T = P_A(h) / V
%   Power lapse: P_A(h) = sigma(h) * P_A_SL  (linear, project convention)
%   eta_p applied ONCE here — do not apply again inside the sim loops.

%   SFC: lb/(hp*hr) to kg/(W*s)
%   1 lb/(hp*hr) * (0.4536 kg/lb) / (745.7 W/hp * 3600 s/hr)
SFCp_imp = 0.548;                                      % [lb/(hp*hr)]
SFCp_SI  = SFCp_imp * 0.4536 / (745.7 * 3600);        % [kg/(W*s)]

% --- Cruise (from CruiseOptimization_v3.m) -------------------------------
LD_cruise = 12.50;   % L/D at 130 KTAS cruise, from CruiseOptimization_v3.m [-]
V_cruise  = 67.0;    % cruise TAS (~130 KTAS)                         [m/s]
h_cruise  = 4572.0;  % cruise altitude (15,000 ft)                    [m]

% --- Best-endurance aerodynamics (loiter) --------------------------------
%   For propeller aircraft, max endurance at CL_BE = sqrt(3*CD0/k).
%   At this CL: CD_BE = 4*CD0 (induced drag = 3x parasite).
%   Reference: Anderson Intro to Flight Sec. 6.6; RaymerWeightEst_v3.m Sec. 3.
CL_BE = sqrt(3 * CD0 / k);  % best-endurance CL                       [-]
CD_BE = 4 * CD0;             % best-endurance CD (= 4*CD0)             [-]
LD_BE = CL_BE / CD_BE;       % best-endurance L/D                      [-]

if CL_BE > 2.4
    % Stall check: CL_BE must not exceed maximum achievable CL
    warning('PUDO670:LoiterStall', ...
        'CL_BE = %.4f exceeds clean CL_max = 2.4. Clamping for loiter.', CL_BE);
    CL_BE = 2.4;
    CD_BE = CD0 + k * CL_BE^2;
    LD_BE = CL_BE / CD_BE;
end

%   Breguet endurance constant K_loiter [N^0.5]
%   E = K_loiter * [W_end^(-0.5) - W_start^(-0.5)]  (W in Newtons, E in s)
%   See RaymerWeightEst_v3.m header for full unit derivation.
K_loiter = (eta_p / (g * SFCp_SI)) * (CL_BE^1.5 / CD_BE) * sqrt(2 * rho_SL * S);


%% =========================================================================
%  SECTION 2 — MISSION PARAMETERS AND WEIGHT SETUP
% =========================================================================

% --- Total required range ------------------------------------------------
R_total_nmi = 450;                    % RFP ground-track requirement    [nmi]
R_total_m   = R_total_nmi * 1852;     % converted to metres             [m]

% --- Payload: Config 1 at 90% --------------------------------------------
%   Full Config 1 payload from PayloadBuildUp_v1.csv:
%     9 pax body (816.47 kg) + baggage (204.12 kg) + 1 FA (90.72 kg)
%     + FA baggage (9.07 kg) + 10 seats (100.00 kg) = 1220.37 kg total
W_pld_C1_full = 1220.37;              % Config 1 full payload           [kg]
W_payload_ref = 0.90 * W_pld_C1_full; % 90% per RFP reference mission   [kg]

% --- Operating Empty Weight (OEW) ----------------------------------------
%   OEW = structural empty + crew (2 pilots at 181.44 kg).
%   FLAG-1: OEW is back-derived from RaymerWeightEst_v3.m single-engine
%   convergence (W0=4077 kg, zeta=0.1313, W_fuel=535 kg, W_payload+crew=1580 kg).
%   OEW is a fixed airframe property — does NOT change between missions.
%   Re-run RaymerWeightEst_v3.m and update W_OEW_kg if S, AR, CD0, or e change.
W_OEW_kg  = 2150.0;                   % OEW (structural + crew)         [kg]
W_crew_kg = 2 * 200 * 0.4536;         % 2 pilots x 200 lb each (bookkeeping) [kg]
%   NOTE: W_crew_kg is EMBEDDED in W_OEW_kg. Do not add it separately.

% --- Non-fuel weight (constant across all iterations) --------------------
W_dry_kg = W_OEW_kg + W_payload_ref;  % total non-fuel weight           [kg]
W_dry_N  = W_dry_kg * g;              % in Newtons (for endurance eq.)  [N]

% --- Raymer historical fuel fractions ------------------------------------
%   Source: Raymer Table 3.2; RaymerWeightEst_v3.m project conventions.
ff_warmup  = 0.990;  % warm-up and taxi (~10 min)
ff_takeoff = 0.995;  % STOL takeoff over 50 ft obstacle
ff_climb   = 0.980;  % climb to 4572 m — Raymer historical, propeller aircraft
%   FLAG-2: ff_climb = 0.980 governs FUEL in climb. The kinematic ROC sim
%   gives DISTANCE and TIME only. Decoupled by team decision because:
%   (a) Raymer ff is more defensible for weight estimation at this phase;
%   (b) per-dt weight updates require knowing fuel fraction from the sim
%       itself, which is circular without a tighter integration scheme.
ff_descent = 0.990;  % descent — Raymer historical
ff_landing = 0.992;  % landing rollout and taxi-in

% --- Config 2 fuel fraction (seed for iteration) -------------------------
%   From RaymerWeightEst_v3.m single-engine convergence with Config 2 payload.
%   The reference mission will converge to a lower zeta (lighter payload).
zeta_C2 = 0.1313;

% --- Loiter and climb simulation parameters ------------------------------
t_loiter_s = 45 * 60;  % loiter reserve duration at sea level          [s]
V_climb    = 70.0;      % fixed climb TAS (team decision)              [m/s]
dt_climb   = 1.0;       % kinematic climb sim time step                [s]


%% =========================================================================
%  SECTION 3 — DESCENT GEOMETRY (weight-independent)
%
%  The descent is fully defined by two constraints:
%    (1) Vertical rate  = 1000 ft/min
%    (2) Horizontal speed = 67 m/s
%  From these, glide speed, angle, time, and horizontal distance follow
%  without knowing the aircraft weight.
%
%  Whether the aircraft needs throttle is checked in Section 8 via the
%  along-path force balance. That result feeds the throttle chart only;
%  descent fuel comes from ff_descent (Raymer).
% =========================================================================

ROD_fpm     = 1000;                          % rate of descent          [ft/min]
ROD_ms      = ROD_fpm * 0.3048 / 60;         % rate of descent          [m/s]
V_horiz_req = 67.0;                           % required horiz speed     [m/s]

%   Glide speed: the TAS giving 67 m/s horizontal AND 1000 fpm vertical.
V_descent  = sqrt(V_horiz_req^2 + ROD_ms^2); % glide speed TAS          [m/s]
gamma_desc = atan2(ROD_ms, V_horiz_req);      % descent angle (+= nose down) [rad]

t_desc_s   = h_cruise / ROD_ms;              % total descent time        [s]
t_desc_min = t_desc_s / 60;                  %                           [min]
d_desc_m   = V_horiz_req * t_desc_s;         % horizontal descent dist   [m]


%% =========================================================================
%  SECTION 4 — LOITER RESERVE: TARGET LANDING WEIGHT W6_target
%
%  Goal: arrive at end of landing rollout with EXACTLY enough fuel to
%  sustain 45 min of loiter at best endurance, burning to W_dry (zero fuel).
%
%  Breguet endurance (W in Newtons):
%    E = K_loiter * [W_dry_N^(-0.5) - W6_N^(-0.5)]
%  Solve for W6_N:
%    W6_N^(-0.5) = W_dry_N^(-0.5) - E / K_loiter
%    W6_N        = (W_dry_N^(-0.5) - E / K_loiter)^(-2)
%
%  W6_target is FIXED across iterations — W_dry does not change between them.
% =========================================================================

W6_target_N      = 1 / (W_dry_N^(-0.5) - t_loiter_s / K_loiter)^2;
W6_target_kg     = W6_target_N / g;
W_loiter_fuel_kg = W6_target_kg - W_dry_kg;  % loiter fuel mass         [kg]
ff_loiter        = W_dry_N / W6_target_N;     % loiter fuel fraction     [-]


%% =========================================================================
%  SECTION 5 — FUEL WEIGHT ITERATION
%
%  Goal: find W_fuel such that W6_computed = W6_target.
%
%  Each iteration:
%   a) W0 = W_dry + W_fuel
%   b) W1 = W0*ff_warmup,  W2 = W1*ff_takeoff  (weight at base of climb)
%   c) Kinematic climb sim (scalars only, W = W2*g fixed throughout):
%        T = P_A(h)/V,  D = q*S*(CD0+k*CL^2),  ROC = V*(T-D)/W
%        Accumulate d_climb_m, t_climb_s
%   d) d_cruise = R_total - d_climb - d_descent
%   e) ff_cruise = exp(-d_cruise * g * SFCp_SI / (eta_p * LD_cruise))
%   f) W3=W2*ff_climb, W4=W3*ff_cruise, W5=W4*ff_descent, W6=W5*ff_landing
%   g) residual = W6_computed - W6_target
%   h) Newton update: W_fuel -= residual / ff_product
%      (from W6 ~ (W_dry+W_fuel)*ff_product, dW6/dW_fuel ~ ff_product)
%
%  Convergence is fast (~5-8 steps) — d_climb changes weakly with W_fuel.
% =========================================================================

tol_kg   = 0.01;   % convergence tolerance                             [kg]
max_iter = 100;    % safety cap

%   Seed: W_fuel = zeta * W0 = zeta*(W_dry+W_fuel) -> W_fuel = zeta*W_dry/(1-zeta)
W_fuel_kg = zeta_C2 * W_dry_kg / (1 - zeta_C2);

% Print header
fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║       PUDO 670  —  REFERENCE MISSION ANALYSIS                       ║\n');
fprintf('║       Config 1  |  90%% Payload  |  450 nmi Total Range              ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  DESCENT GEOMETRY                                                    ║\n');
fprintf('║  %-40s : %8.4f m/s (%.0f fpm)      ║\n', 'Rate of descent', ROD_ms, ROD_fpm);
fprintf('║  %-40s : %8.4f m/s               ║\n', 'Required horizontal speed', V_horiz_req);
fprintf('║  %-40s : %8.4f m/s               ║\n', 'Computed glide speed TAS', V_descent);
fprintf('║  %-40s : %8.4f deg               ║\n', 'Descent angle', rad2deg(gamma_desc));
fprintf('║  %-40s : %8.2f s  (%.2f min)     ║\n', 'Time to descend', t_desc_s, t_desc_min);
fprintf('║  %-40s : %8.2f km (%.2f nmi)     ║\n', 'Horiz distance descent', d_desc_m/1e3, d_desc_m/1852);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  LOITER RESERVE PARAMETERS                                           ║\n');
fprintf('║  %-40s : %8.4f                ║\n', 'CL_BE (best endurance)', CL_BE);
fprintf('║  %-40s : %8.4f                ║\n', 'CD_BE', CD_BE);
fprintf('║  %-40s : %8.4f                ║\n', 'L/D at best endurance', LD_BE);
fprintf('║  %-40s : %8.4e [N^0.5]       ║\n', 'K_loiter', K_loiter);
fprintf('║  %-40s : %8.2f kg             ║\n', 'Dry weight W_dry (OEW+payload)', W_dry_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'Required landing weight W6_target', W6_target_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'Loiter fuel reserve', W_loiter_fuel_kg);
fprintf('║  %-40s : %8.6f                ║\n', 'ff_loiter = W_dry/W6_target', ff_loiter);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  FUEL ITERATION  (tol = %.3f kg,  init guess = %.2f kg)          ║\n', tol_kg, W_fuel_kg);
fprintf('║  %4s | %9s | %9s | %9s | %9s | %9s  ║\n', 'Iter','W_fuel','W0','W6_comp','W6_tgt','Resid');
fprintf('║  %s  ║\n', repmat('-',1,64));

converged   = false;
residual_kg = NaN;
% These are initialized here so they exist if the loop exits unexpectedly
W0_kg = 0; W1_kg = 0; W2_kg = 0; W3_kg = 0;
W4_kg = 0; W5_kg = 0; W6_kg = 0;
d_climb_m = 0; t_climb_s = 0;
ROC_avg_ms = 0; ROC_avg_fpm = 0; t_climb_min = 0;
d_cruise_m = 0; d_cruise_km = 0; d_cruise_nmi = 0;
ff_cruise = 1; Vh_min = 67;

for iter = 1:max_iter

    % (a) Takeoff weight for this iteration
    W0_kg = W_dry_kg + W_fuel_kg;   % [kg]

    % (b) Ground segment weights
    W1_kg = W0_kg  * ff_warmup;    % after warm-up and taxi
    W2_kg = W1_kg  * ff_takeoff;   % after takeoff = weight at base of climb
    W2_N  = W2_kg  * g;            % [N] held constant in climb sim

    % (c) Kinematic climb simulation — SCALARS ONLY
    %   Physics at each time step dt:
    %     T(h)   = P_A(h) / V_climb       thrust from propulsive power [N]
    %     q(h)   = 0.5 * rho(h) * V^2     dynamic pressure            [N/m^2]
    %     CL     = W / (q * S)            small-angle approx: L ~ W   [-]
    %     CD     = CD0 + k * CL^2                                     [-]
    %     D      = q * S * CD                                         [N]
    %     ROC    = V * (T - D) / W        specific excess power       [m/s]
    %     gamma  = asin(ROC/V)            climb angle                 [rad]
    %     V_h    = V * cos(gamma)         horizontal speed            [m/s]
    %   Weight W2_N is constant — no fuel depletion per dt (team decision).

    h_c       = 0.0;
    d_climb_m = 0.0;   % cumulative horizontal distance                [m]
    t_climb_s = 0.0;   % cumulative time                              [s]
    ROC_sum   = 0.0;
    n_steps   = 0;
    Vh_min    = inf;

    while h_c < h_cruise
        [~, rho_c, ~, ~] = StdAtmos(h_c);
        sigma_c = rho_c / rho_SL;
        P_A_c   = sigma_c * P_A_SL;           % propulsive power       [W]
        T_c     = P_A_c / V_climb;            % thrust                 [N]
        q_c     = 0.5 * rho_c * V_climb^2;   % dynamic pressure       [N/m^2]
        CL_c    = W2_N / (q_c * S);           % lift coefficient       [-]
        CD_c    = CD0 + k * CL_c^2;           % drag coefficient       [-]
        D_c     = q_c * S * CD_c;             % drag                   [N]

        % Rate of climb from specific excess power
        ROC_c = V_climb * (T_c - D_c) / W2_N;  % [m/s]
        if ROC_c <= 0
            % Aircraft is at or above its service ceiling for this weight.
            warning('PUDO670:Ceiling', 'ROC <= 0 at h=%.1f m. Aircraft at ceiling.', h_c);
            break;
        end

        % Climb angle and horizontal speed
        gamma_c   = asin(min(ROC_c / V_climb, 1.0));  % [rad]
        V_horiz_c = V_climb * cos(gamma_c);             % [m/s]
        Vh_min    = min(Vh_min, V_horiz_c);
        %   FLAG-3: if Vh_min < 67 m/s, the climb speed of 70 m/s is
        %   insufficient to maintain the horizontal speed constraint.
        %   Printed in summary output below.

        % Clip last step to land exactly at h_cruise
        dt_eff    = min(dt_climb, (h_cruise - h_c) / ROC_c);

        d_climb_m = d_climb_m + V_horiz_c * dt_eff;
        t_climb_s = t_climb_s + dt_eff;
        h_c       = h_c + ROC_c * dt_eff;
        ROC_sum   = ROC_sum + ROC_c;
        n_steps   = n_steps + 1;
    end

    ROC_avg_ms  = ROC_sum / max(n_steps, 1);  % average ROC             [m/s]
    ROC_avg_fpm = ROC_avg_ms * 196.85;         % average ROC             [ft/min]
    t_climb_min = t_climb_s / 60;             %                         [min]

    % (d) Cruise range
    d_cruise_m   = R_total_m - d_climb_m - d_desc_m;   % [m]
    d_cruise_km  = d_cruise_m / 1e3;
    d_cruise_nmi = d_cruise_m / 1852;

    if d_cruise_m <= 0
        error('PUDO670:RangeError', ...
            'Cruise range = %.1f m (negative). Climb+descent exceed 450 nmi.', d_cruise_m);
    end

    % (e) Cruise fuel fraction — Breguet range equation, propeller aircraft:
    %   R = (eta_p / (g*SFCp_SI)) * L/D * ln(W_start/W_end)
    %   -> ff = exp(-R * g * SFCp_SI / (eta_p * L/D))
    ff_cruise = exp(-d_cruise_m * g * SFCp_SI / (eta_p * LD_cruise));

    % (f) Weight chain — each segment starts at the end weight of the previous
    W3_kg = W2_kg * ff_climb;    % after climb   (Raymer)
    W4_kg = W3_kg * ff_cruise;   % after cruise  (Breguet)
    W5_kg = W4_kg * ff_descent;  % after descent (Raymer)
    W6_kg = W5_kg * ff_landing;  % after landing rollout

    % (g) Residual
    residual_kg = W6_kg - W6_target_kg;

    fprintf('║  %4d | %9.3f | %9.3f | %9.3f | %9.3f | %9.4f  ║\n', ...
        iter, W_fuel_kg, W0_kg, W6_kg, W6_target_kg, residual_kg);

    if abs(residual_kg) < tol_kg
        converged = true;
        break;
    end

    % (h) Newton step
    %   W6 ~ (W_dry + W_fuel) * ff_product
    %   dW6/dW_fuel ~ ff_product  (ff_cruise treated as locally constant)
    %   residual > 0: overweight at landing -> reduce W_fuel
    %   residual < 0: underweight at landing -> increase W_fuel
    ff_product = ff_warmup * ff_takeoff * ff_climb * ff_cruise * ff_descent * ff_landing;
    W_fuel_kg  = W_fuel_kg - residual_kg / ff_product;

end  % end fuel iteration

if ~converged
    warning('PUDO670:NoConverge', ...
        'Fuel iteration did not converge in %d iterations. Final residual: %.4f kg', ...
        max_iter, residual_kg);
end

% Store final converged timing
W0_ref_kg    = W0_kg;
t_cruise_s   = d_cruise_m / V_cruise;
t_cruise_min = t_cruise_s / 60;
t_total_min  = t_climb_min + t_cruise_min + t_desc_min;


%% =========================================================================
%  SECTION 6 — COMMAND WINDOW WEIGHT AND MISSION SUMMARY
% =========================================================================

fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  CONVERGED MISSION SUMMARY                                           ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  WEIGHT BREAKDOWN AT TAKEOFF                                         ║\n');
fprintf('║  %-40s : %8.2f kg             ║\n', 'OEW (structural + crew)', W_OEW_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', '  of which: crew (2 pilots)', W_crew_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'Config 1 full payload', W_pld_C1_full);
fprintf('║  %-40s : %8.2f kg  (90%%)      ║\n', 'Reference mission payload', W_payload_ref);
fprintf('║  %-40s : %8.2f kg             ║\n', 'Mission fuel (incl. loiter reserve)', W_fuel_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', '  of which: loiter reserve', W_loiter_fuel_kg);
fprintf('║  %-40s : %8.2f kg  (%6.0f N)  ║\n', 'REFERENCE MISSION MTOW', W0_ref_kg, W0_ref_kg*g);
fprintf('║  %-40s : %8.2f kg  (%6.0f N)  ║\n', 'Config 2 sizing MTOW (reference)', 4077.0, 4077.0*g);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  FUEL FRACTIONS                                                      ║\n');
fprintf('║  %-40s : %9.6f  (Raymer)    ║\n', 'ff_warmup',  ff_warmup);
fprintf('║  %-40s : %9.6f  (Raymer)    ║\n', 'ff_takeoff', ff_takeoff);
fprintf('║  %-40s : %9.6f  (Raymer)    ║\n', 'ff_climb',   ff_climb);
fprintf('║  %-40s : %9.6f  (Breguet)   ║\n', 'ff_cruise',  ff_cruise);
fprintf('║  %-40s : %9.6f  (Raymer)    ║\n', 'ff_descent', ff_descent);
fprintf('║  %-40s : %9.6f  (Raymer)    ║\n', 'ff_landing', ff_landing);
fprintf('║  %-40s : %9.6f  (Breguet E) ║\n', 'ff_loiter',  ff_loiter);
fprintf('║  %-40s : %9.6f               ║\n', 'zeta = W_fuel / W0', W_fuel_kg/W0_ref_kg);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  SEGMENT WEIGHTS                                                     ║\n');
fprintf('║  %-40s : %8.2f kg             ║\n', 'W0  Takeoff (MTOW)', W0_ref_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'W1  After warm-up/taxi', W1_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'W2  After takeoff (base of climb)', W2_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'W3  After climb', W3_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'W4  After cruise', W4_kg);
fprintf('║  %-40s : %8.2f kg             ║\n', 'W5  After descent', W5_kg);
fprintf('║  %-40s : %8.2f kg  (= W6_target) ║\n', 'W6  After landing rollout', W6_kg);
fprintf('║  %-40s : %8.2f kg  (zero fuel) ║\n', 'W7  After 45-min loiter', W_dry_kg);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  HORIZONTAL DISTANCES                                                ║\n');
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)  ║\n', 'Climb distance', d_climb_m/1e3, d_climb_m/1852);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)  ║\n', 'Cruise distance', d_cruise_km, d_cruise_nmi);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)  ║\n', 'Descent distance', d_desc_m/1e3, d_desc_m/1852);
fprintf('║  %-40s : %8.2f km  (%6.2f nmi)  ║\n', 'Total ground track', R_total_m/1e3, R_total_nmi);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  MISSION TIMING (excl. ground ops)                                   ║\n');
fprintf('║  %-40s : %8.2f min             ║\n', 'Climb time', t_climb_min);
fprintf('║  %-40s : %8.2f min             ║\n', 'Cruise time', t_cruise_min);
fprintf('║  %-40s : %8.2f min             ║\n', 'Descent time', t_desc_min);
fprintf('║  %-40s : %8.2f min             ║\n', 'Total air time', t_total_min);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  CLIMB PERFORMANCE                                                   ║\n');
fprintf('║  %-40s : %8.2f m/s  (%.0f fpm) ║\n', 'Average ROC (full throttle)', ROC_avg_ms, ROC_avg_fpm);
fprintf('║  %-40s : %8.2f m/s              ║\n', 'Climb TAS (fixed)', V_climb);
fprintf('║  %-40s : %8.2f m/s              ║\n', 'Min V_horiz = V*cos(gamma)', Vh_min);
if Vh_min >= V_horiz_req
    fprintf('║  CHECK V_horiz: PASS  (>= 67 m/s throughout climb)               ║\n');
else
    fprintf('║  CHECK V_horiz: FAIL  (< 67 m/s — increase V_climb!)             ║\n');
end
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  CRUISE PARAMETERS                                                   ║\n');
fprintf('║  %-40s : %8.2f m/s (~130 KTAS)  ║\n', 'Cruise TAS', V_cruise);
fprintf('║  %-40s : %8.0f m  (%5.0f ft)   ║\n', 'Cruise altitude', h_cruise, h_cruise/0.3048);
fprintf('║  %-40s : %8.2f                  ║\n', 'L/D at cruise', LD_cruise);
fprintf('║  %-40s : %8.2f kg               ║\n', 'W3 start of cruise', W3_kg);
fprintf('║  %-40s : %8.2f kg               ║\n', 'W4 end of cruise', W4_kg);
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n\n');


%% =========================================================================
%  SECTION 7 — FINAL CLIMB SIMULATION (stores full vectors for plotting)
%
%  Identical physics to the scalar sim in Section 5, but every step is
%  saved for Figure 1. Uses the converged W2_kg.
% =========================================================================

W2_N_final = W2_kg * g;   % converged weight at base of climb          [N]

max_pts = 7000;            % safe upper bound (~4572 m / 1 m/s ROC min = 4572 steps)
t_clmb_vec  = zeros(1, max_pts);
h_clmb_vec  = zeros(1, max_pts);
dx_clmb_vec = zeros(1, max_pts);
V_clmb_vec  = V_climb * ones(1, max_pts);  % TAS constant throughout climb

h_c  = 0.0; d_c = 0.0; t_c = 0.0; step = 1;
t_clmb_vec(1) = 0; h_clmb_vec(1) = 0; dx_clmb_vec(1) = 0;

while h_c < h_cruise
    [~, rho_c, ~, ~] = StdAtmos(h_c);
    sigma_c = rho_c / rho_SL;
    P_A_c   = sigma_c * P_A_SL;
    T_c     = P_A_c / V_climb;
    q_c     = 0.5 * rho_c * V_climb^2;
    CL_c    = W2_N_final / (q_c * S);
    CD_c    = CD0 + k * CL_c^2;
    D_c     = q_c * S * CD_c;
    ROC_c   = V_climb * (T_c - D_c) / W2_N_final;

    if ROC_c <= 0, break; end

    gamma_c   = asin(min(ROC_c / V_climb, 1.0));
    V_horiz_c = V_climb * cos(gamma_c);
    dt_eff    = min(dt_climb, (h_cruise - h_c) / ROC_c);

    d_c = d_c + V_horiz_c * dt_eff;
    t_c = t_c + dt_eff;
    h_c = h_c + ROC_c * dt_eff;

    step = step + 1;
    if step > max_pts
        warning('PUDO670:Preallocate', 'max_pts exceeded in Section 7. Increase max_pts.');
        break;
    end
    t_clmb_vec(step)  = t_c;
    h_clmb_vec(step)  = h_c;
    dx_clmb_vec(step) = d_c;
end

n_clmb     = step;
t_clmb_min = t_clmb_vec(1:n_clmb) / 60;   % [min]
h_clmb     = h_clmb_vec(1:n_clmb);         % [m]
dx_clmb    = dx_clmb_vec(1:n_clmb);        % [m] cumulative horizontal distance
V_clmb     = V_clmb_vec(1:n_clmb);         % [m/s]


%% =========================================================================
%  SECTION 8 — THROTTLE PROFILES
%
%  8a CLIMB:   100% throughout (full power, fixed speed — no adjustment)
%  8b CRUISE:  T = D (level flight); decreases as weight drops during cruise
%  8c DESCENT: along-path balance  T = D - W*sin(gamma_desc)
%              T > 0: apply throttle = T*V / P_A(h)
%              T <= 0: throttle = 0 (gravity alone sustains the descent)
%
%  Throttle is expressed as % of maximum available power at that altitude.
%  These values are for charts only — fuel uses Raymer/Breguet fractions.
% =========================================================================

% 8a: Climb — constant 100%
thr_clmb_pct = 100 * ones(1, n_clmb);   % [%]

% 8b: Cruise — T = D, weight drops linearly from W3 to W4
n_cr     = 300;
t_cr_rel = linspace(0, t_cruise_min, n_cr);   % time relative to cruise start [min]
W_cr_N   = linspace(W3_kg, W4_kg, n_cr) * g;  % [N] weight during cruise

[~, rho_cr, ~, ~] = StdAtmos(h_cruise);
sigma_cr    = rho_cr / rho_SL;
P_A_cr      = sigma_cr * P_A_SL;                        % propulsive power available [W]
q_cr        = 0.5 * rho_cr * V_cruise^2;               % [N/m^2]
CL_cr_vec   = W_cr_N / (q_cr * S);                    % [-]
CD_cr_vec   = CD0 + k * CL_cr_vec.^2;                 % [-]
T_cr_vec    = q_cr * S * CD_cr_vec;                    % T = D [N]
thr_cr_pct  = 100 * (T_cr_vec * V_cruise) / P_A_cr;   % [%]

% 8c: Descent — along-path force balance at each altitude step
n_dc       = 300;
h_dc_vec   = linspace(h_cruise, 0, n_dc);              % altitude [m]
t_dc_rel   = linspace(0, t_desc_min, n_dc);             % time relative to descent start [min]
W_dc_N     = linspace(W5_kg, W6_kg, n_dc) * g;         % weight W5->W6 [N]
thr_dc_pct = zeros(1, n_dc);

for i = 1:n_dc
    [~, rho_d, ~, ~] = StdAtmos(h_dc_vec(i));
    sigma_d = rho_d / rho_SL;
    P_A_d   = sigma_d * P_A_SL;               % [W]
    q_d     = 0.5 * rho_d * V_descent^2;      % [N/m^2]
    CL_d    = W_dc_N(i) / (q_d * S);         % [-]
    CD_d    = CD0 + k * CL_d^2;               % [-]
    D_d     = q_d * S * CD_d;                 % [N]
    %   Along-path steady descent: T + W*sin(gamma) = D
    %   T = D - W*sin(gamma)
    %   gamma > 0, so gravity component aids forward motion.
    T_d = D_d - W_dc_N(i) * sin(gamma_desc);
    if T_d > 0
        thr_dc_pct(i) = 100 * (T_d * V_descent) / P_A_d;  % [%]
    else
        thr_dc_pct(i) = 0;   % pure glide; gravity > drag component
    end
end


%% =========================================================================
%  SECTION 9 — PLOTTING
%
%  Figure 1: Altitude (left y) and TAS (right y) vs mission time (x)
%            Line segments colored by cumulative horizontal distance:
%            Purple(0-100km) Blue(100-200) Cyan(200-300) Green(300-400)
%            Yellow(400-500) Orange(500-600) Red(600+ km)
%
%  Figure 2: Throttle vs time, 3 stacked subplots (climb / cruise / descent)
%            All subplots use global mission time on the x-axis.
% =========================================================================

% Colour table: dynamically sized to cover full mission distance
band_km           = 100;
total_dist_km_pre = (d_climb_m + d_cruise_m + d_desc_m) / 1e3;
n_bands_needed    = ceil(total_dist_km_pre / band_km);
colour_base = [
    0.50, 0.00, 0.75;   % purple
    0.00, 0.20, 0.90;   % blue
    0.00, 0.80, 0.90;   % cyan
    0.00, 0.65, 0.10;   % green
    0.95, 0.85, 0.00;   % yellow
    1.00, 0.50, 0.00;   % orange
    0.90, 0.00, 0.00;   % red
];
if n_bands_needed <= size(colour_base,1)
    colour_rgb = colour_base(1:n_bands_needed, :);
else
    colour_rgb = colour_base(mod((0:n_bands_needed-1), size(colour_base,1))+1, :);
end
n_bands = n_bands_needed;

% Build global mission vectors (3 phases concatenated)
t_plot_clmb = t_clmb_min;                                            % [min]
h_plot_clmb = h_clmb;                                                % [m]
V_plot_clmb = V_clmb;                                                % [m/s]
d_plot_clmb = dx_clmb / 1e3;                                         % [km] cumul.

t_plot_cr   = t_climb_min + t_cr_rel;                                % [min]
h_plot_cr   = h_cruise * ones(1, n_cr);                               % [m]
V_plot_cr   = V_cruise  * ones(1, n_cr);                              % [m/s]
d_plot_cr   = (d_climb_m + linspace(0, d_cruise_m, n_cr)) / 1e3;    % [km] cumul.

t_plot_dc   = t_climb_min + t_cruise_min + t_dc_rel;                 % [min]
h_plot_dc   = h_dc_vec;                                               % [m]
V_plot_dc   = V_descent * ones(1, n_dc);                              % [m/s]
d_plot_dc   = (d_climb_m + d_cruise_m + linspace(0, d_desc_m, n_dc)) / 1e3;  % [km]

% Global concatenation
t_all = [t_plot_clmb, t_plot_cr, t_plot_dc];   % [min]
h_all = [h_plot_clmb, h_plot_cr, h_plot_dc];   % [m]
V_all = [V_plot_clmb, V_plot_cr, V_plot_dc];   % [m/s]
d_all = [d_plot_clmb, d_plot_cr, d_plot_dc];   % [km] cumulative

% Global throttle time vectors
t_cr_global = t_climb_min + t_cr_rel;
t_dc_global = t_climb_min + t_cruise_min + t_dc_rel;

% ── FIGURE 1: Altitude & TAS vs Time ──────────────────────────────────────
figure('Name','PUDO 670 Reference Mission: Airspeed & Altitude', ...
    'Units','normalized','Position',[0.02 0.10 0.92 0.68]);

% Left y-axis: Altitude
yyaxis left
hold on; box on; grid on;
for b = 1:n_bands
    d_lo = (b-1) * band_km;
    d_hi = b * band_km;
    in_b = d_all >= d_lo & d_all <= d_hi;
    idx  = find(in_b);
    if isempty(idx), continue; end
    % Extend one index beyond each edge so adjacent bands share endpoints
    idx_ext = unique([max(1,idx(1)-1), idx(:)', min(numel(d_all),idx(end)+1)]);
    plot(t_all(idx_ext), h_all(idx_ext), '-', ...
        'Color', colour_rgb(b,:), 'LineWidth', 2.8);
end
ylabel('Altitude  [m]', 'FontSize', 12, 'Color', 'k');
ylim([0, h_cruise * 1.22]);
set(gca, 'YColor', 'k');

% Right y-axis: TAS
yyaxis right
hold on;
for b = 1:n_bands
    d_lo = (b-1) * band_km;
    d_hi = b * band_km;
    in_b = d_all >= d_lo & d_all <= d_hi;
    idx  = find(in_b);
    if isempty(idx), continue; end
    idx_ext = unique([max(1,idx(1)-1), idx(:)', min(numel(d_all),idx(end)+1)]);
    plot(t_all(idx_ext), V_all(idx_ext), '--', ...
        'Color', colour_rgb(b,:), 'LineWidth', 1.8);
end
ylabel('True Airspeed  [m/s]', 'FontSize', 12, 'Color', 'k');
ylim([0, V_climb * 1.9]);
set(gca, 'YColor', 'k');

% Phase boundary lines and labels
yyaxis left
xline(t_climb_min,               'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xline(t_climb_min+t_cruise_min,  'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(t_climb_min * 0.45,                          h_cruise * 0.40, 'CLIMB', ...
    'FontSize',10,'FontWeight','bold','Color',[0.3 0.3 0.3],'HorizontalAlignment','center');
text(t_climb_min + t_cruise_min * 0.50,           h_cruise * 1.11, 'CRUISE', ...
    'FontSize',10,'FontWeight','bold','Color',[0.3 0.3 0.3],'HorizontalAlignment','center');
text(t_climb_min + t_cruise_min + t_desc_min*0.5, h_cruise * 0.40, 'DESCENT', ...
    'FontSize',10,'FontWeight','bold','Color',[0.3 0.3 0.3],'HorizontalAlignment','center');

xlabel('Mission Time  [min]', 'FontSize', 12);
xlim([0, t_all(end) * 1.02]);   % ensure full descent visible
title({'PUDO 670 — Reference Mission: True Airspeed & Altitude vs Time', ...
    sprintf('Config 1 | 90%% Payload | W_0 = %.0f kg | 450 nmi | Solid = Altitude  Dashed = TAS', ...
    W0_ref_kg)}, 'FontSize', 11, 'FontWeight', 'bold');

% Colour legend (one entry per populated band)
total_dist_km = d_all(end);
leg_h = gobjects(n_bands,1);
leg_lbl = cell(n_bands,1);
for b = 1:n_bands
    d_lo = (b-1)*band_km;
    if d_lo >= total_dist_km, break; end
    yyaxis left
    leg_h(b)   = plot(nan, nan, '-', 'Color', colour_rgb(b,:), 'LineWidth', 3.5);
    leg_lbl{b} = sprintf('%d - %d km', d_lo, b*band_km);
end
valid_idx = arrayfun(@(x) isgraphics(x,'line'), leg_h);
lg = legend(leg_h(valid_idx), leg_lbl(valid_idx), ...
    'Location','northeast','FontSize',9);
lg.Title.String = 'Cumulative distance';


% ── FIGURE 2: Throttle vs Time (3 subplots) ───────────────────────────────
figure('Name','PUDO 670 Reference Mission: Throttle Profile', ...
    'Units','normalized','Position',[0.02 0.05 0.92 0.85]);

% Subplot 1: Climb
ax_c = subplot(3,1,1);
plot(t_plot_clmb, thr_clmb_pct, '-', 'Color',[0.10 0.30 0.85], 'LineWidth',2.5);
hold on;
yline(100, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
ylabel('Throttle  [%]', 'FontSize', 11);
title(sprintf('Climb  |  V_{TAS} = %.0f m/s  |  Full throttle throughout  |  Avg ROC = %.1f m/s  (%.0f fpm)', ...
    V_climb, ROC_avg_ms, ROC_avg_fpm), 'FontSize', 10);
ylim([0, 125]);
xlim([t_plot_clmb(1), t_plot_clmb(end)]);
grid on; box on;
text(mean(t_plot_clmb), 52, ...
    sprintf('Climb time: %.1f min   |   d_{climb} = %.1f km  (%.1f nmi)', ...
    t_climb_min, d_climb_m/1e3, d_climb_m/1852), ...
    'FontSize',9,'HorizontalAlignment','center', ...
    'BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);

% Subplot 2: Cruise
ax_r = subplot(3,1,2);
plot(t_cr_global, thr_cr_pct, '-', 'Color',[0.82 0.15 0.08], 'LineWidth',2.5);
ylabel('Throttle  [%]', 'FontSize', 11);
title(sprintf('Cruise  |  V_{TAS} = %.0f m/s  |  h = %.0f m (15,000 ft)  |  L/D = %.2f', ...
    V_cruise, h_cruise, LD_cruise), 'FontSize', 10);
y_max_cr = max(thr_cr_pct) * 1.35;
ylim([0, y_max_cr]);
xlim([t_cr_global(1), t_cr_global(end)]);
grid on; box on;
text(mean(t_cr_global), max(thr_cr_pct)*0.40, ...
    sprintf('W_{start} = %.0f kg  ->  W_{end} = %.0f kg   |   d_{cruise} = %.0f km  (%.0f nmi)', ...
    W3_kg, W4_kg, d_cruise_km, d_cruise_nmi), ...
    'FontSize',9,'HorizontalAlignment','center', ...
    'BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);

% Subplot 3: Descent
ax_d = subplot(3,1,3);
plot(t_dc_global, thr_dc_pct, '-', 'Color',[0.05 0.60 0.10], 'LineWidth',2.5);
xlabel('Mission Time  [min]', 'FontSize', 11);
ylabel('Throttle  [%]', 'FontSize', 11);
title(sprintf('Descent  |  V_{TAS} = %.4f m/s  |  V_{horiz} = %.0f m/s  |  ROD = %.0f fpm', ...
    V_descent, V_horiz_req, ROD_fpm), 'FontSize', 10);
y_max_dc = max(max(thr_dc_pct) * 1.35, 5.0);
ylim([0, y_max_dc]);
xlim([t_dc_global(1), t_dc_global(end)]);
grid on; box on;
if max(thr_dc_pct) < 0.10
    text(mean(t_dc_global), y_max_dc*0.5, ...
        'Power-off glide  —  gravity alone sustains the required descent profile', ...
        'FontSize',9,'HorizontalAlignment','center', ...
        'BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);
else
    text(mean(t_dc_global), max(thr_dc_pct)*0.35, ...
        sprintf('Throttle required: natural glide is steeper than %.0f fpm / %.0f m/s horiz', ...
        ROD_fpm, V_horiz_req), ...
        'FontSize',9,'HorizontalAlignment','center', ...
        'BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.7 0.7 0.7]);
end

sgtitle('PUDO 670 — Reference Mission Throttle Profile', ...
    'FontSize', 13, 'FontWeight', 'bold');