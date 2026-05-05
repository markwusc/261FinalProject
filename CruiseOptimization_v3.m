% CruiseOptimization_v3.m
% =========================================================================
% Arctic STOL Aircraft — Cruise Altitude Optimization & AR/S Trade Study
% Version 3
% =========================================================================
% CHANGES FROM v2:
%   - Ceiling search now sweeps independently to 15,000 m (uncapped),
%     fixing the bug where absolute ceiling == 130 KTAS ceiling.
%   - Crossover point (V_LDmax = 130 KTAS) found within altitude sweep
%     and marked on all 4 Figure 1 subplots.
%   - Section 4 console output includes crossover point breakdown.
%   - Figure 2 now has two side-by-side subplots:
%       Left : V_LDmax at 15,000 ft with colour bands (from v2)
%       Right: Percent of drag above D_opt at 130 KTAS (new)
%
% CEILING METHOD:
%   Absolute ceiling = first altitude where sigma*P_A_sl < P_R(V_Pmin,W_To).
%   P_R_min is evaluated at V_Pmin = sqrt(2*W_To/(rho*S*CL_Pmin))
%   where CL_Pmin = sqrt(3*CD0/k) (speed for minimum power required).
%   This is independent of the 130 KTAS cruise speed constraint.
%
%   The 130 KTAS power ceiling is found separately as the highest altitude
%   where sigma*P_A_sl >= P_R(130 KTAS, W_mid). These two altitudes will
%   in general differ; the 130 KTAS ceiling is always <= absolute ceiling.
%
% CROSSOVER POINT:
%   The altitude at which V_LDmax(W_mid) = 130 KTAS.
%   Above this altitude, the aircraft can fly at both 130 KTAS AND L/D_max
%   simultaneously (speed constraint no longer active at minimum drag).
%   Below it, 130 KTAS > V_LDmax (speed-constrained, drag > D_min).
%
% CALLS: StdAtmos(h [m])
% =========================================================================

clear; clc; close all;

%% =========================================================================
%  SECTION 1 — DESIGN PARAMETERS
% =========================================================================

W_To   = 42600;       % Max takeoff weight                     [N]
S      = 45.4;        % Wing reference area                    [m^2]
AR     = 6.81;        % Aspect ratio                           [-]
b      = 17.58;       % Wing span                              [m]
CD0    = 0.03;        % Parasite drag, clean cruise            [-]
e      = 0.9;         % Oswald efficiency factor               [-]
zeta   = 0.139;       % Mission fuel fraction Wf/W_To          [-]
W_mid  = W_To * (1 - zeta / 2);   % Mid-cruise weight         [N]

P_shp  = 1050;        % Rated shaft horsepower, PT6A-60A      [shp]
eta_p  = 0.80;        % Propeller efficiency                   [-]
% Useful propulsive power at sea level. eta_p applied ONCE here.
P_A_sl = eta_p * P_shp * 745.7;   % [W]  = 626,388 W

V_min_ms  = 130 * 0.5144;         % 130 KTAS in m/s           [m/s]
h_O2_all  = 15000 * 0.3048;       % 15,000 ft in metres       [m]
g         = 9.81;
rho_sl    = 1.225;

fprintf('=== SECTION 1: DESIGN PARAMETERS ===\n')
fprintf('  W_To    : %.1f N  (%.1f kg)\n', W_To, W_To/g)
fprintf('  W_mid   : %.1f N  (%.1f kg)  [W_To*(1-zeta/2)]\n', W_mid, W_mid/g)
fprintf('  S       : %.2f m^2  |  AR : %.2f  |  b : %.2f m\n', S, AR, b)
fprintf('  CD0     : %.4f  |  e : %.2f  |  zeta : %.3f\n', CD0, e, zeta)
fprintf('  P_shp   : %.0f shp  |  eta_p : %.2f  |  P_A_sl : %.2f kW\n\n', ...
    P_shp, eta_p, P_A_sl/1e3)

%% =========================================================================
%  SECTION 2 — AERODYNAMIC QUANTITIES (altitude-independent)
% =========================================================================

k       = 1 / (pi * AR * e);

% L/D_max operating point (minimum drag, best range for propeller aircraft)
CL_opt  = sqrt(CD0 / k);       % CL at L/D_max              [-]
CD_opt  = 2 * CD0;             % CD at L/D_max (= 2*CD0)    [-]
LD_max  = CL_opt / CD_opt;     % Maximum L/D                [-]

% V_Pmin operating point (minimum power required — used for ceiling)
% d(P_R)/dV = 0  →  CL_Pmin = sqrt(3*CD0/k),  CD_Pmin = 4*CD0
CL_Pmin = sqrt(3 * CD0 / k);   % CL at minimum P_R         [-]
CD_Pmin = 4 * CD0;             % CD at minimum P_R         [-]
% NOTE: V_Pmin = 3^(-1/4) * V_LDmax ≈ 0.760 * V_LDmax at any altitude.
% They do NOT converge — the user note about "same speed at ceiling" refers
% to the practical operating sense, not an aerodynamic identity.

fprintf('=== SECTION 2: AERODYNAMIC QUANTITIES ===\n')
fprintf('  k        : %.5f\n', k)
fprintf('  CL_opt   : %.4f  |  CD_opt  : %.4f  |  LD_max : %.3f\n', ...
    CL_opt, CD_opt, LD_max)
fprintf('  CL_Pmin  : %.4f  |  CD_Pmin : %.4f\n', CL_Pmin, CD_Pmin)
fprintf('  V_Pmin/V_LDmax ratio : %.4f  (always = 3^(-1/4) = 0.7598)\n\n', 3^(-0.25))

%% =========================================================================
%  SECTION 3 — CEILING CALCULATIONS
%  Two separate sweeps to 15,000 m to avoid the cap-induced equality bug:
%    (A) Absolute ceiling : P_A = P_R(V_Pmin, W_To)  — V_Pmin minimizes P_R
%    (B) 130 KTAS ceiling : P_A = P_R(130 KTAS, W_mid) — fixed cruise speed
%  These use W_To and W_mid respectively for consistency with their roles.
%  The absolute ceiling uses W_To (heaviest aircraft — most conservative).
% =========================================================================

h_search = linspace(0, 15000, 10000);   % Dense search to 15,000 m [m]
h_abs_ceil  = NaN;
h_130_ceil  = NaN;

for i = 1:length(h_search)
    [~, rho_i, ~, ~] = StdAtmos(h_search(i));
    sigma_i = rho_i / rho_sl;
    PA_i    = sigma_i * P_A_sl;

    % ── (A) Absolute ceiling ──────────────────────────────────────────────
    % P_R_min evaluated at V_Pmin and W_To
    if isnan(h_abs_ceil)
        V_Pm_i   = sqrt(2 * W_To / (rho_i * S * CL_Pmin));
        q_Pm_i   = 0.5 * rho_i * V_Pm_i^2;
        PR_min_i = CD_Pmin * q_Pm_i * S * V_Pm_i;   % = D_Pmin * V_Pmin
        if PA_i < PR_min_i
            h_abs_ceil = h_search(i);
        end
    end

    % ── (B) 130 KTAS power ceiling ────────────────────────────────────────
    % P_R evaluated at fixed V = 130 KTAS and W_To
    if isnan(h_130_ceil)
        q_130_i   = 0.5 * rho_i * V_min_ms^2;
        CL_130_i  = W_To / (q_130_i * S);
        CD_130_i  = CD0 + k * CL_130_i^2;
        D_130_i   = CD_130_i * q_130_i * S;
        PR_130_i  = D_130_i * V_min_ms;
        if PA_i < PR_130_i
            h_130_ceil = h_search(i);
        end
    end

    if ~isnan(h_abs_ceil) && ~isnan(h_130_ceil)
        break   % Both found — stop early
    end
end

if isnan(h_abs_ceil);  h_abs_ceil = h_search(end); warning('Abs ceiling > 15,000 m'); end
if isnan(h_130_ceil);  h_130_ceil = h_search(end); warning('130 KTAS ceiling > 15,000 m'); end

fprintf('=== SECTION 3: CEILING RESULTS ===\n')
fprintf('  Absolute ceiling (P_A = P_R_min at V_Pmin, W_To) : %.0f m  (%.0f ft)\n', ...
    h_abs_ceil, h_abs_ceil/0.3048)
fprintf('  130 KTAS ceiling (P_A = P_R at 130 KTAS, W_To)  : %.0f m  (%.0f ft)\n\n', ...
    h_130_ceil, h_130_ceil/0.3048)

%% =========================================================================
%  SECTION 4 — ALTITUDE SWEEP  (0 to absolute ceiling)
%  At each altitude, ALL quantities evaluated at V = 130 KTAS.
%  V_LDmax is computed at W_mid for the crossover search.
% =========================================================================

n_pts = 800;
h_vec = linspace(0, h_abs_ceil, n_pts);   % [m]

V_LDmax_vec = zeros(1, n_pts);
CL_vec      = zeros(1, n_pts);
LD_vec      = zeros(1, n_pts);
D_p_vec     = zeros(1, n_pts);   % Parasite drag at 130 KTAS
D_i_vec     = zeros(1, n_pts);   % Induced  drag at 130 KTAS
D_vec       = zeros(1, n_pts);   % Total    drag at 130 KTAS
P_R_vec     = zeros(1, n_pts);
P_A_vec     = zeros(1, n_pts);
excess_vec  = zeros(1, n_pts);

for i = 1:n_pts
    [~, rho_i, ~, ~] = StdAtmos(h_vec(i));
    sigma_i = rho_i / rho_sl;

    % V_LDmax at this altitude (uses W_mid to match the crossover definition)
    V_LDmax_vec(i) = sqrt(2 * W_mid / (rho_i * S * CL_opt));

    % Aerodynamics at 130 KTAS, W_mid, level flight
    q_i          = 0.5 * rho_i * V_min_ms^2;
    CL_vec(i)    = W_mid / (q_i * S);
    D_p_vec(i)   = CD0 * q_i * S;
    D_i_vec(i)   = k * W_mid^2 / (q_i * S);
    D_vec(i)     = D_p_vec(i) + D_i_vec(i);
    LD_vec(i)    = W_mid / D_vec(i);
    P_R_vec(i)   = D_vec(i) * V_min_ms;
    P_A_vec(i)   = sigma_i * P_A_sl;
    excess_vec(i) = P_A_vec(i) - P_R_vec(i);
end

% ── Find crossover: V_LDmax = 130 KTAS ────────────────────────────────────
% V_LDmax increases monotonically with altitude. Find the first index where
% V_LDmax >= V_min_ms and interpolate to get the precise crossing altitude.
idx_cross = find(V_LDmax_vec >= V_min_ms, 1, 'first');
if ~isempty(idx_cross) && idx_cross > 1
    % Linear interpolation between idx_cross-1 and idx_cross
    h_lo  = h_vec(idx_cross-1);  V_lo = V_LDmax_vec(idx_cross-1);
    h_hi  = h_vec(idx_cross);    V_hi = V_LDmax_vec(idx_cross);
    h_cross = h_lo + (V_min_ms - V_lo) / (V_hi - V_lo) * (h_hi - h_lo);
else
    h_cross = NaN;
    warning('V_LDmax crossover not found within ceiling sweep.')
end

% Properties at crossover altitude (interpolated)
if ~isnan(h_cross)
    [~, rho_cross, ~, ~] = StdAtmos(h_cross);
    sigma_cross = rho_cross / rho_sl;
    % At crossover: V_LDmax = V_130 = V_min_ms
    % Aircraft is now flying at exactly minimum drag speed
    q_cross    = 0.5 * rho_cross * V_min_ms^2;
    CL_cross   = W_mid / (q_cross * S);          % should equal CL_opt
    Dp_cross   = CD0 * q_cross * S;
    Di_cross   = k * W_mid^2 / (q_cross * S);
    D_cross    = Dp_cross + Di_cross;             % = D_min at this altitude
    LD_cross   = W_mid / D_cross;
    PR_cross   = D_cross * V_min_ms;
    PA_cross   = sigma_cross * P_A_sl;
    exc_cross  = PA_cross - PR_cross;
    thr_cross  = PR_cross / PA_cross;
    D_min_cross = W_mid / LD_max;   % theoretical minimum drag (= D_cross)
end

% Properties at 15,000 ft specifically
[~, rho_15k, ~, ~] = StdAtmos(h_O2_all);
sigma_15k   = rho_15k / rho_sl;
q_15k       = 0.5 * rho_15k * V_min_ms^2;
CL_15k      = W_mid / (q_15k * S);
Dp_15k      = CD0 * q_15k * S;
Di_15k      = k * W_mid^2 / (q_15k * S);
D_15k       = Dp_15k + Di_15k;
LD_15k      = W_mid / D_15k;
PR_15k      = D_15k * V_min_ms;
PA_15k      = sigma_15k * P_A_sl;
exc_15k     = PA_15k - PR_15k;
thr_15k     = PR_15k / PA_15k;
D_opt_15k   = W_mid / LD_max;    % minimum possible drag at W_mid
pct_above   = (D_15k - D_opt_15k) / D_opt_15k * 100;
V_LDmax_15k = sqrt(2 * W_mid / (rho_15k * S * CL_opt));

fprintf('=== SECTION 4: ALTITUDE SWEEP SUMMARY ===\n')
fprintf('  Altitude sweep range : 0 to %.0f m (absolute ceiling)\n\n', h_abs_ceil)

if ~isnan(h_cross)
    fprintf('  --- Crossover Point (V_LDmax = 130 KTAS) ---\n')
    fprintf('  Altitude       : %7.0f m  (%.0f ft)\n',   h_cross, h_cross/0.3048)
    fprintf('  Air density    : %7.4f kg/m^3  (sigma = %.4f)\n', rho_cross, sigma_cross)
    fprintf('  V_cruise       : %7.2f m/s  (130.0 KTAS)\n', V_min_ms)
    fprintf('  CL (= CL_opt)  : %7.4f  (deficit from CL_opt : %.4f)\n', CL_cross, CL_opt-CL_cross)
    fprintf('  L/D (= LD_max) : %7.2f\n', LD_cross)
    fprintf('  D_parasite     : %7.1f N  (%5.1f%% of total)\n', Dp_cross, 100*Dp_cross/D_cross)
    fprintf('  D_induced      : %7.1f N  (%5.1f%% of total)\n', Di_cross, 100*Di_cross/D_cross)
    fprintf('  D_total        : %7.1f N  (100.0%% — at D_min by definition)\n', D_cross)
    fprintf('  P_R            : %7.2f kW\n', PR_cross/1e3)
    fprintf('  P_A            : %7.2f kW  (sigma-scaled)\n', PA_cross/1e3)
    fprintf('  Throttle       : %7.1f%%\n', thr_cross*100)
    fprintf('  Excess power   : %7.2f kW\n\n', exc_cross/1e3)
end

fprintf('  --- 15,000 ft (%.0f m) at 130 KTAS ---\n', h_O2_all)
fprintf('  Air density    : %7.4f kg/m^3  (sigma = %.4f)\n', rho_15k, sigma_15k)
fprintf('  V_cruise       : %7.2f m/s  (130.0 KTAS)\n', V_min_ms)
fprintf('  V_LDmax here   : %7.2f m/s  (%.1f KTAS)\n', V_LDmax_15k, V_LDmax_15k/0.5144)
fprintf('  CL (actual)    : %7.4f  (CL_opt = %.4f, deficit = %.4f)\n', CL_15k, CL_opt, CL_opt-CL_15k)
fprintf('  L/D (actual)   : %7.2f  (LD_max = %.2f, deficit = %.2f)\n', LD_15k, LD_max, LD_max-LD_15k)
fprintf('  D_parasite     : %7.1f N  (%5.1f%% of total drag)\n', Dp_15k, 100*Dp_15k/D_15k)
fprintf('  D_induced      : %7.1f N  (%5.1f%% of total drag)\n', Di_15k, 100*Di_15k/D_15k)
fprintf('  D_total        : %7.1f N  (%.1f%% above D_opt = %.1f N)\n', D_15k, pct_above, D_opt_15k)
fprintf('  P_R            : %7.2f kW\n', PR_15k/1e3)
fprintf('  P_A            : %7.2f kW  (sigma-scaled)\n', PA_15k/1e3)
fprintf('  Throttle       : %7.1f%%\n', thr_15k*100)
fprintf('  Excess power   : %7.2f kW\n', exc_15k/1e3)
if exc_15k >= 0
    fprintf('  Status         : FEASIBLE at 130 KTAS, 15,000 ft\n\n')
else
    fprintf('  Status         : INFEASIBLE (P_A < P_R at 15,000 ft / 130 KTAS)\n\n')
end

%% =========================================================================
%  SECTION 5 — BREGUET RANGE CHECK
% =========================================================================

SFCp_SI = 0.548 * 0.4536 / (745.7 * 3600);
R_m     = (eta_p / (g * SFCp_SI)) * LD_15k * log(W_To / (W_To*(1-zeta)));
R_nmi   = R_m / 1852;

fprintf('=== SECTION 5: BREGUET RANGE CHECK (130 KTAS, 15,000 ft) ===\n')
fprintf('  L/D at cruise : %.2f\n', LD_15k)
fprintf('  Range         : %.0f m  (%.0f nmi)\n', R_m, R_nmi)
fprintf('  Requirement   : >= 450 nmi\n')
if R_nmi >= 450
    fprintf('  Status        : PASS  (+%.0f nmi margin)\n\n', R_nmi-450)
else
    fprintf('  Status        : FAIL  (%.0f nmi short)\n\n', 450-R_nmi)
end

%% =========================================================================
%  FIGURE 1 — ALTITUDE SWEEP (4 subplots, all axes in SI)
%  Crossover point marked on all 4 subplots.
%  Reference lines: 15,000 ft O2 threshold, absolute ceiling,
%                   130 KTAS power ceiling, crossover altitude.
% =========================================================================

col_O2    = [0.75 0.00 0.75];   % magenta  — 15,000 ft O2 line
col_ceil  = [0.40 0.40 0.40];   % grey     — absolute ceiling
col_130c  = [0.00 0.55 0.00];   % green    — 130 KTAS power ceiling
col_cross = [0.85 0.45 0.00];   % orange   — crossover point

% Marker data at crossover (for all subplots)
if ~isnan(h_cross)
    cross_LD   = LD_cross;
    cross_PR   = PR_cross / 1e3;
    cross_PA   = PA_cross / 1e3;
    cross_exc  = exc_cross / 1e3;
    cross_VLD  = V_min_ms;   % by definition at crossover
end

figure('Name', 'Figure 1: Cruise Altitude Optimization', ...
    'Units', 'normalized', 'Position', [0.02 0.05 0.88 0.82]);

% ── Subplot 1: V_LDmax vs altitude ───────────────────────────────────────
subplot(2,2,1); hold on;
plot(h_vec, V_LDmax_vec, 'b-',  'LineWidth', 2.0, 'DisplayName', 'V_{LDmax}  (W_{mid})');
yline(V_min_ms, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('130 KTAS = %.2f m/s', V_min_ms));
xline(h_O2_all,    '--', 'Color', col_O2,   'LineWidth', 1.3, ...
    'DisplayName', sprintf('15,000 ft = %.0f m (O_2)', h_O2_all));
xline(h_abs_ceil,  '-',  'Color', col_ceil,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('Abs. ceiling = %.0f m', h_abs_ceil));
xline(h_130_ceil,  '-',  'Color', col_130c,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('130 KTAS ceiling = %.0f m', h_130_ceil));
if ~isnan(h_cross)
    plot(h_cross, cross_VLD, 'o', 'Color', col_cross, 'MarkerSize', 10, ...
        'MarkerFaceColor', col_cross, ...
        'DisplayName', sprintf('Crossover = %.0f m', h_cross));
    text(h_cross + h_abs_ceil*0.02, cross_VLD - 1.5, ...
        sprintf('%.0f m\n(%.0f ft)', h_cross, h_cross/0.3048), ...
        'Color', col_cross, 'FontSize', 8);
end
xlabel('Altitude  [m]', 'FontSize', 11);
ylabel('Speed  [m/s]',  'FontSize', 11);
title('Min-Drag Speed vs Altitude', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 7.5);
grid on;

% ── Subplot 2: L/D at 130 KTAS vs altitude ───────────────────────────────
subplot(2,2,2); hold on;
plot(h_vec, LD_vec, 'b-', 'LineWidth', 2.0, 'DisplayName', 'L/D at 130 KTAS');
yline(LD_max, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('L/D_{max} = %.2f', LD_max));
xline(h_O2_all,   '--', 'Color', col_O2,   'LineWidth', 1.3, ...
    'DisplayName', sprintf('15,000 ft = %.0f m', h_O2_all));
xline(h_abs_ceil, '-',  'Color', col_ceil,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('Abs. ceiling = %.0f m', h_abs_ceil));
xline(h_130_ceil, '-',  'Color', col_130c,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('130 KTAS ceiling = %.0f m', h_130_ceil));
if ~isnan(h_cross)
    plot(h_cross, cross_LD, 'o', 'Color', col_cross, 'MarkerSize', 10, ...
        'MarkerFaceColor', col_cross, ...
        'DisplayName', sprintf('Crossover = %.0f m', h_cross));
    text(h_cross + h_abs_ceil*0.02, cross_LD - 0.3, ...
        sprintf('L/D = %.2f\n= L/D_{max}', cross_LD), ...
        'Color', col_cross, 'FontSize', 8);
end
xlabel('Altitude  [m]', 'FontSize', 11);
ylabel('L/D  [-]',      'FontSize', 11);
title('L/D at 130 KTAS vs Altitude', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 7.5);
grid on;

% ── Subplot 3: Power vs altitude ─────────────────────────────────────────
subplot(2,2,3); hold on;
plot(h_vec, P_R_vec/1e3, 'r-', 'LineWidth', 2.0, 'DisplayName', 'P_R (130 KTAS)');
plot(h_vec, P_A_vec/1e3, 'b-', 'LineWidth', 2.0, 'DisplayName', 'P_A (\sigma-scaled)');
xline(h_O2_all,   '--', 'Color', col_O2,   'LineWidth', 1.3, ...
    'DisplayName', sprintf('15,000 ft = %.0f m', h_O2_all));
xline(h_abs_ceil, '-',  'Color', col_ceil,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('Abs. ceiling = %.0f m', h_abs_ceil));
xline(h_130_ceil, '-',  'Color', col_130c,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('130 KTAS ceiling = %.0f m', h_130_ceil));
if ~isnan(h_cross)
    plot(h_cross, cross_PR, 'o', 'Color', col_cross, 'MarkerSize', 10, ...
        'MarkerFaceColor', col_cross, 'DisplayName', sprintf('Crossover = %.0f m', h_cross));
    plot(h_cross, cross_PA, 'o', 'Color', col_cross, 'MarkerSize', 10, ...
        'MarkerFaceColor', col_cross, 'HandleVisibility', 'off');
end
xlabel('Altitude  [m]', 'FontSize', 11);
ylabel('Power  [kW]',   'FontSize', 11);
title('Power Required & Available vs Altitude', 'FontSize', 11);
legend('Location', 'northeast', 'FontSize', 7.5);
grid on;

% ── Subplot 4: Excess power vs altitude ──────────────────────────────────
subplot(2,2,4); hold on;
plot(h_vec, excess_vec/1e3, 'k-', 'LineWidth', 2.0, 'DisplayName', 'Excess power');
yline(0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'P_A = P_R');
xline(h_O2_all,   '--', 'Color', col_O2,   'LineWidth', 1.3, ...
    'DisplayName', sprintf('15,000 ft = %.0f m', h_O2_all));
xline(h_abs_ceil, '-',  'Color', col_ceil,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('Abs. ceiling = %.0f m', h_abs_ceil));
xline(h_130_ceil, '-',  'Color', col_130c,  'LineWidth', 1.5, ...
    'DisplayName', sprintf('130 KTAS ceiling = %.0f m', h_130_ceil));
if ~isnan(h_cross)
    plot(h_cross, cross_exc, 'o', 'Color', col_cross, 'MarkerSize', 10, ...
        'MarkerFaceColor', col_cross, ...
        'DisplayName', sprintf('Crossover = %.0f m', h_cross));
end
xlabel('Altitude  [m]',      'FontSize', 11);
ylabel('Excess Power  [kW]', 'FontSize', 11);
title('Excess Power at 130 KTAS vs Altitude', 'FontSize', 11);
legend('Location', 'northeast', 'FontSize', 7.5);
grid on;

sgtitle(sprintf(['Cruise Optimization v3  |  W_{mid} = %.1f kN  (\\zeta = %.3f)  |  ' ...
    'V_{cruise} = %.2f m/s (130 KTAS)\n' ...
    'S = %.1f m^2,  AR = %.2f,  CD_0 = %.3f,  e = %.2f  |  ' ...
    'P_{A,sl} = %.1f kW  (\\eta_p = %.2f,  %.0f shp)'], ...
    W_mid/1e3, zeta, V_min_ms, S, AR, CD0, e, P_A_sl/1e3, eta_p, P_shp), ...
    'FontSize', 10);

%% =========================================================================
%  FIGURE 2 — AR vs S: TWO SUBPLOTS
%   Left  (2,1,1): V_LDmax at 15,000 ft — colour bands by % below 130 KTAS
%   Right (2,1,2): % drag above D_opt at 130 KTAS, 15,000 ft — jet colorbar
% =========================================================================

nAR = 200;  nS = 200;
AR_vec2 = linspace(4,  14, nAR);
S_vec2  = linspace(20, 90, nS);

V_LDmax_grid = zeros(nAR, nS);   % [m/s]
pct_drag_grid = zeros(nAR, nS);  % [%]

for i = 1:nAR
    for j = 1:nS
        AR_ij  = AR_vec2(i);
        S_ij   = S_vec2(j);
        k_ij   = 1 / (pi * AR_ij * e);

        % CL_opt and LD_max for this (AR, S)
        CL_opt_ij  = sqrt(CD0 / k_ij);
        CD_opt_ij  = 2 * CD0;
        LD_max_ij  = CL_opt_ij / CD_opt_ij;

        % V_LDmax at 15,000 ft (W_mid)
        V_LDmax_grid(i,j) = sqrt(2 * W_mid / (rho_15k * S_ij * CL_opt_ij));

        % Drag at 130 KTAS, 15,000 ft for this (AR, S)
        CL_130_ij   = W_mid / (q_15k * S_ij);
        CD_130_ij   = CD0 + k_ij * CL_130_ij^2;
        D_130_ij    = CD_130_ij * q_15k * S_ij;

        % Optimal (minimum) drag at W_mid
        D_opt_ij    = W_mid / LD_max_ij;

        % Percent above optimal
        pct_drag_grid(i,j) = (D_130_ij - D_opt_ij) / D_opt_ij * 100;
    end
end

% ── Colour bands for left subplot ─────────────────────────────────────────
T100 = 1.00 * V_min_ms;  % >= 130 KTAS        → green
T90  = 0.90 * V_min_ms;  % 90-100%            → light green
T80  = 0.80 * V_min_ms;  % 80-90%             → yellow
T70  = 0.70 * V_min_ms;  % 70-80%             → light red
                          % < 70%              → dark red

z_green = V_LDmax_grid >= T100;
z_lgn   = (V_LDmax_grid >= T90)  & ~z_green;
z_yell  = (V_LDmax_grid >= T80)  & ~z_green & ~z_lgn;
z_lred  = (V_LDmax_grid >= T70)  & ~z_green & ~z_lgn & ~z_yell;
z_dred  = V_LDmax_grid  <  T70;

rgb2 = zeros(nAR, nS, 3);
% Green [0.10 0.60 0.10]
rgb2(:,:,1)=rgb2(:,:,1)+0.10*z_green; rgb2(:,:,2)=rgb2(:,:,2)+0.60*z_green; rgb2(:,:,3)=rgb2(:,:,3)+0.10*z_green;
% Light green [0.60 0.92 0.60]
rgb2(:,:,1)=rgb2(:,:,1)+0.60*z_lgn;   rgb2(:,:,2)=rgb2(:,:,2)+0.92*z_lgn;   rgb2(:,:,3)=rgb2(:,:,3)+0.60*z_lgn;
% Yellow [0.95 0.90 0.20]
rgb2(:,:,1)=rgb2(:,:,1)+0.95*z_yell;  rgb2(:,:,2)=rgb2(:,:,2)+0.90*z_yell;  rgb2(:,:,3)=rgb2(:,:,3)+0.20*z_yell;
% Light red [0.95 0.55 0.55]
rgb2(:,:,1)=rgb2(:,:,1)+0.95*z_lred;  rgb2(:,:,2)=rgb2(:,:,2)+0.55*z_lred;  rgb2(:,:,3)=rgb2(:,:,3)+0.55*z_lred;
% Dark red [0.65 0.08 0.08]
rgb2(:,:,1)=rgb2(:,:,1)+0.65*z_dred;  rgb2(:,:,2)=rgb2(:,:,2)+0.08*z_dred;  rgb2(:,:,3)=rgb2(:,:,3)+0.08*z_dred;

figure('Name', 'Figure 2: AR vs S Trade Study', ...
    'Units', 'normalized', 'Position', [0.04 0.05 0.92 0.80]);

% ── Left subplot: V_LDmax colour bands ────────────────────────────────────
subplot(1,2,1);
imagesc(S_vec2, AR_vec2, rgb2);
set(gca, 'YDir', 'normal');
hold on;

% Threshold boundary contours (dashed black)
contour(S_vec2, AR_vec2, V_LDmax_grid, [T100 T100], 'k-',  'LineWidth', 2.2);
contour(S_vec2, AR_vec2, V_LDmax_grid, [T90  T90],  'k--', 'LineWidth', 1.6);
contour(S_vec2, AR_vec2, V_LDmax_grid, [T80  T80],  'k--', 'LineWidth', 1.6);
contour(S_vec2, AR_vec2, V_LDmax_grid, [T70  T70],  'k--', 'LineWidth', 1.6);

% Speed labels (white contours)
[Cs, hs] = contour(S_vec2, AR_vec2, V_LDmax_grid, 8, ...
    'Color', [1 1 1], 'LineWidth', 0.8);
clabel(Cs, hs, 'FontSize', 7.5, 'Color', [1 1 1], 'LabelSpacing', 220);

% Current design point
plot(S, AR, 'w^', 'MarkerSize', 12, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
text(S+0.8, AR+0.12, sprintf('Design\nAR=%.2f\nS=%.1f m^2', AR, S), ...
    'Color', 'w', 'FontSize', 8, 'FontWeight', 'bold');

% Legend proxies
hp(1)=patch(NaN,NaN,[0.10 0.60 0.10],'EdgeColor','none', ...
    'DisplayName',sprintf('\\geq 130 KTAS (\\geq %.2f m/s)', T100));
hp(2)=patch(NaN,NaN,[0.60 0.92 0.60],'EdgeColor','none', ...
    'DisplayName',sprintf('90-100%%  (%.2f–%.2f m/s)', T90, T100));
hp(3)=patch(NaN,NaN,[0.95 0.90 0.20],'EdgeColor','none', ...
    'DisplayName',sprintf('80-90%%   (%.2f–%.2f m/s)', T80, T90));
hp(4)=patch(NaN,NaN,[0.95 0.55 0.55],'EdgeColor','none', ...
    'DisplayName',sprintf('70-80%%   (%.2f–%.2f m/s)', T70, T80));
hp(5)=patch(NaN,NaN,[0.65 0.08 0.08],'EdgeColor','none', ...
    'DisplayName',sprintf('< 70%%    (< %.2f m/s)', T70));
hp(6)=plot(NaN,NaN,'w^','MarkerSize',10,'MarkerFaceColor','w', ...
    'DisplayName','Current design point');
legend(hp, 'Location','northeast','FontSize',7.5, ...
    'Color',[0.15 0.15 0.15],'TextColor','w','EdgeColor',[0.4 0.4 0.4]);

xlabel('Wing Area  S  [m^2]', 'FontSize', 12);
ylabel('Aspect Ratio  AR  [-]', 'FontSize', 12);
title({'V_{LDmax} at 15,000 ft  vs  AR and S', ...
    '(contour values in m/s)'}, 'FontSize', 11);
grid on; ax1=gca; ax1.GridColor=[1 1 1]; ax1.GridAlpha=0.18; ax1.FontSize=10;
xlim([S_vec2(1) S_vec2(end)]); ylim([AR_vec2(1) AR_vec2(end)]);

% ── Right subplot: % drag above D_opt at 130 KTAS ─────────────────────────
subplot(1,2,2);
% Cap display range for colorbar readability; extreme values stay visible
pct_display = min(pct_drag_grid, 200);
contourf(S_vec2, AR_vec2, pct_display, 30, 'LineColor', 'none');
colormap(gca, jet);
cb = colorbar;
cb.Label.String = '% Drag above D_{opt} at 130 KTAS, 15,000 ft';
cb.Label.FontSize = 10;
clim([0 150]);   % 0–150% range; cells above 150% are max-red
hold on;

% Labelled iso-lines for navigation
[Cp, hp2] = contour(S_vec2, AR_vec2, pct_drag_grid, ...
    [10 25 50 75 100 150], ...
    'Color', [0.15 0.15 0.15], 'LineWidth', 0.9);
clabel(Cp, hp2, 'FontSize', 8, 'Color', [0.05 0.05 0.05]);

% Current design point
plot(S, AR, 'w^', 'MarkerSize', 12, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
text(S+0.8, AR+0.12, sprintf('Design\n%.0f%% above\nD_{opt}', ...
    (D_15k-D_opt_15k)/D_opt_15k*100), ...
    'Color','w','FontSize',8,'FontWeight','bold');

xlabel('Wing Area  S  [m^2]', 'FontSize', 12);
ylabel('Aspect Ratio  AR  [-]', 'FontSize', 12);
title({'% Drag above D_{opt}  at 130 KTAS, 15,000 ft', ...
    '(D_{opt} = W_{mid}/LD_{max} for each AR, S)'}, 'FontSize', 11);
grid on; ax2=gca; ax2.FontSize=10;
xlim([S_vec2(1) S_vec2(end)]); ylim([AR_vec2(1) AR_vec2(end)]);

sgtitle(sprintf(['AR vs S Trade Study  |  W_{mid} = %.1f kN  |  ' ...
    'CD_0 = %.3f  |  e = %.2f  |  15,000 ft = %.0f m'], ...
    W_mid/1e3, CD0, e, h_O2_all), 'FontSize', 11);

annotation('textbox', [0.13 0.005 0.75 0.033], ...
    'String', ['NOTE: V_{LDmax} < 130 KTAS everywhere in STOL-relevant design space. ' ...
    'Green region (small S, high AR) trades cruise efficiency for longer field length.'], ...
    'FontSize', 8, 'EdgeColor','none','HorizontalAlignment','center', ...
    'Color',[0.30 0.30 0.30]);
