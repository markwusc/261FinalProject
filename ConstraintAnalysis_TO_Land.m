% ConstraintAnalysis_TO_Land.m
% =========================================================================
% CONSTRAINT ANALYSIS: Takeoff AND Landing Distance vs. Wing Area (S) and
% Aspect Ratio (AR)
% =========================================================================
% PURPOSE:
%   Sweep wing area S and aspect ratio AR and evaluate BOTH the takeoff
%   field length (over 50 ft obstacle) AND the landing field length (over
%   50 ft obstacle) at each combination.  Each cell in the design space is
%   classified as:
%       GREEN      — Both TO and landing distance <= 500 ft (152.4 m) [PASS]
%       LIGHT RED  — One constraint violated (TO or landing, not both)
%       DARK RED   — Both constraints violated
%
%   The 500 ft iso-contour for each constraint is overlaid so the exact
%   boundary between feasible and infeasible regions is readable.
%
% DESIGN PHILOSOPHY / ITERATION NOTE:
%   This is a CONSTRAINT diagram, not an optimisation.  The goal is to map
%   the feasible design space so that a single (S, AR) operating point can
%   be selected inside the green region with sufficient margin on both sides.
%
%   W_To is held fixed (no weight loop yet).  As weight converges the
%   feasible region will shift — re-run this script every iteration.
%   The conservative choice is to use W_To for landing weight as well
%   (zero fuel burn assumed), which slightly pessimises the landing contour
%   but ensures any selected design point survives worst-case heavy landing.
%
% KEY PHYSICS DECISIONS:
%   Takeoff:
%       - Ground roll uses energy method (Anderson §6.7): 1.44*W^2 / (...)
%       - mu_r = 0.4  (gravel rolling friction, RFP constant)
%       - Ground-effect correction on induced drag (Raymer eq. 12.6)
%       - V_Lo = 1.2 * V_stall,  V_avg = 0.7 * V_Lo
%       - Climb segment linearised at constant V_Lo (conservative)
%
%   Landing:
%       - Air phase: d_air = (CL_max/CD) * (h_obs + 0.133*V_stall^2/(2g))
%         High L/D gives a FLATTER glide slope → LONGER d_air.  This is
%         the fundamental STOL aerodynamic penalty: efficiency fights you
%         during landing.
%       - Ground roll from V_TD = 1.3*V_stall to rest at V_avg = 0.7*V_TD
%       - Braking: mu_brake = 0.5 (harder braking surface than takeoff)
%       - Reverse thrust T_rev = eta_rev * T_A assists deceleration
%       - Landing weight conservatively set equal to W_To (worst-case)
%
% REFERENCES:
%   Anderson, "Introduction to Flight", Ch. 6 (takeoff & landing)
%   Raymer, "Aircraft Design: A Conceptual Approach", Ch. 12, 17
% =========================================================================

clear; clc; close all;

% =========================================================================
%  SECTION 1 — FIXED DESIGN PARAMETERS  (update every design iteration)
% =========================================================================

% ── Weight ───────────────────────────────────────────────────────────────
W_To    = 32000;    % Max takeoff weight                          [N]
                    % ESTIMATE ~4077 kg — update when weight loop converges
zeta = 0.125; % fuel fraction, changes with mission
fuel_pct_remaining = 0.05; % reserve fuel percentage

W_fuel_total  = zeta      * W_To;         % Total usable fuel weight [N]
W_fuel_remain = fuel_pct_remaining * W_fuel_total;  % Fuel aboard at landing [N]
W_fuel_burned = W_fuel_total - W_fuel_remain;       % Fuel consumed on mission [N]
W_land = W_To - W_fuel_burned;               % Aircraft landing weight [N]

% ── Propulsion ───────────────────────────────────────────────────────────

shp2W = 745.7; % conversion factor from hp to W
P = 1050; % [hp] Engine rated shp, input one engine.
eta = 0.8;
P_A = eta*P*shp2W;

V_thrustcalc = 30; % [m/s] velocity which calculates thrust
T_A     = P_A/V_thrustcalc;   % Maximum available thrust at S/L ISA          [N]
                    % (~11240 lbf total) — update after engine selection

% Reverse thrust via propeller beta-pitch (PT6A turboprop class).
% Published effectiveness: 40–55% of max forward thrust.
% Using 0.45 as a conservative mid-range value.
eta_rev = 0.40;    % Thrust reversal effectiveness factor         [-]
T_rev   = eta_rev * T_A;   % Effective reverse thrust             [N]
                            % = 22500 N  (~5058 lbf)

% ── Aerodynamics ─────────────────────────────────────────────────────────
CL_max  = 2.6;     % Max lift coefficient with full-flap config   [-]
                    % Per RFP §4.3: aircraft CLmax = 0.8 * airfoil CLmax
                    % Assumes high-lift STOL airfoil (e.g. NACA 4415 + flaps)
                    % NOTE: same CL_max used for both TO and landing; in
                    % reality the landing flap setting may differ slightly.
                    % Revisit when airfoil is finalised.

CD_0_clean  = 0.030;   % Parasite drag — clean (takeoff) config   [-]
CD_0_land   = 0.060;   % Parasite drag — landing config (full flap)[-]
                        % ~0.02 increment above clean; refine with drag buildup

e       = 0.9;     % Oswald efficiency factor (RFP §4.3, clean)   [-]

% ── Geometry ─────────────────────────────────────────────────────────────
h_wing  = 4.0;     % Wing height above ground (ground-effect)     [m]
                    % Typical high-wing STOL strut-braced config.
                    % Adjust when CAD geometry is finalised.

% ── Friction / braking ───────────────────────────────────────────────────
mu_r     = 0.4;    % Rolling friction — gravel runway (RFP const) [-]
mu_brake = 0.5;    % Braking friction — landing ground roll        [-]
                    % Slightly higher than mu_r: brakes locked vs rolling.
                    % Use 0.3 for wet/icy gravel as a sensitivity check.

% ── Physical constants ────────────────────────────────────────────────────
g   = 9.81;        % Gravitational acceleration                    [m/s^2]
rho = 1.225;       % Sea-level ISA air density                     [kg/m^3]
h_obs = 15.24;     % 50 ft obstacle in metres                      [m]
d_req = 152.4;     % 500 ft field-length requirement               [m]

% ── Sweep ranges ─────────────────────────────────────────────────────────
% Sized around expected operating point for this aircraft class.
% DHC-6 Twin Otter: S ~ 39 m², AR ~ 10.  Caravan: S ~ 26 m², AR ~ 9.
nS  = 120;
nAR = 120;
S_vec  = linspace(35, 80, nS);    % Wing reference area sweep     [m^2]
AR_vec = linspace(5,  13, nAR);   % Aspect ratio sweep            [-]

% =========================================================================
%  SECTION 2 — PRE-ALLOCATE RESULT MATRICES
% =========================================================================

d_To_grid   = zeros(nAR, nS);   % Takeoff distance grid  (rows=AR, cols=S)
d_land_grid = zeros(nAR, nS);   % Landing distance grid

% =========================================================================
%  SECTION 3 — DOUBLE SWEEP OVER (S, AR)
% =========================================================================

for i = 1:nAR
    for j = 1:nS

        S_ij  = S_vec(j);
        AR_ij = AR_vec(i);

        % Derive span from area and aspect ratio
        b_ij = sqrt(AR_ij * S_ij);   % b = sqrt(AR * S)           [m]

        % ── Induced drag factor ───────────────────────────────────────
        k_ij = 1 / (pi * AR_ij * e);

        % ── STALL SPEED (common to both TO and landing) ───────────────
        % Uses W_To for takeoff, W_land for landing.
        V_stall_TO   = sqrt(2 * W_To   / (rho * S_ij * CL_max));  % [m/s]
        V_stall_land = sqrt(2 * W_land / (rho * S_ij * CL_max));  % [m/s]

        % =============================================================
        %  TAKEOFF DISTANCE  (mirrors TakeoffDistance.m exactly)
        % =============================================================

        V_Lo  = 1.2 * V_stall_TO;          % Liftoff speed (FAR 23)    [m/s]
        V_avg = 0.7 * V_Lo;                % Average ground-roll speed [m/s]

        q_avg = 0.5 * rho * V_avg^2;       % Dynamic pressure at V_avg [Pa]
        q_Lo  = 0.5 * rho * V_Lo^2;        % Dynamic pressure at V_Lo  [Pa]

        % Ground-effect factor on induced drag (Raymer eq. 12.6)
        % phi → 0 deep in ground effect, phi → 1 out of it
        phi = (16 * h_wing / b_ij)^2 / ((16 * h_wing / b_ij)^2 + 1);

        % Aerodynamic forces during ground roll (at V_avg)
        L_avg = q_avg * S_ij * CL_max;
        D_gnd = q_avg * S_ij * CD_0_clean + phi * k_ij * W_To^2 / (q_avg * S_ij);

        % Net driving force during ground roll
        F_net = T_A - D_gnd - mu_r * (W_To - L_avg);

        if F_net <= 0
            % Cannot accelerate to liftoff — infeasible cell
            d_To_grid(i,j) = Inf;
        else
            % Ground roll distance (Anderson §6.7 energy method)
            d_Lo = (1.44 * W_To^2) / (rho * g * S_ij * CL_max * F_net);

            % Drag at liftoff for climb segment (no ground effect, conservative)
            D_Lo = q_Lo * S_ij * CD_0_clean + k_ij * W_To^2 / (q_Lo * S_ij);

            T_excess = T_A - D_Lo;   % Excess thrust at liftoff speed [N]

            if T_excess <= 0
                % Cannot climb — infeasible cell
                d_To_grid(i,j) = Inf;
            else
                % Climb distance to clear h_obs at V_Lo (linearised)
                ROC     = V_Lo * T_excess / W_To;   % Rate of climb [m/s]
                d_climb = h_obs * V_Lo / ROC;        % [m]

                d_To_grid(i,j) = d_Lo + d_climb;    % Total TO distance [m]
            end
        end

        % =============================================================
        %  LANDING DISTANCE  (mirrors LandingDist.m exactly)
        % =============================================================

        % Drag polar in landing configuration (full flap)
        CD_land = CD_0_land + k_ij * CL_max^2;   % Total CD at CL_max

        % Approach L/D ratio — governs glide slope and d_air
        % IMPORTANT: high L/D = flat glide slope = LONGER d_air
        % This is the fundamental STOL landing penalty.
        E_land = CL_max / CD_land;   % Approach L/D [-]

        % Air phase: obstacle clearance + flare geometry
        % h_obs = 15 m (50 ft), 0.133*Vstall^2/(2g) = flare distance contribution
        d_air = E_land * (15 + 0.133 * V_stall_land^2 / (2 * g));   % [m]

        % Ground roll from V_TD = 1.3*V_stall to rest
        V_TD  = 1.3 * V_stall_land;            % Touchdown speed [m/s]
        V_avg_land = 0.7 * V_TD;               % Average ground roll speed [m/s]
        q_land     = 0.5 * rho * V_avg_land^2; % Dynamic pressure [Pa]

        L_land = q_land * S_ij * CL_max;       % Lift reduces normal force [N]
        D_land = q_land * S_ij * CD_land;       % Aero drag assists stopping [N]

        % Ground roll — 1.69 is the landing analogue of the 1.44 TO factor
        % T_rev, D_land, and friction all retard the aircraft (denominator)
        d_ground = 1.69 * W_land^2 / ...
            (rho * g * S_ij * CL_max * ...
            (D_land + T_rev + mu_brake * (W_land - L_land)));   % [m]

        d_land_grid(i,j) = d_air + d_ground;   % Total landing distance [m]

    end
end

% =========================================================================
%  SECTION 4 — CLASSIFY EACH CELL (PASS / PARTIAL FAIL / FULL FAIL)
% =========================================================================
% Three exclusive states for the shading layer:
%   pass_TO   : d_To   <= d_req
%   pass_land : d_land <= d_req
%   Combination determines color zone.

pass_TO   = (d_To_grid   <= d_req);   % logical mask: passes TO constraint
pass_land = (d_land_grid <= d_req);   % logical mask: passes landing constraint

% Zone masks (used to build RGB image)
zone_green     = pass_TO & pass_land;           % Both pass  → green
zone_light_red = xor(pass_TO, pass_land);       % Exactly one fails → light red
zone_dark_red  = ~pass_TO & ~pass_land;         % Both fail → dark red

% =========================================================================
%  SECTION 5 — BUILD RGB IMAGE FOR SHADED BACKGROUND
% =========================================================================
% Construct a (nAR x nS x 3) RGB array directly — allows per-cell colour
% without pcolor artefacts.  Colours chosen for accessibility and clarity.

rgb = zeros(nAR, nS, 3);   % Preallocate RGB image

% Dark red  [0.65 0.10 0.10] — both constraints violated
rgb(:,:,1) = rgb(:,:,1) + 0.65 * zone_dark_red;
rgb(:,:,2) = rgb(:,:,2) + 0.10 * zone_dark_red;
rgb(:,:,3) = rgb(:,:,3) + 0.10 * zone_dark_red;

% Light red  [0.95 0.60 0.60] — one constraint violated
rgb(:,:,1) = rgb(:,:,1) + 0.95 * zone_light_red;
rgb(:,:,2) = rgb(:,:,2) + 0.60 * zone_light_red;
rgb(:,:,3) = rgb(:,:,3) + 0.60 * zone_light_red;

% Green  [0.20 0.70 0.30] — both constraints satisfied
rgb(:,:,1) = rgb(:,:,1) + 0.20 * zone_green;
rgb(:,:,2) = rgb(:,:,2) + 0.70 * zone_green;
rgb(:,:,3) = rgb(:,:,3) + 0.30 * zone_green;

% =========================================================================
%  SECTION 6 — FIGURE
% =========================================================================

figure('Name', 'Constraint Analysis: Takeoff + Landing Distance (S vs AR)', ...
       'Units', 'normalized', 'Position', [0.05 0.08 0.70 0.78]);

% ── Render shaded background ─────────────────────────────────────────────
% imagesc maps array indices to axis values via the S_vec / AR_vec limits.
imagesc(S_vec, AR_vec, rgb);

% MATLAB imagesc plots rows top-to-bottom; flip so AR increases upward
% (standard aerodynamic convention — large AR at top of plot).
set(gca, 'YDir', 'normal');
hold on;

% ── Takeoff distance contours ────────────────────────────────────────────
% Cap Inf values so contour() can operate (Inf cells will still be dark red)
d_To_plot = min(d_To_grid, 600);   % cap at 600 m for contouring

% Dense contours for spatial context (thin, white)
contour(S_vec, AR_vec, d_To_plot, ...
        linspace(50, 400, 15), ...
        'Color', [0.85 0.85 0.85], 'LineWidth', 0.6);

% The critical 500 ft takeoff contour — solid white, thick
[~, h_TO_req] = contour(S_vec, AR_vec, d_To_plot, [d_req d_req], ...
    'w-', 'LineWidth', 2.5);

% ── Landing distance contours ────────────────────────────────────────────
% Dense contours (thin, dark grey)
contour(S_vec, AR_vec, d_land_grid, ...
        linspace(50, 400, 15), ...
        'Color', [0.25 0.25 0.25], 'LineWidth', 0.6);

% The critical 500 ft landing contour — dashed black, thick
[~, h_land_req] = contour(S_vec, AR_vec, d_land_grid, [d_req d_req], ...
    'k--', 'LineWidth', 2.5);

% ── Legend proxy handles ──────────────────────────────────────────────────
% Patch handles for the shaded zones (not drawn, just for legend entries)
h_green  = patch(NaN, NaN, [0.20 0.70 0.30], 'EdgeColor', 'none', ...
                 'DisplayName', 'PASS both (TO \leq 500 ft & Land \leq 500 ft)');
h_lred   = patch(NaN, NaN, [0.95 0.60 0.60], 'EdgeColor', 'none', ...
                 'DisplayName', 'FAIL one constraint only');
h_dred   = patch(NaN, NaN, [0.65 0.10 0.10], 'EdgeColor', 'none', ...
                 'DisplayName', 'FAIL both constraints');
h_TO_ln  = plot(NaN, NaN, 'w-',  'LineWidth', 2.5, ...
                'DisplayName', sprintf('Takeoff d_{TO} = %.0f ft (%.1f m)', ...
                               d_req/0.3048, d_req));
h_ld_ln  = plot(NaN, NaN, 'k--', 'LineWidth', 2.5, ...
                'DisplayName', sprintf('Landing d_{land} = %.0f ft (%.1f m)', ...
                               d_req/0.3048, d_req));

legend([h_green, h_lred, h_dred, h_TO_ln, h_ld_ln], ...
       'Location', 'northeast', 'FontSize', 9.5, ...
       'Color', [0.95 0.95 0.95], 'EdgeColor', [0.4 0.4 0.4]);

% ── Axis labels and title ─────────────────────────────────────────────────
xlabel('Wing Reference Area  S  [m^2]', 'FontSize', 13);
ylabel('Aspect Ratio  AR  [-]',         'FontSize', 13);
title({'Constraint Analysis — Takeoff & Landing  |  AR vs. S', ...
       sprintf(['W_{TO} = %.0f N (%.0f kg)   T_A = %.0f N   ', ...
                'CL_{max} = %.1f   CD_{0,clean} = %.3f   ', ...
                'CD_{0,land} = %.3f'], ...
               W_To, W_To/g, T_A, CL_max, CD_0_clean, CD_0_land), ...
       sprintf(['\\mu_r = %.1f (gravel TO)   \\mu_{brake} = %.1f   ', ...
                '\\eta_{rev} = %.2f   T_{rev} = %.0f N'], ...
               mu_r, mu_brake, eta_rev, T_rev)}, ...
      'FontSize', 10.5);

% ── Formatting ────────────────────────────────────────────────────────────
grid on;
ax = gca;
ax.GridColor = [0.5 0.5 0.5];
ax.GridAlpha = 0.30;
ax.FontSize  = 11;
ax.Box       = 'on';
xlim([S_vec(1)  S_vec(end)]);
ylim([AR_vec(1) AR_vec(end)]);

% ── Disclaimer annotation ─────────────────────────────────────────────────
annotation('textbox', [0.12 0.005 0.78 0.038], ...
    'String', ['NOTE: W_{TO}, T_A, CD_0, CL_{max} are preliminary estimates. ' ...
               'Re-run after weight iteration and engine selection.'], ...
    'FontSize', 7.5, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'Color', [0.35 0.35 0.35]);

hold off;

% =========================================================================
%  SECTION 7 — CONSOLE SUMMARY
% =========================================================================

n_total      = nS * nAR;
n_green      = nnz(zone_green);
n_light_red  = nnz(zone_light_red);
n_dark_red   = nnz(zone_dark_red);

fprintf('========================================\n')
fprintf('  CONSTRAINT ANALYSIS SUMMARY\n')
fprintf('========================================\n')
fprintf('  W_To          : %.0f N  (%.0f kg)\n',   W_To, W_To/g)
fprintf('  W_land        : %.0f N  (%.0f kg)\n', W_land, W_land/g)
fprintf('  P_A           : %.0f kW  (%.0f hp)\n',  P_A/1000, P_A/shp2W)
fprintf('  T_A           : %.0f N  (%.0f lbf)\n',  T_A, T_A*0.2248)
fprintf('  T_rev         : %.0f N  (eta=%.2f)\n',  T_rev, eta_rev)
fprintf('  CL_max        : %.2f\n',  CL_max)
fprintf('  CD_0 (clean)  : %.4f\n',  CD_0_clean)
fprintf('  CD_0 (land)   : %.4f\n',  CD_0_land)
fprintf('  e             : %.2f\n',  e)
fprintf('  mu_r  (TO)    : %.2f\n',  mu_r)
fprintf('  mu_brake      : %.2f\n',  mu_brake)
fprintf('  h_wing        : %.1f m\n', h_wing)
fprintf('  Requirement   : d <= %.1f m  (500 ft)\n', d_req)
fprintf('----------------------------------------\n')
fprintf('  Grid size     : %d x %d  (%d cells)\n', nAR, nS, n_total)
fprintf('  PASS both     : %d cells  (%.1f%%)\n',  n_green,     100*n_green/n_total)
fprintf('  FAIL one      : %d cells  (%.1f%%)\n',  n_light_red, 100*n_light_red/n_total)
fprintf('  FAIL both     : %d cells  (%.1f%%)\n',  n_dark_red,  100*n_dark_red/n_total)
fprintf('----------------------------------------\n')
fprintf('  Min d_TO      : %.1f m  (%.0f ft)\n', min(d_To_grid(:)),   min(d_To_grid(:))/0.3048)
fprintf('  Min d_land    : %.1f m  (%.0f ft)\n', min(d_land_grid(:)), min(d_land_grid(:))/0.3048)
fprintf('========================================\n')
