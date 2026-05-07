% ConstraintAnalysis_TO_Land_v3.m
% =========================================================================
% CONSTRAINT ANALYSIS v3 — AR vs S Design Space
%
% FIGURES PRODUCED
%   Figure 1 — Single Engine constraint diagram only (zone-shaded, AR vs S)
%   Figure 2 — Both SE and TE side-by-side (1×2 subplot, AR vs S)
%
% CHANGES FROM v2
%   Physics updates (aligned with TakeoffDist.m and LandingDist.m):
%     ┌─────────────────────┬────────────────┬────────────────────────────┐
%     │ Parameter           │ v2             │ v3                         │
%     ├─────────────────────┼────────────────┼────────────────────────────┤
%     │ CD0_takeoff         │ 2.5×CD0_clean  │ CD0_clean + CD0_flaps_TO   │
%     │ CD0_approach        │ 5.0×CD0_clean  │ CD0_clean + CD0_flaps_land │
%     │ CD0_groundroll      │ (same as appch)│ CD0_approach + CD0_spoilers│
%     │ CL_max_TO           │ 2.40           │ 3.2 × 0.80 = 2.56          │
%     │ CL_max_land         │ 2.60           │ 3.4 × 0.80 = 2.72          │
%     │ mu_r (rolling)      │ 0.40           │ 0.10 (hard-packed gravel)  │
%     │ liftKillFactor      │ 0.30           │ 0.20                       │
%     │ eta_rev             │ 0.45           │ 0.60 (Raymer Ch.13)        │
%     └─────────────────────┴────────────────┴────────────────────────────┘
%   Pink dashed contour: d_Lo = 70% × 500 ft (106.68 m) boundary.
%     Shows where the takeoff GROUND ROLL alone consumes 70% of the
%     field-length budget. Cells to the right of this line are preferred
%     (d_Lo stays within 70% budget with room to climb over the obstacle).
%   Design point logic (automatic):
%     At the standard 5% margin intersection, evaluate d_Lo.
%     If d_Lo > d_Lo_pink: pink is the binding constraint →
%         design point = intersection of pink + 5% landing margin.
%     Otherwise: design point = intersection of 5% TO + 5% landing margins.
%   Figures 2 and 3 from v2 (Cruise L/D and Optimum Altitude) removed.
%   W_To and SHP values retained from v2.
%
% COLOUR CONVENTION (both figures)
%   GREEN      Both d_TO ≤ d_marg  AND  d_land ≤ d_marg  (5% margin met)
%   YELLOW     Both d_TO ≤ d_req   AND  d_land ≤ d_req,   outside 5% margin
%   LIGHT RED  Exactly one of {d_TO, d_land} exceeds d_req
%   DARK RED   Both d_TO > d_req   AND  d_land > d_req
%
% METHOD REFERENCES
%   Anderson, "Introduction to Flight", Ch. 6 (Takeoff and Landing)
%   Raymer,   "Aircraft Design: A Conceptual Approach", Ch. 12, 13, 17
% =========================================================================

clear; clc; close all;

% =========================================================================
%  SECTION 1 — SHARED PARAMETERS
% =========================================================================

% ── Physical constants ─────────────────────────────────────────────────────
g      = 9.81;     % gravitational acceleration        [m/s^2]
rho_SL = 1.225;    % sea-level ISA air density          [kg/m^3]
shp2W  = 745.7;    % shaft-hp → Watts conversion factor
h_obs  = 15.24;    % 50 ft obstacle height              [m]

% ── Field-length requirements ──────────────────────────────────────────────
d_req      = 152.4;          % 500 ft strict requirement         [m]
d_marg     = d_req * 0.95;   % 5% design margin  (≈ 144.78 m)   [m]
d_Lo_pink  = 0.70 * d_req;   % pink boundary: d_Lo = 70% × 500ft (= 106.68 m) [m]
                               % Cells at this boundary use 70% of the field
                               % budget for ground roll alone; the remaining
                               % 30% must accommodate obstacle clearance.

% ── Propulsion (retained from v2) ─────────────────────────────────────────
P_shp    = 1050;   % rated shaft power per engine   [hp]
eta_prop = 0.80;   % propeller efficiency during ground phase   [-]

% ── Drag buildup (new methodology — component-based) ──────────────────────
% All CD0 values built from first principles rather than scaling factors.
CD0_clean = 0.0288;   % parasite drag, clean cruise config [-]

% Takeoff flap drag increment (partial flap, 30 deg)
flapSpan_frac    = 0.80;   % fraction of semi-span covered by flaps
flapDefl_TO_deg  = 30;     % takeoff flap deflection [deg]
flapDefl_land_deg = 50;    % landing flap deflection [deg]  (full deployment)
CD0_flaps_TO   = 0.0023 * flapSpan_frac * flapDefl_TO_deg;    % = 0.0552
CD0_takeoff    = CD0_clean + CD0_flaps_TO;   % total CD0, takeoff config [-]

% Landing flap drag increment (full flap, 50 deg)
CD0_flaps_land = 0.0023 * flapSpan_frac * flapDefl_land_deg;  % = 0.0920
CD0_approach   = CD0_clean + CD0_flaps_land;   % total CD0, approach (air phase) [-]

% Spoiler drag increment (ground roll only)
% Formula: ΔCD0 = Cd,normal × (S_spoiler/S_ref) × sin(θ_spoiler)
% S_ref cancels when S_spoiler = (S_spoiler/S_ref) × S_ref — constant.
Cd_spoiler_normal = 1.28;   % spoiler panel normal drag coeff [-]
S_spoiler_ratio   = 0.10;   % S_spoiler,total / S_ref [-]
theta_spl_deg     = 60;     % spoiler deflection at touchdown [deg]
CD0_spoilers   = Cd_spoiler_normal * S_spoiler_ratio * sin(theta_spl_deg*pi/180);  % = 0.1109
CD0_groundroll = CD0_approach + CD0_spoilers;   % total CD0, ground roll [-]
% Spoilers NOT active airborne; CD0_approach used for d_air, CD0_groundroll for d_ground.

% ── Maximum lift coefficients ──────────────────────────────────────────────
% Per RFP §4.3: aircraft CL_max = 0.80 × airfoil/flap CL_max.
CL_max_TO   = 3.2 * 0.80;   % takeoff flap setting  = 2.56 [-]
CL_max_land = 3.4 * 0.80;   % full-flap landing     = 2.72 [-]

% ── Wing / geometry ────────────────────────────────────────────────────────
e      = 0.9;   % Oswald efficiency factor (clean, RFP §4.3) [-]
h_wing = 3.0;   % wing height above ground (high-wing STOL)  [m]

% ── Ground friction, braking, thrust reversal ──────────────────────────────
mu_r           = 0.10;   % rolling friction, hard-packed gravel (updated from v2's 0.40)
                           % Published data for compacted Arctic gravel: 0.05–0.12
mu_brake       = 0.50;   % braking friction, landing ground roll [-]
eta_rev        = 0.60;   % reverse-pitch propeller effectiveness (Raymer Ch.13) [-]
liftKillFactor = 0.20;   % fraction of CL_max retained with spoilers deployed [-]

% ── Fuel accounting (v2 method — consistent with W_To inputs) ──────────────
zeta        = 0.1295;   % fuel fraction  W_fuel / W_To [-]
fuel_margin = 0.03;    % IFR reserve as fraction of total fuel [-]
% W_fuel_burned = zeta × W_To × (1 - fuel_margin)
% W_land = W_To - W_fuel_burned

% ── Sweep ranges ───────────────────────────────────────────────────────────
nS  = 200;
nAR = 200;
S_vec  = linspace(20, 70, nS);    % wing reference area [m^2]
AR_vec = linspace( 5, 14, nAR);   % aspect ratio        [-]

% =========================================================================
%  SECTION 2 — AIRCRAFT CONFIGURATIONS  (W_To and SHP retained from v2)
% =========================================================================

W_To_arr  = [40000, 60441];                          % MTOW per config    [N]
n_eng_arr = [1, 2];                                  % engine count
P_A_arr   = eta_prop * P_shp * shp2W * n_eng_arr;   % propulsive power   [W]

W_fuel_burned_arr = zeta .* W_To_arr .* (1 - fuel_margin);
W_land_arr        = W_To_arr - W_fuel_burned_arr;    % landing weight     [N]

cfg_names = { ...
    'Single Engine  (42 kN,  1 \times 1050 shp)', ...
    'Twin Engine    (61 kN,  2 \times 1050 shp)' };

fprintf('=== v3 Parameter Summary ===\n')
fprintf('  CD0_clean      : %.4f\n',   CD0_clean)
fprintf('  CD0_flaps_TO   : %.4f  (0.0023 × %.2f × %.0f deg)\n', CD0_flaps_TO,   flapSpan_frac, flapDefl_TO_deg)
fprintf('  CD0_takeoff    : %.4f\n',   CD0_takeoff)
fprintf('  CD0_flaps_land : %.4f  (0.0023 × %.2f × %.0f deg)\n', CD0_flaps_land, flapSpan_frac, flapDefl_land_deg)
fprintf('  CD0_approach   : %.4f\n',   CD0_approach)
fprintf('  CD0_spoilers   : %.4f  (%.2f × %.2f × sin(%.0f deg))\n', CD0_spoilers, Cd_spoiler_normal, S_spoiler_ratio, theta_spl_deg)
fprintf('  CD0_groundroll : %.4f\n',   CD0_groundroll)
fprintf('  CL_max_TO      : %.4f  (3.2 × 0.80)\n', CL_max_TO)
fprintf('  CL_max_land    : %.4f  (3.4 × 0.80)\n', CL_max_land)
fprintf('  mu_r           : %.2f  (updated from v2 0.40)\n', mu_r)
fprintf('  liftKillFactor : %.2f  (updated from v2 0.30)\n', liftKillFactor)
fprintf('  eta_rev        : %.2f  (Raymer Ch.13, updated from v2 0.45)\n', eta_rev)
fprintf('  d_Lo_pink      : %.2f m  (70%% of 500ft)\n\n', d_Lo_pink)

fprintf('=== Aircraft Configuration Summary ===\n')
for c = 1:2
    label = strrep(cfg_names{c}, '\times', 'x');
    fprintf('  Config %d: %s\n', c, label)
    fprintf('    W_To   = %6.0f N  (%5.0f kg)\n', W_To_arr(c), W_To_arr(c)/g)
    fprintf('    W_land = %6.0f N  (%5.0f kg)\n', W_land_arr(c), W_land_arr(c)/g)
    fprintf('    P_A    = %6.1f kW  (%5.0f shp total)\n\n', P_A_arr(c)/1e3, P_A_arr(c)/shp2W)
end

% =========================================================================
%  SECTION 3 — PRE-ALLOCATE RESULT MATRICES
% =========================================================================

d_To_g   = {zeros(nAR,nS), zeros(nAR,nS)};   % total takeoff distance [m]
d_Lo_g   = {zeros(nAR,nS), zeros(nAR,nS)};   % ground roll only       [m]  ← new in v3
d_land_g = {zeros(nAR,nS), zeros(nAR,nS)};   % total landing distance [m]

% =========================================================================
%  SECTION 4 — DOUBLE SWEEP OVER (cfg, AR, S)
% =========================================================================

for cfg = 1:2

    W_To   = W_To_arr(cfg);
    W_land = W_land_arr(cfg);
    P_A    = P_A_arr(cfg);

    for i = 1:nAR
        for j = 1:nS

            S  = S_vec(j);
            AR = AR_vec(i);
            b  = sqrt(AR * S);         % wing span [m]
            k  = 1 / (pi * AR * e);   % induced drag factor [-]

            % Ground-effect correction on induced drag (Raymer eq. 12.6)
            phi = (16*h_wing/b)^2 / ((16*h_wing/b)^2 + 1);

            % =============================================================
            %  TAKEOFF DISTANCE
            %  Method: Anderson §6.7 energy method + linearised climb
            % =============================================================

            V_stall_TO = sqrt(2*W_To / (rho_SL*S*CL_max_TO));   % [m/s]
            V_Lo       = 1.2 * V_stall_TO;   % FAR 23 liftoff speed [m/s]

            % Speed-varying thrust: T = P_A / V evaluated at 0.7·V_Lo
            % (mean ground-roll speed). This differs from v2's fixed T_A.
            T_A_TO = P_A / (0.7 * V_Lo);   % [N]

            q_avg = 0.5 * rho_SL * (0.7*V_Lo)^2;   % q at mean gnd speed [Pa]
            q_Lo  = 0.5 * rho_SL * V_Lo^2;          % q at liftoff speed   [Pa]

            % Forces at average ground-roll speed
            L_avg = q_avg * S * CL_max_TO;
            D_gnd = q_avg*S*CD0_takeoff + phi*k*W_To^2/(q_avg*S);
            F_net = T_A_TO - D_gnd - mu_r*(W_To - L_avg);

            if F_net <= 0
                % Cannot accelerate to V_Lo
                d_To_g{cfg}(i,j) = Inf;
                d_Lo_g{cfg}(i,j) = Inf;
                continue
            end

            % Ground roll (Anderson 1.44 factor)
            d_Lo = (1.44*W_To^2) / (rho_SL*g*S*CL_max_TO*F_net);
            d_Lo_g{cfg}(i,j) = d_Lo;   % store for pink contour

            % Obstacle clearance (climb segment at V_Lo)
            % Thrust re-evaluated at V_Lo — slightly lower than at 0.7·V_Lo
            T_A_Lo   = P_A / V_Lo;
            D_Lo     = q_Lo*S*CD0_takeoff + k*W_To^2/(q_Lo*S);
            T_excess = T_A_Lo - D_Lo;

            if T_excess <= 0
                % Cannot sustain climb at V_Lo
                d_To_g{cfg}(i,j) = Inf;
                continue
            end

            ROC     = V_Lo * T_excess / W_To;   % rate of climb [m/s]
            d_climb = h_obs * V_Lo / ROC;        % obstacle clearance distance [m]
            d_To_g{cfg}(i,j) = d_Lo + d_climb;

            % =============================================================
            %  LANDING DISTANCE
            %  Method: air phase (E·obstacle formula) + deceleration roll
            % =============================================================

            V_stall_land = sqrt(2*W_land / (rho_SL*S*CL_max_land));   % [m/s]
            V_TD         = 1.3 * V_stall_land;   % touchdown speed [m/s]

            % Speed-varying reverse thrust at average ground-roll speed
            T_A_land = P_A / (0.7 * V_TD);   % forward equivalent [N]
            T_rev    = eta_rev * T_A_land;    % reverse thrust [N]

            % Air phase: drag polar at CL_max_land, CD0_approach (no spoilers)
            CD_app = CD0_approach + k * CL_max_land^2;
            E_land = CL_max_land / CD_app;   % approach L/D [-]
            d_air  = E_land * (15 + 0.133*V_stall_land^2/(2*g));   % [m]

            % Ground roll: CD0_groundroll (spoilers) + liftKillFactor
            CD_gnd = CD0_groundroll + k * CL_max_land^2;
            q_land = 0.5 * rho_SL * (0.7*V_TD)^2;
            L_land = q_land * S * CL_max_land * liftKillFactor;   % spoilers kill lift
            D_land = q_land * S * CD_gnd;
            denom  = D_land + T_rev + mu_brake*(W_land - L_land);

            if denom <= 0
                d_land_g{cfg}(i,j) = Inf;
                continue
            end

            d_ground = 1.69*W_land^2 / (rho_SL*g*S*CL_max_land*denom);
            d_land_g{cfg}(i,j) = d_air + d_ground;

        end % j (S)
    end % i (AR)

end % cfg

% =========================================================================
%  SECTION 5 — ZONE CLASSIFICATION AND RGB IMAGES
% =========================================================================

rgb_img = cell(1,2);
for c = 1:2
    rgb_img{c} = buildRGB(d_To_g{c}, d_land_g{c}, d_req, d_marg);
end

% =========================================================================
%  SECTION 6 — DESIGN POINT FINDING (both configs)
% =========================================================================
% For each config:
%   Step 1. Find STANDARD intersection: min‖(d_TO - d_marg)² + (d_land - d_marg)²‖
%   Step 2. Evaluate d_Lo at that standard intersection point.
%   Step 3. If d_Lo > d_Lo_pink → pink is the binding constraint.
%           Design point = min‖(d_Lo - d_Lo_pink)² + (d_land - d_marg)²‖
%           Otherwise design point = standard intersection.
%
% Rationale: if the ground roll alone already exceeds 70% of the 500 ft
% budget at the nominal design point, then the TO ground roll constraint
% is tighter than the full-distance TO margin, and we must move to larger
% S or AR to satisfy both the pink and landing margin simultaneously.

dp_S            = zeros(1,2);
dp_AR           = zeros(1,2);
dp_irow         = zeros(1,2);
dp_icol         = zeros(1,2);
dp_pink_binding = false(1,2);

for c = 1:2

    d_To_cap   = min(d_To_g{c},  1e5);
    d_land_cap = min(d_land_g{c}, 1e5);
    d_Lo_cap   = min(d_Lo_g{c},  1e5);

    % Standard 5% margin intersection
    obj_std = (d_To_cap - d_marg).^2 + (d_land_cap - d_marg).^2;
    [~, lin_std] = min(obj_std(:));
    [irow_std, icol_std] = ind2sub([nAR nS], lin_std);

    % Check if pink is binding at the standard intersection
    d_Lo_at_std   = d_Lo_g{c}(irow_std, icol_std);
    pink_binding  = isfinite(d_Lo_at_std) && (d_Lo_at_std > d_Lo_pink);
    dp_pink_binding(c) = pink_binding;

    if pink_binding
        % Shift design point to where ground roll budget + landing margin both met
        obj_pink = (d_Lo_cap - d_Lo_pink).^2 + (d_land_cap - d_marg).^2;
        [~, lin_pink] = min(obj_pink(:));
        [irow_dp, icol_dp] = ind2sub([nAR nS], lin_pink);
    else
        irow_dp = irow_std;
        icol_dp = icol_std;
    end

    dp_irow(c) = irow_dp;
    dp_icol(c) = icol_dp;
    dp_S(c)    = S_vec(icol_dp);
    dp_AR(c)   = AR_vec(irow_dp);

end

% ── Print design point summary ────────────────────────────────────────────

fprintf('=== Design Point Summary ===\n')
for c = 1:2
    ir  = dp_irow(c);
    ic  = dp_icol(c);
    k_dp      = 1 / (pi * dp_AR(c) * e);
    CD_app_dp = CD0_approach + k_dp * CL_max_land^2;   % approach drag polar
    E_dp      = CL_max_land / CD_app_dp;               % approach L/D
    gamma_dp  = atan(1/E_dp) * (180/pi);               % glide angle [deg]

    if dp_pink_binding(c)
        basis_str = 'pink (d_Lo = 70% x 500ft) + 5% landing margin';
    else
        basis_str = '5% TO margin + 5% landing margin';
    end

    fprintf('  Config %d  (%s):\n', c, strrep(cfg_names{c}, '\times', 'x'))
    fprintf('    Design basis  : %s\n', basis_str)
    fprintf('    S             = %.1f m^2\n', dp_S(c))
    fprintf('    AR            = %.2f\n',     dp_AR(c))
    fprintf('    b             = %.2f m\n',   sqrt(dp_AR(c)*dp_S(c)))
    fprintf('    d_TO          = %.1f m  (%.0f ft)\n', d_To_g{c}(ir,ic), d_To_g{c}(ir,ic)/0.3048)
    fprintf('    d_Lo          = %.1f m  (%.0f ft)  [%.0f%% of 500ft]\n', ...
            d_Lo_g{c}(ir,ic), d_Lo_g{c}(ir,ic)/0.3048, 100*d_Lo_g{c}(ir,ic)/d_req)
    fprintf('    d_land        = %.1f m  (%.0f ft)\n', d_land_g{c}(ir,ic), d_land_g{c}(ir,ic)/0.3048)
    fprintf('    L/D (appch)   = %.3f\n', E_dp)
    fprintf('    Glide angle   = %.2f deg  (cot(gamma) = E = %.3f)\n', gamma_dp, E_dp)
    fprintf('\n')
end

% =========================================================================
%  SECTION 7 — FIGURE 1: SINGLE ENGINE CONSTRAINT DIAGRAM
% =========================================================================
% Full zone-shaded constraint diagram for the primary (SE) configuration.
% Includes the pink d_Lo boundary and the automatic design point marker.

c_primary = 1;   % Figure 1 shows SE configuration

fig1 = figure('Name', 'Constraint Analysis v3: Single Engine', ...
              'Units', 'normalized', 'Position', [0.02 0.08 0.60 0.80]);

% ── Zone-shaded background ────────────────────────────────────────────────
imagesc(S_vec, AR_vec, rgb_img{c_primary});
set(gca, 'YDir', 'normal');
hold on;

d_To_plot  = min(d_To_g{c_primary},  600);
d_Lo_plot  = min(d_Lo_g{c_primary},  600);

% ── Faint context contours ────────────────────────────────────────────────
contour(S_vec, AR_vec, d_To_plot, linspace(50,400,12), ...
        'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
contour(S_vec, AR_vec, d_land_g{c_primary}, linspace(50,400,12), ...
        'Color', [0.30 0.30 0.30], 'LineWidth', 0.5);

% ── Takeoff contours (white) ──────────────────────────────────────────────
contour(S_vec, AR_vec, d_To_plot, [d_req  d_req],  'w-',  'LineWidth', 2.5);
contour(S_vec, AR_vec, d_To_plot, [d_marg d_marg], 'w--', 'LineWidth', 1.8);

% ── Landing contours (black) ──────────────────────────────────────────────
contour(S_vec, AR_vec, d_land_g{c_primary}, [d_req  d_req],  'k-',  'LineWidth', 2.5);
contour(S_vec, AR_vec, d_land_g{c_primary}, [d_marg d_marg], 'k--', 'LineWidth', 1.8);

% ── Pink dashed: d_Lo = 70% × 500ft boundary ─────────────────────────────
% Each cell on this contour has a takeoff ground roll (d_Lo) that equals
% 70% of the field-length requirement. Left of this line: ground roll is
% within budget. Right: ground roll alone exceeds 70% of 500ft.
contour(S_vec, AR_vec, d_Lo_plot, [d_Lo_pink d_Lo_pink], ...
        '--', 'Color', [1.0 0.4 0.7], 'LineWidth', 2.2);

% ── Design point marker ───────────────────────────────────────────────────
ir = dp_irow(c_primary);
ic = dp_icol(c_primary);
plot(dp_S(c_primary), dp_AR(c_primary), 'p', ...
     'MarkerSize', 16, 'MarkerFaceColor', 'w', ...
     'MarkerEdgeColor', 'k', 'LineWidth', 1.8);

if dp_pink_binding(c_primary)
    dp_label = sprintf(' S=%.1f m^2\n AR=%.2f\n(pink binding)', ...
                       dp_S(c_primary), dp_AR(c_primary));
else
    dp_label = sprintf(' S=%.1f m^2\n AR=%.2f', ...
                       dp_S(c_primary), dp_AR(c_primary));
end
text(dp_S(c_primary) + 0.6, dp_AR(c_primary), dp_label, ...
     'FontSize', 8, 'FontWeight', 'bold', ...
     'BackgroundColor', [0.97 0.97 0.97], 'EdgeColor', 'k', 'Margin', 2);

% ── Legend ────────────────────────────────────────────────────────────────
h_g    = patch(NaN,NaN, [0.20 0.70 0.30], 'EdgeColor','none');
h_y    = patch(NaN,NaN, [0.95 0.90 0.10], 'EdgeColor','none');
h_lr   = patch(NaN,NaN, [0.95 0.60 0.60], 'EdgeColor','none');
h_dr   = patch(NaN,NaN, [0.65 0.10 0.10], 'EdgeColor','none');
h_TO_s = plot(NaN,NaN, 'w-',  'LineWidth', 2.5);
h_TO_m = plot(NaN,NaN, 'w--', 'LineWidth', 1.8);
h_ld_s = plot(NaN,NaN, 'k-',  'LineWidth', 2.5);
h_ld_m = plot(NaN,NaN, 'k--', 'LineWidth', 1.8);
h_pink = plot(NaN,NaN, '--',   'Color', [1.0 0.4 0.7], 'LineWidth', 2.2);
h_pt   = plot(NaN,NaN, 'p', 'MarkerSize', 11, ...
              'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth', 1.5);

legend([h_g, h_y, h_lr, h_dr, h_TO_s, h_TO_m, h_ld_s, h_ld_m, h_pink, h_pt], ...
       {'Both pass (5% margin)', ...
        'Both pass (strict only)', ...
        'One constraint fails', ...
        'Both constraints fail', ...
        sprintf('d_{TO} = 500 ft  (%.1f m)',            d_req), ...
        sprintf('d_{TO} = 5%% margin  (%.1f m)',         d_marg), ...
        sprintf('d_{land} = 500 ft  (%.1f m)',           d_req), ...
        sprintf('d_{land} = 5%% margin  (%.1f m)',       d_marg), ...
        sprintf('d_{Lo} = 70%%×500ft  (%.1f m)',         d_Lo_pink), ...
        'Design point'}, ...
       'Location', 'northeast', 'FontSize', 7.5, ...
       'Color', [0.95 0.95 0.95], 'EdgeColor', [0.4 0.4 0.4]);

xlabel('Wing Area  S  [m^2]',   'FontSize', 12);
ylabel('Aspect Ratio  AR  [-]', 'FontSize', 12);
title({ ...
    sprintf('Constraint Analysis v3  —  %s', strrep(cfg_names{c_primary},'\times','×')), ...
    sprintf('CD0: TO=%.4f  |  appch=%.4f  |  gnd=%.4f  |  CL_{max}: TO=%.2f  land=%.2f', ...
            CD0_takeoff, CD0_approach, CD0_groundroll, CL_max_TO, CL_max_land), ...
    sprintf('\\mu_r=%.2f  \\mu_{brake}=%.2f  \\eta_{rev}=%.2f  liftKill=%.2f  e=%.2f  h_{wing}=%.1f m', ...
            mu_r, mu_brake, eta_rev, liftKillFactor, e, h_wing)}, ...
    'FontSize', 10);

ax1 = gca;
ax1.FontSize  = 10;
ax1.GridColor = [0.5 0.5 0.5];
ax1.GridAlpha = 0.25;
grid on;
xlim([S_vec(1) S_vec(end)]);
ylim([AR_vec(1) AR_vec(end)]);
hold off;

% =========================================================================
%  SECTION 8 — FIGURE 2: BOTH CONFIGURATIONS (1×2 SUBPLOT)
% =========================================================================

fig2 = figure('Name', 'Constraint Analysis v3: SE and TE', ...
              'Units', 'normalized', 'Position', [0.02 0.05 0.95 0.75]);

ax_f2 = gobjects(1,2);

for p = 1:2

    ax_f2(p) = subplot(1,2,p);

    imagesc(S_vec, AR_vec, rgb_img{p});
    set(gca, 'YDir', 'normal');
    hold on;

    d_To_plot_p = min(d_To_g{p}, 600);
    d_Lo_plot_p = min(d_Lo_g{p}, 600);

    % Faint context contours
    contour(S_vec, AR_vec, d_To_plot_p, linspace(50,400,12), ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
    contour(S_vec, AR_vec, d_land_g{p}, linspace(50,400,12), ...
            'Color', [0.30 0.30 0.30], 'LineWidth', 0.5);

    % Takeoff (white)
    contour(S_vec, AR_vec, d_To_plot_p, [d_req  d_req],  'w-',  'LineWidth', 2.5);
    contour(S_vec, AR_vec, d_To_plot_p, [d_marg d_marg], 'w--', 'LineWidth', 1.8);

    % Landing (black)
    contour(S_vec, AR_vec, d_land_g{p}, [d_req  d_req],  'k-',  'LineWidth', 2.5);
    contour(S_vec, AR_vec, d_land_g{p}, [d_marg d_marg], 'k--', 'LineWidth', 1.8);

    % Pink: d_Lo = 70% × 500ft boundary
    contour(S_vec, AR_vec, d_Lo_plot_p, [d_Lo_pink d_Lo_pink], ...
            '--', 'Color', [1.0 0.4 0.7], 'LineWidth', 2.2);

    % Design point marker
    ir_p = dp_irow(p);
    ic_p = dp_icol(p);
    plot(dp_S(p), dp_AR(p), 'p', ...
         'MarkerSize', 14, 'MarkerFaceColor', 'w', ...
         'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    if dp_pink_binding(p)
        dp_lbl = sprintf(' S=%.1f\n AR=%.2f\n(pink)', dp_S(p), dp_AR(p));
    else
        dp_lbl = sprintf(' S=%.1f\n AR=%.2f', dp_S(p), dp_AR(p));
    end
    text(dp_S(p) + 0.5, dp_AR(p), dp_lbl, ...
         'FontSize', 7.5, 'FontWeight', 'bold', ...
         'BackgroundColor', [0.97 0.97 0.97], 'EdgeColor', 'k', 'Margin', 2);

    % Legend (only on first subplot to avoid clutter)
    if p == 1
        h_g_2  = patch(NaN,NaN, [0.20 0.70 0.30], 'EdgeColor','none');
        h_y_2  = patch(NaN,NaN, [0.95 0.90 0.10], 'EdgeColor','none');
        h_lr_2 = patch(NaN,NaN, [0.95 0.60 0.60], 'EdgeColor','none');
        h_dr_2 = patch(NaN,NaN, [0.65 0.10 0.10], 'EdgeColor','none');
        h_TO_s2 = plot(NaN,NaN, 'w-',  'LineWidth', 2.5);
        h_TO_m2 = plot(NaN,NaN, 'w--', 'LineWidth', 1.8);
        h_ld_s2 = plot(NaN,NaN, 'k-',  'LineWidth', 2.5);
        h_ld_m2 = plot(NaN,NaN, 'k--', 'LineWidth', 1.8);
        h_pk_2  = plot(NaN,NaN, '--', 'Color', [1.0 0.4 0.7], 'LineWidth', 2.2);
        h_pt_2  = plot(NaN,NaN, 'p', 'MarkerSize', 10, ...
                       'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);
        legend([h_g_2, h_y_2, h_lr_2, h_dr_2, h_TO_s2, h_TO_m2, ...
                h_ld_s2, h_ld_m2, h_pk_2, h_pt_2], ...
               {'Both pass (5% margin)', 'Both pass (strict)', ...
                'One fails', 'Both fail', ...
                sprintf('d_{TO}=500ft'),  sprintf('d_{TO} 5%% marg'), ...
                sprintf('d_{land}=500ft'), sprintf('d_{land} 5%% marg'), ...
                sprintf('d_{Lo}=70%%×500ft'), 'Design point'}, ...
               'Location', 'northeast', 'FontSize', 6.5, ...
               'Color', [0.95 0.95 0.95], 'EdgeColor', [0.4 0.4 0.4]);
    end

    xlabel('Wing Area  S  [m^2]',   'FontSize', 12);
    ylabel('Aspect Ratio  AR  [-]', 'FontSize', 12);
    title(strrep(cfg_names{p}, '\times', '×'), 'FontSize', 11);

    ax = gca;
    ax.FontSize  = 10;
    ax.GridColor = [0.5 0.5 0.5];
    ax.GridAlpha = 0.28;
    grid on;
    xlim([S_vec(1) S_vec(end)]);
    ylim([AR_vec(1) AR_vec(end)]);
    hold off;

end

sgtitle({ ...
    'Constraint Analysis v3  —  Takeoff & Landing Field Length  |  AR vs S', ...
    sprintf('CD0: TO=%.4f  appch=%.4f  gnd=%.4f  |  CL_{max}: TO=%.2f  land=%.2f', ...
            CD0_takeoff, CD0_approach, CD0_groundroll, CL_max_TO, CL_max_land), ...
    sprintf('\\mu_r=%.2f  \\mu_{brake}=%.2f  \\eta_{rev}=%.2f  liftKill=%.2f  e=%.2f  h_{wing}=%.1f m  \\zeta=%.3f', ...
            mu_r, mu_brake, eta_rev, liftKillFactor, e, h_wing, zeta)}, ...
    'FontSize', 10.5);

linkaxes(ax_f2);

% =========================================================================
%  LOCAL FUNCTION — buildRGB
% =========================================================================
% Classifies each (AR, S) cell into a constraint zone and returns a
% 3-channel RGB image for use with imagesc.
%
% Zones (mutually exclusive and exhaustive):
%   GREEN     : d_TO ≤ d_marg AND d_land ≤ d_marg   (5% margin met for both)
%   YELLOW    : d_TO ≤ d_req  AND d_land ≤ d_req,   but not GREEN
%   LIGHT RED : XOR(d_TO ≤ d_req, d_land ≤ d_req)   (exactly one fails)
%   DARK RED  : d_TO > d_req  AND d_land > d_req     (both fail)

function rgb = buildRGB(d_To, d_land, d_req, d_marg)

    pass_TO_strict = d_To   <= d_req;
    pass_ld_strict = d_land <= d_req;
    pass_TO_marg   = d_To   <= d_marg;
    pass_ld_marg   = d_land <= d_marg;

    zone_green = pass_TO_marg  & pass_ld_marg;
    zone_yel   = (pass_TO_strict & pass_ld_strict) & ~zone_green;
    zone_lred  = xor(pass_TO_strict, pass_ld_strict);
    zone_dred  = ~pass_TO_strict & ~pass_ld_strict;

    rgb = zeros([size(d_To), 3]);
    rgb(:,:,1) = 0.65*zone_dred + 0.95*zone_lred + 0.95*zone_yel + 0.20*zone_green;
    rgb(:,:,2) = 0.10*zone_dred + 0.60*zone_lred + 0.90*zone_yel + 0.70*zone_green;
    rgb(:,:,3) = 0.10*zone_dred + 0.60*zone_lred + 0.10*zone_yel + 0.30*zone_green;

end
