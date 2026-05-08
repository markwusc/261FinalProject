% CA_TWvsWS_To_Land_v2.m
% =========================================================================
% CONSTRAINT ANALYSIS v2 — T/W vs W/S with Engine Power Locus
% Single Engine Configuration
%
% NEW IN v2 (vs v1):
%   - Engine power P [shp] is converted to P_A [W] and used to compute
%     the thrust available at each W/S via T_A = P_A / (0.7 * V_Lo(W/S)).
%     This traces a physically meaningful power locus curve through the
%     T/W vs W/S design space.
%   - A pink dotted curve overlays all three figures showing the full
%     engine locus: the T/W achievable at every W/S for this engine.
%   - A pink filled dot marks the current design point, defined by S_ref.
%   - Exact takeoff and landing distances are recomputed at the design
%     point (not interpolated from the grid) and reported with pass/fail.
%
% FIGURES
%   Figure 1 — Side-by-side heatmaps (TO distance | landing distance)
%              with engine locus curve and design point overlaid
%   Figure 2 — Combined zone-shaded constraint diagram with engine locus
%              curve, design point, and 5% margin intersection labelled
%
% VARIABLE ORDERING (most likely to change → least likely):
%   P_shp       — engine changes happen early and drive everything downstream
%   W_To        — weight estimate refines every iteration
%   zeta        — fuel fraction tied to mission/engine iteration
%   AR          — wing architecture; more fundamental than S alone
%   S_ref       — reference wing area (single design point within W/S sweep)
%   CD_0_clean  — refines after drag buildup is complete
%   CL_max_TO   — refines after airfoil and flap geometry are finalised
%   CL_max_land — same; landing flap setting may differ from TO
%   e           — Oswald efficiency; relatively stable after AR is set
%   liftKillFactor — spoiler geometry; set once and rarely revisited
%   eta         — propeller efficiency; stable for a given prop design
%   eta_rev     — reversal effectiveness; stable for a given engine/prop
%   mu_r        — RFP constant (gravel); should not change
%   mu_brake    — RFP constant (gravel braking); should not change
%
% DEPENDENCIES
%   None beyond base MATLAB. StdAtmos.m is NOT required for this script.
%
% VERSION: 2.0
% =========================================================================

clear; clc; close all;

% =========================================================================
%  SECTION 1 — DESIGN INPUTS  (edit these each iteration)
% =========================================================================

% ── Propulsion ────────────────────────────────────────────────────────────
P_shp = 1050;    % rated shaft power per engine                  [hp]
                  % PT6A-class turboprop. Change when engine is finalised.

% ── Weight ────────────────────────────────────────────────────────────────
W_To  = 42000;   % max takeoff weight, single engine             [N]
                  % (~4281 kg). Update every weight-loop iteration.

% ── Fuel fraction ─────────────────────────────────────────────────────────
zeta  = 0.131;   % W_fuel / W_To (both climb + cruise + reserve) [-]
                  % Derived from Breguet mission analysis. Revisit when
                  % engine SFC and mission profile are finalised.

% ── Wing geometry ─────────────────────────────────────────────────────────
AR    = 6.76;       % aspect ratio — fixed for this analysis         [-]
                  % AR = 7 chosen for STOL structural robustness.
                  % Lower AR → shorter span → lighter wing box.
                  % Revisit if aeroelastic or range constraints tighten.

S_ref = 37.3;    % reference wing area — current design point    [m^2]
                  % Sets W/S_design = W_To / S_ref.
                  % Derived from v2 constraint analysis intersection.
                  % Update when wing sizing is revised.

% ── Aerodynamics ──────────────────────────────────────────────────────────
CD_0_clean = 0.029;   % zero-lift parasite drag, clean config     [-]
                       % Preliminary estimate. Refine with full drag buildup
                       % (fuselage wetted area, empennage, interference).

CL_max_TO   = 2.54;   % max lift coefficient — takeoff flap        [-]
                       % = 0.8 * airfoil CL_max per RFP §4.3.
                       % Update after airfoil selection and flap sizing.

CL_max_land = 2.72;   % max lift coefficient — landing flap        [-]
                       % Full flap setting; typically 10-15% above TO value.
                       % Update after airfoil + flap geometry finalised.

e = 0.9;             % Oswald efficiency factor (clean, RFP §4.3) [-]
                       % Assumed constant; decreases with wing stores (→0.8).

liftKillFactor = 0.20; % fraction of CL_max retained with spoilers [-]
                        % Spoilers destroy lift → more weight on gear →
                        % more braking friction. Revisit after spoiler design.

% ── Propulsion efficiency ─────────────────────────────────────────────────
eta     = 0.80;  % propeller efficiency                           [-]
                  % Typical for a variable-pitch constant-speed prop.
                  % Stable once prop is sized; rarely changes after selection.

eta_rev = 0.60;  % thrust reversal effectiveness (PT6A beta-pitch)[-]
                  % Range 0.40–0.55 per manufacturer data.
                  % Conservative mid-range; stable after engine finalised.

% ── Ground friction ───────────────────────────────────────────────────────
mu_r     = 0.10; % rolling friction — gravel runway (RFP constant)[-]
                  % Do not change between missions; fixed by RFP.

mu_brake = 0.50; % braking friction — landing ground roll         [-]
                  % Slightly higher than mu_r (brakes locked vs rolling).
                  % Use 0.30 for wet/icy gravel sensitivity check.

% =========================================================================
%  SECTION 2 — DERIVED CONSTANTS  (do not edit below this line)
% =========================================================================

% ── Physical constants ────────────────────────────────────────────────────
g      = 9.81;    % gravitational acceleration                    [m/s^2]
rho_SL = 1.225;   % sea-level ISA air density                     [kg/m^3]
h_obs  = 15.24;   % 50 ft obstacle height                         [m]
shp2W  = 745.7;   % shaft-hp to Watts conversion                  [W/hp]

% ── Field-length requirements ─────────────────────────────────────────────
d_req  = 152.4;          % 500 ft strict requirement              [m]
d_marg = d_req * 0.95;   % 5% design margin (≈ 144.78 m)          [m]

% ── Power available ───────────────────────────────────────────────────────
% P_A is the shaft power delivered to the propeller disc.
% eta converts shaft power to thrust power: T * V = eta * P_shaft.
P_A = eta * P_shp * shp2W;   % thrust power available             [W]

% ── Fuel / landing weight ─────────────────────────────────────────────────
fuel_margin   = 0.03;                              % 6% IFR reserve [-]
W_fuel_burned = zeta * W_To * (1 - fuel_margin);  % fuel consumed  [N]
W_land        = W_To - W_fuel_burned;              % landing weight [N]

% ── Drag scaling ──────────────────────────────────────────────────────────
CD_0_TO   = 0.084;   % takeoff partial flap  → 0.075   [-]
%CD_0_land = 5.0 * CD_0_clean;   % full flap + spoiler   → 0.150   [-]
CD_0_land = 0.2;
% NOTE: CD_0_land = 0.150 is on the high end for this class
% (typical STOL range 0.060–0.100). Revisit after flap drag buildup.

% ── Wing geometry derived from fixed AR ───────────────────────────────────
k      = 1 / (pi * AR * e);     % induced drag factor (constant)  [-]
h_wing = 3.0;                    % wing height above ground        [m]

% ── Design point wing loading ─────────────────────────────────────────────
WS_design = W_To / S_ref;        % W/S at current design point     [N/m^2]

% ── Sweep ranges ─────────────────────────────────────────────────────────
nTW = 120;
nWS = 120;
TW_vec = linspace(0.25, 0.80, nTW);   % thrust-to-weight ratio  [-]
WS_vec = linspace(400,  1500, nWS);   % wing loading            [N/m^2]

% =========================================================================
%  SECTION 3 — ENGINE POWER LOCUS
% =========================================================================
% For a turboprop with fixed shaft power P_A, the available thrust at any
% wing loading is set by the takeoff ground-roll speed:
%
%   T_A(W/S) = P_A / (0.7 * V_Lo(W/S))
%
% where V_Lo = 1.2 * V_stall = 1.2 * sqrt(2 * W/S / (rho * CL_max_TO))
%
% This gives a unique T/W at each W/S, tracing a curve through the design
% space. Higher W/S → higher V_Lo → lower T_A → lower T/W.
% The curve falls monotonically from top-left to bottom-right.

V_stall_locus = sqrt(2 * WS_vec / (rho_SL * CL_max_TO));   % [m/s]
V_Lo_locus    = 1.2 * V_stall_locus;                        % [m/s]
T_A_locus     = P_A ./ (0.7 * V_Lo_locus);                  % [N]
TW_locus      = T_A_locus / W_To;                           % [-]

% ── Design point on the locus ─────────────────────────────────────────────
% Interpolate locus to find T/W at exactly W/S = WS_design
TW_design = interp1(WS_vec, TW_locus, WS_design, 'linear', 'extrap');
T_A_design = TW_design * W_To;   % thrust available at design point [N]
S_design   = S_ref;               % same as S_ref by definition
b_design   = sqrt(AR * S_design); % span at design point            [m]

% =========================================================================
%  SECTION 4 — EXACT DESIGN POINT DISTANCE CALCULATION
% =========================================================================
% Recompute d_TO and d_land analytically at the design point.
% This gives exact values independent of grid resolution.

% ── Ground-effect correction ──────────────────────────────────────────────
phi_design = (16*h_wing/b_design)^2 / ((16*h_wing/b_design)^2 + 1);

% ── Takeoff distance at design point ─────────────────────────────────────
V_stall_TO_dp = sqrt(2 * W_To / (rho_SL * S_design * CL_max_TO));
V_Lo_dp       = 1.2 * V_stall_TO_dp;
V_avg_TO_dp   = 0.7 * V_Lo_dp;

q_avg_dp = 0.5 * rho_SL * V_avg_TO_dp^2;
q_Lo_dp  = 0.5 * rho_SL * V_Lo_dp^2;

L_avg_dp = q_avg_dp * S_design * CL_max_TO;
D_gnd_dp = q_avg_dp*S_design*CD_0_TO + phi_design*k*W_To^2/(q_avg_dp*S_design);
F_net_dp = T_A_design - D_gnd_dp - mu_r*(W_To - L_avg_dp);

if F_net_dp <= 0
    d_To_dp  = Inf;
    dp_TO_status = 'FAIL (cannot accelerate to V_Lo)';
else
    d_Lo_dp    = (1.44*W_To^2) / (rho_SL*g*S_design*CL_max_TO*F_net_dp);
    D_Lo_dp    = q_Lo_dp*S_design*CD_0_TO + k*W_To^2/(q_Lo_dp*S_design);
    T_exc_dp   = T_A_design - D_Lo_dp;

    if T_exc_dp <= 0
        d_To_dp = Inf;
        dp_TO_status = 'FAIL (insufficient climb thrust)';
    else
        ROC_dp    = V_Lo_dp * T_exc_dp / W_To;
        d_clm_dp  = h_obs * V_Lo_dp / ROC_dp;
        d_To_dp   = d_Lo_dp + d_clm_dp;
        if d_To_dp <= d_marg
            dp_TO_status = sprintf('PASS — within 5%% margin (%.1f m <= %.1f m)', d_To_dp, d_marg);
        elseif d_To_dp <= d_req
            dp_TO_status = sprintf('PASS — strict only (%.1f m <= %.1f m, outside 5%% margin)', d_To_dp, d_req);
        else
            dp_TO_status = sprintf('FAIL (%.1f m > %.1f m)', d_To_dp, d_req);
        end
    end
end

% ── Landing distance at design point ─────────────────────────────────────
WS_land_dp   = W_land / S_design;
V_stall_ld_dp = sqrt(2 * WS_land_dp / (rho_SL * CL_max_land));
V_TD_dp       = 1.3 * V_stall_ld_dp;
V_avg_ld_dp   = 0.7 * V_TD_dp;
T_rev_dp      = eta_rev * T_A_design;

CD_land_dp = CD_0_land + k * CL_max_land^2;
E_land_dp  = CL_max_land / CD_land_dp;
d_air_dp   = E_land_dp * (15 + 0.133*V_stall_ld_dp^2/(2*g));

q_land_dp  = 0.5 * rho_SL * V_avg_ld_dp^2;
L_land_dp  = q_land_dp * S_design * CL_max_land * liftKillFactor;
D_land_dp  = q_land_dp * S_design * CD_land_dp;
denom_dp   = D_land_dp + T_rev_dp + mu_brake*(W_land - L_land_dp);

if denom_dp <= 0
    d_land_dp = Inf;
    dp_ld_status = 'FAIL (cannot decelerate)';
else
    d_gnd_dp  = 1.69*W_land^2 / (rho_SL*g*S_design*CL_max_land*denom_dp);
    d_land_dp = d_air_dp + d_gnd_dp;
    if d_land_dp <= d_marg
        dp_ld_status = sprintf('PASS — within 5%% margin (%.1f m <= %.1f m)', d_land_dp, d_marg);
    elseif d_land_dp <= d_req
        dp_ld_status = sprintf('PASS — strict only (%.1f m <= %.1f m, outside 5%% margin)', d_land_dp, d_req);
    else
        dp_ld_status = sprintf('FAIL (%.1f m > %.1f m)', d_land_dp, d_req);
    end
end

% ── Console: design point summary ────────────────────────────────────────
fprintf('=== Design Point Summary ===\n')
fprintf('  S_ref        : %.1f m^2\n',    S_ref)
fprintf('  W/S design   : %.1f N/m^2\n',  WS_design)
fprintf('  T/W design   : %.4f\n',         TW_design)
fprintf('  T_A design   : %.1f N  (%.0f lbf)\n', T_A_design, T_A_design*0.2248)
fprintf('  P_A          : %.1f kW  (%.0f shp)\n', P_A/1000, P_A/shp2W)
fprintf('  W_land       : %.0f N  (%.0f kg)\n', W_land, W_land/g)
fprintf('---\n')
fprintf('  d_TO  : %.1f m  (%.0f ft)  →  %s\n', d_To_dp,  d_To_dp/0.3048,  dp_TO_status)
fprintf('  d_land: %.1f m  (%.0f ft)  →  %s\n', d_land_dp, d_land_dp/0.3048, dp_ld_status)
fprintf('============================\n\n')

% =========================================================================
%  SECTION 5 — HEATMAP SWEEP OVER (T/W, W/S)
% =========================================================================

d_To_grid   = zeros(nTW, nWS);
d_land_grid = zeros(nTW, nWS);

for i = 1:nTW
    for j = 1:nWS

        TW = TW_vec(i);
        WS = WS_vec(j);

        T_A = TW * W_To;        % thrust at this T/W point        [N]
        S   = W_To / WS;        % wing area at this W/S           [m^2]
        b   = sqrt(AR * S);     % span                            [m]

        phi = (16*h_wing/b)^2 / ((16*h_wing/b)^2 + 1);

        WS_land = W_land / S;

        % ── Takeoff distance ─────────────────────────────────────────────
        V_stall_TO = sqrt(2 * WS / (rho_SL * CL_max_TO));
        V_Lo       = 1.2 * V_stall_TO;
        V_avg_TO   = 0.7 * V_Lo;

        q_avg = 0.5 * rho_SL * V_avg_TO^2;
        q_Lo  = 0.5 * rho_SL * V_Lo^2;

        L_avg = q_avg * S * CL_max_TO;
        D_gnd = q_avg*S*CD_0_TO + phi*k*W_To^2/(q_avg*S);
        F_net = T_A - D_gnd - mu_r*(W_To - L_avg);

        if F_net <= 0
            d_To_grid(i,j) = Inf;
        else
            d_Lo     = (1.44*W_To^2) / (rho_SL*g*S*CL_max_TO*F_net);
            D_Lo     = q_Lo*S*CD_0_TO + k*W_To^2/(q_Lo*S);
            T_excess = T_A - D_Lo;

            if T_excess <= 0
                d_To_grid(i,j) = Inf;
            else
                ROC            = V_Lo * T_excess / W_To;
                d_climb        = h_obs * V_Lo / ROC;
                d_To_grid(i,j) = d_Lo + d_climb;
            end
        end

        % ── Landing distance ─────────────────────────────────────────────
        V_stall_land = sqrt(2 * WS_land / (rho_SL * CL_max_land));
        V_TD         = 1.3 * V_stall_land;
        V_avg_land   = 0.7 * V_TD;
        T_rev        = eta_rev * T_A;

        CD_land = CD_0_land + k * CL_max_land^2;
        E_land  = CL_max_land / CD_land;
        d_air   = E_land * (15 + 0.133*V_stall_land^2/(2*g));

        q_land  = 0.5 * rho_SL * V_avg_land^2;
        L_land  = q_land * S * CL_max_land * liftKillFactor;
        D_land  = q_land * S * CD_land;
        denom   = D_land + T_rev + mu_brake*(W_land - L_land);

        if denom <= 0
            d_land_grid(i,j) = Inf;
        else
            d_ground         = 1.69*W_land^2 / (rho_SL*g*S*CL_max_land*denom);
            d_land_grid(i,j) = d_air + d_ground;
        end

    end
end

% =========================================================================
%  SECTION 6 — ZONE CLASSIFICATION AND RGB IMAGE
% =========================================================================

pass_TO_strict = d_To_grid   <= d_req;
pass_ld_strict = d_land_grid <= d_req;
pass_TO_marg   = d_To_grid   <= d_marg;
pass_ld_marg   = d_land_grid <= d_marg;

zone_green = pass_TO_marg  & pass_ld_marg;
zone_yel   = (pass_TO_strict & pass_ld_strict) & ~zone_green;
zone_lred  = xor(pass_TO_strict, pass_ld_strict);
zone_dred  = ~pass_TO_strict & ~pass_ld_strict;

rgb = zeros(nTW, nWS, 3);
rgb(:,:,1) = 0.65*zone_dred + 0.95*zone_lred + 0.95*zone_yel + 0.20*zone_green;
rgb(:,:,2) = 0.10*zone_dred + 0.60*zone_lred + 0.90*zone_yel + 0.70*zone_green;
rgb(:,:,3) = 0.10*zone_dred + 0.60*zone_lred + 0.10*zone_yel + 0.30*zone_green;

% =========================================================================
%  SECTION 7 — 5% MARGIN CONTOUR INTERSECTION POINT
% =========================================================================
% Find the (W/S, T/W) cell nearest the crossing of both 5% margin lines.
% Minimise: f = (d_TO - d_marg)^2 + (d_land - d_marg)^2

d_To_cap   = min(d_To_grid,   1e5);
d_land_cap = min(d_land_grid, 1e5);
obj = (d_To_cap - d_marg).^2 + (d_land_cap - d_marg).^2;

[~, lin_idx]       = min(obj(:));
[int_row, int_col] = ind2sub([nTW nWS], lin_idx);

int_TW_val = TW_vec(int_row);
int_WS_val = WS_vec(int_col);
int_dTo    = d_To_grid(int_row, int_col);
int_dLand  = d_land_grid(int_row, int_col);

fprintf('=== 5%% Margin Contour Intersection ===\n')
fprintf('  T/W    : %.4f\n',       int_TW_val)
fprintf('  W/S    : %.1f N/m^2\n', int_WS_val)
fprintf('  d_TO   : %.1f m  (%.0f ft)\n', int_dTo,   int_dTo/0.3048)
fprintf('  d_land : %.1f m  (%.0f ft)\n', int_dLand, int_dLand/0.3048)
fprintf('=======================================\n\n')

% =========================================================================
%  SECTION 8 — SHARED PLOT HELPERS
% =========================================================================
% Colours and sizes defined once so all three figures are consistent.

pink_rgb    = [1.00 0.41 0.71];   % hot pink for locus curve and design dot
locus_lw    = 1.8;                % locus curve line width
locus_ls    = ':';                % locus curve line style (dotted)
dp_msz      = 12;                 % design point marker size
int_msz     = 14;                 % intersection marker size

d_To_plot   = min(d_To_grid,   600);
d_land_plot = min(d_land_grid, 600);
d_clim      = [40, 300];

% Clip engine locus T/W to plot y-axis range so it doesn't run off the axes
TW_locus_clipped = min(max(TW_locus, TW_vec(1)), TW_vec(end));

% =========================================================================
%  SECTION 9 — FIGURE 1: SIDE-BY-SIDE DISTANCE HEATMAPS
% =========================================================================

fig1 = figure('Name','TO and Landing Distance Heatmaps (T/W vs W/S)', ...
              'Units','normalized','Position',[0.02 0.08 0.95 0.74]);
colormap(fig1, jet);

subplot_titles = {'Takeoff Distance  d_{TO}  [m]', ...
                  'Landing Distance  d_{land}  [m]'};
dist_grids     = {d_To_plot,  d_land_plot};
dist_full      = {d_To_grid,  d_land_grid};

ax1 = gobjects(1,2);

for p = 1:2

    ax1(p) = subplot(1,2,p);

    % ── Colour-filled distance field ─────────────────────────────────────
    contourf(WS_vec, TW_vec, dist_grids{p}, 25, 'LineColor','none');
    clim(d_clim);
    hold on;

    % Labelled context contours
    [C_ctx, h_ctx] = contour(WS_vec, TW_vec, dist_grids{p}, 10, ...
                             'k-', 'LineWidth', 0.7);
    clabel(C_ctx, h_ctx, 'FontSize', 7, 'LabelSpacing', 160);

    % Strict 500 ft and 5% margin boundary lines
    contour(WS_vec, TW_vec, dist_full{p}, [d_req  d_req],  'w-',  'LineWidth', 2.5);
    contour(WS_vec, TW_vec, dist_full{p}, [d_marg d_marg], 'w--', 'LineWidth', 1.8);

    % ── Engine power locus (pink dotted) ─────────────────────────────────
    % Shows the T/W delivered by this engine at every W/S.
    % The curve falls as W/S increases because higher W/S → higher V_Lo →
    % lower thrust for the same shaft power.
    plot(WS_vec, TW_locus_clipped, locus_ls, ...
         'Color', pink_rgb, 'LineWidth', locus_lw);

    % ── Design point (pink filled dot) ───────────────────────────────────
    plot(WS_design, TW_design, 'o', ...
         'MarkerSize', dp_msz, 'MarkerFaceColor', pink_rgb, ...
         'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
    text(WS_design + 15, TW_design, ...
         sprintf(' S=%.1f m^2\n W/S=%.0f\n T/W=%.3f', ...
                 S_ref, WS_design, TW_design), ...
         'FontSize', 7.5, 'FontWeight', 'bold', ...
         'BackgroundColor', [0.97 0.97 0.97], ...
         'EdgeColor', pink_rgb, 'Margin', 2);

    % ── Colorbar ─────────────────────────────────────────────────────────
    cb = colorbar;
    cb.Label.String = subplot_titles{p};
    cb.FontSize = 10;

    % ── Legend ───────────────────────────────────────────────────────────
    h_req  = plot(NaN,NaN, 'w-',  'LineWidth', 2.5);
    h_marg = plot(NaN,NaN, 'w--', 'LineWidth', 1.8);
    h_loc  = plot(NaN,NaN, locus_ls, 'Color', pink_rgb, 'LineWidth', locus_lw);
    h_dp   = plot(NaN,NaN, 'o', 'MarkerSize', 8, ...
                  'MarkerFaceColor', pink_rgb, 'MarkerEdgeColor','k','LineWidth',1.0);
    legend([h_req, h_marg, h_loc, h_dp], ...
           {sprintf('500 ft limit (%.1f m)', d_req), ...
            sprintf('5%% margin (%.1f m)',   d_marg), ...
            sprintf('Engine locus  P=%.0f shp', P_shp), ...
            sprintf('Design point  S=%.1f m^2', S_ref)}, ...
           'Location','northwest','FontSize',8, ...
           'Color',[0.15 0.15 0.15],'TextColor','w', ...
           'EdgeColor',[0.5 0.5 0.5]);

    xlabel('Wing Loading  W/S  [N/m^2]',    'FontSize', 12);
    ylabel('Thrust-to-Weight  T/W  [-]',    'FontSize', 12);
    title(subplot_titles{p},                'FontSize', 11);

    ax = gca;
    ax.FontSize  = 10;
    ax.GridColor = [0.8 0.8 0.8];
    ax.GridAlpha = 0.25;
    grid on;
    xlim([WS_vec(1) WS_vec(end)]);
    ylim([TW_vec(1) TW_vec(end)]);
    hold off;

end

sgtitle({ ...
    'Takeoff & Landing Distance  —  T/W vs W/S  |  Single Engine', ...
    sprintf('W_{To}=%.0f N  AR=%.0f  e=%.2f  CD_0=[%.3f/%.3f/%.3f]  (clean/TO/land)', ...
            W_To, AR, e, CD_0_clean, CD_0_TO, CD_0_land), ...
    sprintf('CL_{max,TO}=%.1f  CL_{max,land}=%.1f  P=%.0f shp  \\eta=%.2f  P_A=%.1f kW', ...
            CL_max_TO, CL_max_land, P_shp, eta, P_A/1000) }, ...
    'FontSize', 10.5);

linkaxes(ax1);

% =========================================================================
%  SECTION 10 — FIGURE 2: COMBINED ZONE-SHADED CONSTRAINT DIAGRAM
% =========================================================================

fig2 = figure('Name','Combined Constraint Diagram: T/W vs W/S', ...
              'Units','normalized','Position',[0.10 0.08 0.70 0.78]);

% ── Zone-shaded background ────────────────────────────────────────────────
imagesc(WS_vec, TW_vec, rgb);
set(gca, 'YDir', 'normal');
hold on;

% ── Takeoff distance contours (white) ────────────────────────────────────
contour(WS_vec, TW_vec, d_To_plot, linspace(50,400,12), ...
        'Color',[0.85 0.85 0.85],'LineWidth',0.6);
contour(WS_vec, TW_vec, d_To_grid, [d_req  d_req],  'w-',  'LineWidth',2.5);
contour(WS_vec, TW_vec, d_To_grid, [d_marg d_marg], 'w--', 'LineWidth',1.8);

% ── Landing distance contours (black) ─────────────────────────────────────
contour(WS_vec, TW_vec, d_land_plot, linspace(50,400,12), ...
        'Color',[0.30 0.30 0.30],'LineWidth',0.6);
contour(WS_vec, TW_vec, d_land_grid, [d_req  d_req],  'k-',  'LineWidth',2.5);
contour(WS_vec, TW_vec, d_land_grid, [d_marg d_marg], 'k--', 'LineWidth',1.8);

% ── Engine power locus (pink dotted) ─────────────────────────────────────
plot(WS_vec, TW_locus_clipped, locus_ls, ...
     'Color', pink_rgb, 'LineWidth', locus_lw + 0.4);

% ── 5% margin intersection point (star) ──────────────────────────────────
plot(int_WS_val, int_TW_val, 'p', ...
     'MarkerSize', int_msz, 'MarkerFaceColor','w', ...
     'MarkerEdgeColor','k', 'LineWidth',1.8);
text(int_WS_val + 12, int_TW_val, ...
     sprintf(' T/W=%.3f\n W/S=%.0f N/m^2\n d_{TO}=%.0f m (%.0f ft)\n d_{land}=%.0f m (%.0f ft)', ...
             int_TW_val, int_WS_val, ...
             int_dTo,    int_dTo/0.3048, ...
             int_dLand,  int_dLand/0.3048), ...
     'FontSize', 8, 'FontWeight','bold', ...
     'BackgroundColor',[0.97 0.97 0.97], ...
     'EdgeColor','k','Margin',3);

% ── Design point (pink dot with full annotation) ──────────────────────────
plot(WS_design, TW_design, 'o', ...
     'MarkerSize', dp_msz, 'MarkerFaceColor', pink_rgb, ...
     'MarkerEdgeColor','k', 'LineWidth',1.5);
text(WS_design + 12, TW_design - 0.035, ...
     sprintf([' S=%.1f m^2\n W/S=%.0f N/m^2\n T/W=%.3f\n' ...
              ' d_{TO}=%.0f m (%.0f ft)\n d_{land}=%.0f m (%.0f ft)'], ...
             S_ref, WS_design, TW_design, ...
             d_To_dp,   d_To_dp/0.3048, ...
             d_land_dp, d_land_dp/0.3048), ...
     'FontSize', 8, 'FontWeight','bold', ...
     'BackgroundColor',[0.97 0.97 0.97], ...
     'EdgeColor', pink_rgb, 'Margin', 3);

% ── Legend ────────────────────────────────────────────────────────────────
h_g   = patch(NaN,NaN,[0.20 0.70 0.30],'EdgeColor','none');
h_y   = patch(NaN,NaN,[0.95 0.90 0.10],'EdgeColor','none');
h_lr  = patch(NaN,NaN,[0.95 0.60 0.60],'EdgeColor','none');
h_dr  = patch(NaN,NaN,[0.65 0.10 0.10],'EdgeColor','none');
h_ts  = plot(NaN,NaN,'w-',  'LineWidth',2.5);
h_tm  = plot(NaN,NaN,'w--', 'LineWidth',1.8);
h_ls  = plot(NaN,NaN,'k-',  'LineWidth',2.5);
h_lm  = plot(NaN,NaN,'k--', 'LineWidth',1.8);
h_loc = plot(NaN,NaN, locus_ls,'Color',pink_rgb,'LineWidth',locus_lw+0.4);
h_dp  = plot(NaN,NaN,'o','MarkerSize',9, ...
             'MarkerFaceColor',pink_rgb,'MarkerEdgeColor','k','LineWidth',1.2);
h_int = plot(NaN,NaN,'p','MarkerSize',11, ...
             'MarkerFaceColor','w','MarkerEdgeColor','k','LineWidth',1.5);

legend([h_g,h_y,h_lr,h_dr,h_ts,h_tm,h_ls,h_lm,h_loc,h_dp,h_int], ...
       {'Both pass (5% margin)', ...
        'Both pass (strict only)', ...
        'One constraint fails', ...
        'Both constraints fail', ...
        sprintf('d_{TO} = 500 ft (%.1f m)',         d_req), ...
        sprintf('d_{TO} = 475 ft (%.1f m, 5%% margin)', d_marg), ...
        sprintf('d_{land} = 500 ft (%.1f m)',        d_req), ...
        sprintf('d_{land} = 475 ft (%.1f m, 5%% margin)', d_marg), ...
        sprintf('Engine locus  P=%.0f shp, \\eta=%.2f', P_shp, eta), ...
        sprintf('Design point  S=%.1f m^2  (W/S=%.0f N/m^2)', S_ref, WS_design), ...
        sprintf('5%% margin intersection  T/W=%.3f', int_TW_val)}, ...
       'Location','northeast','FontSize',8, ...
       'Color',[0.95 0.95 0.95],'EdgeColor',[0.4 0.4 0.4]);

xlabel('Wing Loading  W/S  [N/m^2]',       'FontSize',13);
ylabel('Thrust-to-Weight Ratio  T/W  [-]', 'FontSize',13);
title({ ...
    'Constraint Analysis  —  T/W vs W/S  |  Single Engine', ...
    sprintf('W_{To}=%.0f N (%.0f kg)   P=%.0f shp   \\eta=%.2f   P_A=%.1f kW   AR=%.0f   e=%.2f', ...
            W_To, W_To/g, P_shp, eta, P_A/1000, AR, e), ...
    sprintf('CD_0=[%.3f/%.3f/%.3f] (clean/TO/land)   CL_{max}=[%.1f TO/%.1f land]   \\zeta=%.3f   6%% fuel reserve', ...
            CD_0_clean, CD_0_TO, CD_0_land, CL_max_TO, CL_max_land, zeta) }, ...
    'FontSize',10);

ax2 = gca;
ax2.FontSize  = 11;
ax2.GridColor = [0.5 0.5 0.5];
ax2.GridAlpha = 0.28;
grid on;
ax2.Box = 'on';
xlim([WS_vec(1) WS_vec(end)]);
ylim([TW_vec(1) TW_vec(end)]);
hold off;
