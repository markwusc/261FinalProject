% ConstraintAnalysis_TO_Land_v2.m
% =========================================================================
% CONSTRAINT ANALYSIS v2 — AR vs S Design Space
% Single-engine (SE) and twin-engine (TE) configurations compared side by side.
%
% FIGURES PRODUCED
%   Figure 1 — Constraint diagram (takeoff + landing distance)   1×2 subplot
%   Figure 2 — Cruise L/D at optimum altitude                    1×2 subplot
%   Figure 3 — Optimum cruise altitude heatmap                   1×2 subplot
%
% COLOUR CONVENTION (Figure 1)
%   GREEN      Both d_TO ≤ d_marg  AND  d_land ≤ d_marg  (5% margin met)
%   YELLOW     Both d_TO ≤ d_req   AND  d_land ≤ d_req,   but outside 5% margin
%   LIGHT RED  Exactly one of {d_TO, d_land} exceeds d_req
%   DARK RED   Both d_TO > d_req   AND  d_land > d_req
%
% KEY MODELLING CHOICES
%   CD_0 scaling:
%     Takeoff (2.5×): partial flap ~20° adds ΔCD_0 ≈ 0.045 above clean.
%     Landing (5.0×): full flap + spoiler deployment. CD_0_land = 0.150.
%       NOTE — this is on the high end for this aircraft class (typical STOL
%       landing CD_0 ≈ 0.060–0.100). Revisit after flap-geometry drag buildup.
%
%   Thrust model: T_A = P_A / (0.7·V_Lo) for takeoff, P_A / (0.7·V_TD) for
%   landing. This approximates propeller thrust T = P/V at the mean ground-phase
%   speed. The same T_A_TO is reused for the climb check (at V_Lo), which is
%   slightly optimistic (actual thrust at V_Lo = P_A/V_Lo < P_A/(0.7·V_Lo)).
%   This is consistent with the energy-method approach and is conservative
%   overall because the ground-roll forces dominate the total distance.
%
%   Cruise weight = W_To throughout (max weight — conservative assumption).
%   True mid-cruise weight is lower, which would improve L/D and raise the
%   cruise altitude slightly.
%
%   Turboprop power lapse: P_A(h) = 0.9 · η · P_shp · (ρ(h)/ρ_SL)
%   StdAtmos.m must be on the MATLAB path (provided in project files).
% =========================================================================

clear; clc; close all;

% =========================================================================
%  SECTION 1 — SHARED PARAMETERS
% =========================================================================

% ── Physical constants ────────────────────────────────────────────────────
g       = 9.81;          % gravitational acceleration       [m/s^2]
rho_SL  = 1.225;         % sea-level ISA air density        [kg/m^3]
shp2W   = 745.7;         % shaft-hp → Watts
h_obs   = 15.24;         % 50 ft obstacle height            [m]
d_req   = 152.4;         % 500 ft field-length requirement  [m]
d_marg  = d_req * 0.95;  % 5% design margin  (≈ 144.78 m)  [m]

% ── Propulsion ────────────────────────────────────────────────────────────
P_shp    = 1050;   % rated shaft power per engine   [hp]
eta_prop = 0.80;   % propeller efficiency           [-]

% ── Drag polars ───────────────────────────────────────────────────────────
% Three states of flap/spoiler deployment — same base CD_0, scaled by config.
% NOTE: 5× landing CD_0 = 0.150. Typical STOL range is 0.060–0.100 for
% landing config; this value is conservative. Update after drag buildup.
CD_0_clean = 0.030;              % clean / cruise                   [-]
CD_0_TO    = 2.5 * CD_0_clean;  % takeoff partial flap  → 0.075    [-]
CD_0_land  = 5.0 * CD_0_clean;  % landing full flap + spoiler → 0.150 [-]

% ── Maximum lift coefficients ─────────────────────────────────────────────
% Per RFP §4.3: aircraft CL_max = 0.8 × airfoil CL_max.
CL_max_TO    = 2.4;        % takeoff flap setting       [-]
CL_max_land  = 2.6;        % full-flap landing config   [-]
CL_max_clean = 1.8;  % clean CL upper bound for cruise check [-]

% ── Wing / geometry ───────────────────────────────────────────────────────
e      = 0.9;   % Oswald efficiency factor (clean, RFP §4.3)  [-]
h_wing = 3.0;   % wing height above ground (high-wing STOL)   [m]

% ── Ground friction, braking, thrust reversal ─────────────────────────────
mu_r           = 0.40;  % rolling friction — gravel runway (RFP constant) [-]
mu_brake       = 0.50;  % braking friction — landing ground roll           [-]
eta_rev        = 0.45;  % thrust reversal effectiveness (PT6A beta-pitch)  [-]
liftKillFactor = 0.10;  % fraction of CL_max retained with spoilers active [-]
                         % Spoilers destroy lift → more weight on gear →
                         % larger normal force → larger braking friction force.

% ── Cruise (used for Figures 2 and 3) ─────────────────────────────────────
V_cruise = 130 * 0.5144;   % 130 KTAS → 66.87 m/s  [m/s]

% ── Fuel accounting ───────────────────────────────────────────────────────
zeta        = 0.139;  % fuel fraction  W_fuel / W_To  (both configs)  [-]
fuel_margin = 0.06;   % IFR reserve as fraction of total fuel          [-]
% Landing weight: W_land = W_To - W_fuel_burned
%   W_fuel_burned = zeta × W_To × (1 - fuel_margin)

% ── Sweep ranges ─────────────────────────────────────────────────────────
nS  = 200;
nAR = 200;
S_vec  = linspace(20, 70,  nS);    % wing reference area  [m^2]
AR_vec = linspace( 5, 14, nAR);    % aspect ratio         [-]

% ── Altitude search vector ────────────────────────────────────────────────
% Used to find the altitude where P_A(h) = P_R(h) at 90% throttle.
% Capped at 6000 m per design brief. If PA > PR at 6000 m, 6000 m is reported.
h_alt_vec = linspace(0, 6000, 301);           % [m]
[~, rho_alt, ~, ~] = StdAtmos(h_alt_vec);    % density vector at each altitude

% =========================================================================
%  SECTION 2 — AIRCRAFT CONFIGURATIONS
% =========================================================================
% Single engine: 42 kN MTOW, 1 × 1050 shp
% Twin engine:   61 kN MTOW, 2 × 1050 shp (P_A doubled)
% Both share the same zeta and fuel margin.

W_To_arr  = [42572, 64253];                          % MTOW             [N]
n_eng_arr = [1, 2];                                  % engine count
P_A_arr   = eta_prop * P_shp * shp2W * n_eng_arr;   % power available  [W]

% Landing weight
W_fuel_burned_arr = zeta .* W_To_arr .* (1 - fuel_margin);
W_land_arr        = W_To_arr - W_fuel_burned_arr;    % [N]

cfg_names = { ...
    'Single Engine  (42 kN,  1 \times 1050 shp)', ...
    'Twin Engine    (61 kN,  2 \times 1050 shp)' };

% Console: configuration summary
fprintf('=== Aircraft Configuration Summary ===\n')
for c = 1:2
    label = strrep(cfg_names{c}, '\times', 'x');
    fprintf('  Config %d: %s\n',                     c, label)
    fprintf('    W_To       = %6.0f N  (%5.0f kg)\n', W_To_arr(c),          W_To_arr(c)/g)
    fprintf('    W_land     = %6.0f N  (%5.0f kg)\n', W_land_arr(c),        W_land_arr(c)/g)
    fprintf('    P_A        = %6.1f kW  (%5.0f shp total)\n', ...
            P_A_arr(c)/1e3, P_A_arr(c)/shp2W)
    fprintf('    fuel burnt = %6.1f N  (zeta=%.3f, margin=%.0f%%)\n\n', ...
            W_fuel_burned_arr(c), zeta, fuel_margin*100)
end

% =========================================================================
%  SECTION 3 — PRE-ALLOCATE RESULT MATRICES
% =========================================================================
% Cell arrays indexed by configuration (index 1 = SE, 2 = TE).

d_To_g   = {zeros(nAR,nS), zeros(nAR,nS)};   % takeoff field length    [m]
d_land_g = {zeros(nAR,nS), zeros(nAR,nS)};   % landing field length    [m]
LD_g     = {zeros(nAR,nS), zeros(nAR,nS)};   % cruise L/D at h_opt     [-]
h_opt_g  = {zeros(nAR,nS), zeros(nAR,nS)};   % optimum cruise altitude [m]

% =========================================================================
%  SECTION 4 — DOUBLE SWEEP OVER (AR, S) FOR BOTH CONFIGURATIONS
% =========================================================================

for cfg = 1:2

    % Assign configuration-specific values for this outer iteration
    W_To   = W_To_arr(cfg);
    W_land = W_land_arr(cfg);
    P_A    = P_A_arr(cfg);

    for i = 1:nAR
        for j = 1:nS

            S  = S_vec(j);
            AR = AR_vec(i);
            b  = sqrt(AR * S);          % wing span derived from AR and S  [m]
            k  = 1 / (pi * AR * e);    % induced drag factor               [-]

            % Ground-effect correction on induced drag (Raymer eq. 12.6)
            % phi → 0 deep in ground effect (low, wide wing near ground)
            % phi → 1 out of ground effect
            phi = (16*h_wing/b)^2 / ((16*h_wing/b)^2 + 1);

            % =============================================================
            %  TAKEOFF DISTANCE
            %  Method: Anderson §6.7 energy method + linearised climb
            % =============================================================

            V_stall_TO = sqrt(2*W_To / (rho_SL*S*CL_max_TO));   % [m/s]
            V_Lo       = 1.2 * V_stall_TO;     % liftoff speed (FAR 23)   [m/s]

            % Propeller thrust at average ground-roll speed (0.7·V_Lo)
            % T = P/V evaluated at the mean acceleration phase speed.
            % This is the physically correct way to convert shaft power to
            % propeller thrust for a variable-pitch constant-speed propeller
            % operating near its design advance ratio.
            T_A_TO = P_A / (0.7 * V_Lo);       % [N]

            q_avg = 0.5 * rho_SL * (0.7*V_Lo)^2;   % dynamic pressure at V_avg [Pa]
            q_Lo  = 0.5 * rho_SL * V_Lo^2;          % dynamic pressure at V_Lo  [Pa]

            % Forces during ground roll (evaluated at V_avg = 0.7·V_Lo)
            L_avg = q_avg * S * CL_max_TO;
            D_gnd = q_avg*S*CD_0_TO + phi*k*W_To^2/(q_avg*S);   % takeoff CD_0
            F_net = T_A_TO - D_gnd - mu_r*(W_To - L_avg);

            if F_net <= 0
                % Net force cannot overcome friction — aircraft will not reach V_Lo
                d_To_val = Inf;
            else
                % Ground roll (Anderson energy method)
                d_Lo = (1.44*W_To^2) / (rho_SL*g*S*CL_max_TO*F_net);

                % Drag at liftoff for climb segment (no ground effect — conservative)
                D_Lo     = q_Lo*S*CD_0_TO + k*W_To^2/(q_Lo*S);
                % T_A_TO reused for climb check; this is evaluated at 0.7·V_Lo
                % which slightly overestimates thrust at V_Lo (optimistic climb)
                T_excess = T_A_TO - D_Lo;

                if T_excess <= 0
                    % Cannot sustain climb at V_Lo — infeasible
                    d_To_val = Inf;
                else
                    ROC      = V_Lo * T_excess / W_To;   % rate of climb [m/s]
                    d_climb  = h_obs * V_Lo / ROC;        % [m]
                    d_To_val = d_Lo + d_climb;             % total TO field length [m]
                end
            end

            % =============================================================
            %  LANDING DISTANCE
            %  Method: air phase (glide + flare) + ground roll deceleration
            % =============================================================

            V_stall_land = sqrt(2*W_land / (rho_SL*S*CL_max_land));
            V_TD         = 1.3 * V_stall_land;   % touchdown speed [m/s]

            % Propeller reverse thrust at average landing ground-roll speed
            T_A_land = P_A / (0.7 * V_TD);   % forward-equivalent thrust [N]
            T_rev    = eta_rev * T_A_land;    % effective reverse thrust  [N]

            % Landing drag polar (full flap) — high CD_0_land intentionally
            % reduces E_land (approach L/D), which shortens the air phase.
            % This is the aerodynamic STOL landing design lever.
            CD_land = CD_0_land + k * CL_max_land^2;
            E_land  = CL_max_land / CD_land;   % approach L/D [-]

            % Air phase: 50 ft obstacle clearance + flare geometry
            % The 0.133·Vstall²/(2g) term is the horizontal flare distance.
            % NOTE: high E_land = flat glide slope = LONGER d_air.
            % Large CD_0_land reduces E_land — intentionally traded for shorter field.
            d_air = E_land * (15 + 0.133*V_stall_land^2/(2*g));

            % Ground roll from V_TD to rest, with spoilers deployed
            q_land = 0.5 * rho_SL * (0.7*V_TD)^2;
            % liftKillFactor reduces lift: spoilers limit lift build-up,
            % increasing the net normal force on the gear → more brake friction
            L_land = q_land * S * CL_max_land * liftKillFactor;
            D_land = q_land * S * CD_land;   % aero drag assists deceleration
            denom  = D_land + T_rev + mu_brake*(W_land - L_land);

            if denom <= 0
                % Cannot decelerate — guard condition (should not occur with these params)
                d_land_val = Inf;
            else
                % Ground roll (landing analogue of 1.44 takeoff factor is 1.69)
                d_ground   = 1.69*W_land^2 / (rho_SL*g*S*CL_max_land*denom);
                d_land_val = d_air + d_ground;
            end

            % =============================================================
            %  OPTIMUM CRUISE ALTITUDE
            %  Find h where P_A(h) = P_R(h) at 90% throttle, V = V_cruise
            % =============================================================

            % Power available: turboprop lapse linear with density
            % (standard first-order approximation; actual lapse is slightly
            %  non-linear due to temperature effects on air mass flow)
            PA_h = 0.9 * P_A * (rho_alt / rho_SL);   % [W] — vector over h_alt_vec

            % Power required at constant TAS = V_cruise
            % P_R = D × V; D = parasite + induced (clean config for cruise)
            q_h  = 0.5 * rho_alt * V_cruise^2;
            D_h  = q_h*S*CD_0_clean + k*W_To^2./(q_h*S);
            PR_h = D_h * V_cruise;                     % [W] — vector

            % Invalidate altitudes where CL would exceed clean stall limit
            % (aircraft cannot fly at V_cruise at that altitude — too slow)
            CL_h  = W_To ./ (q_h * S);
            valid = CL_h <= CL_max_clean;
            PA_h(~valid) = NaN;
            PR_h(~valid) = NaN;

            % Power surplus: positive → aircraft can sustain V_cruise (or climb)
            surplus = PA_h - PR_h;

            % Find the highest altitude where surplus is still positive.
            % This is the cruise ceiling at 90% throttle and V_cruise.
            pos_idx = find(surplus > 0 & ~isnan(surplus));

            if isempty(pos_idx)
                % PA < PR even at sea level — cannot sustain cruise speed
                h_opt_val = 0;
            elseif pos_idx(end) == length(h_alt_vec)
                % Surplus remains positive at 6000 m — cap at search limit
                h_opt_val = h_alt_vec(end);
            else
                % Interpolate zero crossing for a smooth altitude estimate
                idx_a = pos_idx(end);
                idx_b = idx_a + 1;
                if isnan(surplus(idx_b))
                    % Next point invalidated by CL limit — use last valid point
                    h_opt_val = h_alt_vec(idx_a);
                else
                    h_opt_val = interp1([surplus(idx_a) surplus(idx_b)], ...
                                        [h_alt_vec(idx_a) h_alt_vec(idx_b)], 0);
                end
            end

            % =============================================================
            %  CRUISE L/D AT OPTIMUM ALTITUDE
            %  Evaluate L/D using clean polar at the (S,AR)-specific h_opt
            % =============================================================
            rho_opt = interp1(h_alt_vec, rho_alt, h_opt_val, 'linear', rho_alt(end));
            q_opt   = 0.5 * rho_opt * V_cruise^2;
            % Cap CL at clean stall limit — physically cannot exceed this
            CL_opt  = min(W_To / (q_opt * S), CL_max_clean);
            CD_opt  = CD_0_clean + k * CL_opt^2;
            LD_val  = CL_opt / CD_opt;

            % =============================================================
            %  STORE RESULTS
            % =============================================================
            d_To_g{cfg}(i,j)   = d_To_val;
            d_land_g{cfg}(i,j) = d_land_val;
            LD_g{cfg}(i,j)     = LD_val;
            h_opt_g{cfg}(i,j)  = h_opt_val;

        end % j (S sweep)
    end % i (AR sweep)

end % cfg

% =========================================================================
%  SECTION 5 — ZONE CLASSIFICATION AND RGB IMAGES
% =========================================================================

rgb_img = cell(1,2);
for c = 1:2
    rgb_img{c} = buildRGB(d_To_g{c}, d_land_g{c}, d_req, d_marg);
end

% =========================================================================
%  SECTION 6 — FIND AND REPORT 5% MARGIN CONTOUR INTERSECTION POINTS
% =========================================================================
% The "5% margin intersection" is the (S, AR) point that simultaneously lies
% closest to both the d_TO = d_marg contour and the d_land = d_marg contour.
% Method: minimise the objective  f = (d_TO - d_marg)^2 + (d_land - d_marg)^2
% over the grid.  The global minimiser is the grid point nearest the crossing.
% Inf values are capped before the objective is evaluated.

int_S    = zeros(1,2);
int_AR   = zeros(1,2);
int_irow = zeros(1,2);   % row index (AR dimension) of intersection
int_icol = zeros(1,2);   % col index (S dimension)  of intersection

for c = 1:2
    d_To_cap   = min(d_To_g{c},   1e5);
    d_land_cap = min(d_land_g{c}, 1e5);
    obj = (d_To_cap - d_marg).^2 + (d_land_cap - d_marg).^2;
    [~, lin_idx] = min(obj(:));
    [irow, icol] = ind2sub([nAR nS], lin_idx);
    int_irow(c) = irow;
    int_icol(c) = icol;
    int_AR(c)   = AR_vec(irow);
    int_S(c)    = S_vec(icol);
end

fprintf('=== 5%% Margin Contour Intersection ===\n')
for c = 1:2
    ir = int_irow(c);
    ic = int_icol(c);
    fprintf('  Config %d (%s):\n', c, strrep(cfg_names{c},'\times','x'))
    fprintf('    Intersection: S = %.1f m^2,  AR = %.2f\n', int_S(c), int_AR(c))
    fprintf('    d_TO   = %.1f m  (%.0f ft)\n', d_To_g{c}(ir,ic),   d_To_g{c}(ir,ic)/0.3048)
    fprintf('    d_land = %.1f m  (%.0f ft)\n', d_land_g{c}(ir,ic), d_land_g{c}(ir,ic)/0.3048)
    fprintf('\n')
end

% =========================================================================
%  SECTION 7 — FIGURE 1: CONSTRAINT DIAGRAM (1×2 SUBPLOT)
% =========================================================================

fig1 = figure('Name', 'Constraint Analysis: TO + Landing', ...
              'Units','normalized','Position',[0.02 0.05 0.95 0.75]);

ax_f1 = gobjects(1,2);

for p = 1:2

    ax_f1(p) = subplot(1,2,p);

    % ── Shaded background (zone classification) ──────────────────────────
    imagesc(S_vec, AR_vec, rgb_img{p});
    set(gca, 'YDir', 'normal');
    hold on;

    d_To_plot = min(d_To_g{p}, 600);   % cap Inf for contouring

    % ── Takeoff contours (white) ─────────────────────────────────────────
    % Context lines (faint)
    contour(S_vec, AR_vec, d_To_plot, linspace(50,400,12), ...
            'Color',[0.85 0.85 0.85], 'LineWidth',0.6);
    % Strict 500 ft requirement
    contour(S_vec, AR_vec, d_To_plot, [d_req  d_req],  'w-',  'LineWidth',2.5);
    % 5% margin (475 ft)
    contour(S_vec, AR_vec, d_To_plot, [d_marg d_marg], 'w--', 'LineWidth',1.8);

    % ── Landing contours (black) ──────────────────────────────────────────
    % Context lines (faint)
    contour(S_vec, AR_vec, d_land_g{p}, linspace(50,400,12), ...
            'Color',[0.30 0.30 0.30], 'LineWidth',0.6);
    % Strict 500 ft requirement
    contour(S_vec, AR_vec, d_land_g{p}, [d_req  d_req],  'k-',  'LineWidth',2.5);
    % 5% margin (475 ft)
    contour(S_vec, AR_vec, d_land_g{p}, [d_marg d_marg], 'k--', 'LineWidth',1.8);

    % ── Intersection marker ───────────────────────────────────────────────
    plot(int_S(p), int_AR(p), 'p', ...
         'MarkerSize',14, 'MarkerFaceColor','w', ...
         'MarkerEdgeColor','k', 'LineWidth',1.5);
    text(int_S(p) + 0.5, int_AR(p), ...
         sprintf(' S=%.1f m^2\n AR=%.2f', int_S(p), int_AR(p)), ...
         'FontSize',8, 'FontWeight','bold', ...
         'BackgroundColor',[0.97 0.97 0.97], 'EdgeColor','k', 'Margin',2);

    % ── Legend ────────────────────────────────────────────────────────────
    h_g  = patch(NaN,NaN, [0.20 0.70 0.30], 'EdgeColor','none');
    h_y  = patch(NaN,NaN, [0.95 0.90 0.10], 'EdgeColor','none');
    h_lr = patch(NaN,NaN, [0.95 0.60 0.60], 'EdgeColor','none');
    h_dr = patch(NaN,NaN, [0.65 0.10 0.10], 'EdgeColor','none');
    h_TO_s = plot(NaN,NaN, 'w-',  'LineWidth',2.5);
    h_TO_m = plot(NaN,NaN, 'w--', 'LineWidth',1.8);
    h_ld_s = plot(NaN,NaN, 'k-',  'LineWidth',2.5);
    h_ld_m = plot(NaN,NaN, 'k--', 'LineWidth',1.8);
    h_pt   = plot(NaN,NaN, 'p', 'MarkerSize',10, ...
                  'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);

    legend([h_g, h_y, h_lr, h_dr, h_TO_s, h_TO_m, h_ld_s, h_ld_m, h_pt], ...
           {'Both pass (5% margin)', ...
            'Both pass (strict only)', ...
            'One constraint fails', ...
            'Both constraints fail', ...
            sprintf('d_{TO} = 500 ft (%.1f m)', d_req), ...
            sprintf('d_{TO} = 475 ft (%.1f m, 5%% margin)', d_marg), ...
            sprintf('d_{land} = 500 ft (%.1f m)', d_req), ...
            sprintf('d_{land} = 475 ft (%.1f m, 5%% margin)', d_marg), ...
            '5% margin intersection'}, ...
           'Location','northeast', 'FontSize',7.0, ...
           'Color',[0.95 0.95 0.95], 'EdgeColor',[0.4 0.4 0.4]);

    % ── Labels ───────────────────────────────────────────────────────────
    xlabel('Wing Area  S  [m^2]',   'FontSize',12);
    ylabel('Aspect Ratio  AR  [-]', 'FontSize',12);
    title(strrep(cfg_names{p},'\times','×'), 'FontSize',11);

    ax = gca;
    ax.FontSize  = 10;
    ax.GridColor = [0.5 0.5 0.5];
    ax.GridAlpha = 0.30;
    grid on;
    xlim([S_vec(1) S_vec(end)]);
    ylim([AR_vec(1) AR_vec(end)]);
    hold off;

end

sgtitle({ ...
    'Constraint Analysis — Takeoff & Landing Field Length  |  AR vs S', ...
    sprintf('CL_{max,TO}=%.1f   CL_{max,land}=%.1f   CD_0=[%.3f / %.3f / %.3f]  (clean / TO / land)', ...
            CL_max_TO, CL_max_land, CD_0_clean, CD_0_TO, CD_0_land), ...
    sprintf('\\mu_r=%.2f   \\mu_{brake}=%.2f   \\eta_{rev}=%.2f   liftKill=%.2f   e=%.2f   h_{wing}=%.1f m   \\zeta=%.3f', ...
            mu_r, mu_brake, eta_rev, liftKillFactor, e, h_wing, zeta) }, ...
    'FontSize',10.5);

linkaxes(ax_f1);

% =========================================================================
%  SECTION 8 — FIGURE 2: CRUISE L/D (1×2 SUBPLOT)
% =========================================================================
% L/D computed at the (S,AR)-specific optimum altitude using clean polar.
% W = W_To (max weight — conservative). True value would be slightly higher.

% Shared colour limits across both subplots for direct comparison
LD_all  = [LD_g{1}(:); LD_g{2}(:)];
LD_clim = [floor(min(LD_all)*2)/2,  ceil(max(LD_all)*2)/2];

fig2 = figure('Name','Cruise L/D at Optimum Altitude', ...
              'Units','normalized','Position',[0.02 0.05 0.95 0.72]);
colormap(fig2, jet);

ax_f2 = gobjects(1,2);

for p = 1:2

    ax_f2(p) = subplot(1,2,p);

    contourf(S_vec, AR_vec, LD_g{p}, 20, 'LineColor','none');
    clim(LD_clim);
    hold on;
    [C_ld, h_ld] = contour(S_vec, AR_vec, LD_g{p}, 10, 'k-', 'LineWidth',0.8);
    clabel(C_ld, h_ld, 'FontSize',8, 'LabelSpacing',180);
    hold off;

    cb = colorbar;
    cb.Label.String = 'L/D at cruise  [-]';
    cb.FontSize = 10;

    xlabel('Wing Area  S  [m^2]',   'FontSize',12);
    ylabel('Aspect Ratio  AR  [-]', 'FontSize',12);
    title(strrep(cfg_names{p},'\times','×'), 'FontSize',11);

    ax = gca;
    ax.FontSize  = 10;
    ax.GridAlpha = 0.20;
    grid on;
    xlim([S_vec(1) S_vec(end)]);
    ylim([AR_vec(1) AR_vec(end)]);

end

sgtitle({ ...
    'Cruise L/D at Optimum Altitude  |  V_{cruise} = 130 KTAS', ...
    'L/D evaluated at the (S, AR)-specific altitude where P_A(h) = P_R(h) at 90% throttle', ...
    'Clean polar used (CD_{0,clean})  |  W = W_{To}  (conservative — max-weight cruise assumption)' }, ...
    'FontSize',10.5);

linkaxes(ax_f2);

% =========================================================================
%  SECTION 9 — FIGURE 3: OPTIMUM CRUISE ALTITUDE HEATMAP (1×2 SUBPLOT)
% =========================================================================
% Altitude where P_A(h) = P_R(h) at 90% throttle and V_cruise = 130 KTAS.
% Colour axis fixed at [0, 6] km (the search ceiling) for direct comparison.

fig3 = figure('Name','Optimum Cruise Altitude', ...
              'Units','normalized','Position',[0.02 0.05 0.95 0.72]);
colormap(fig3, parula);

ax_f3 = gobjects(1,2);

for p = 1:2

    ax_f3(p) = subplot(1,2,p);

    contourf(S_vec, AR_vec, h_opt_g{p}/1000, 20, 'LineColor','none');
    clim([0 6]);
    hold on;
    [C_h, h_hc] = contour(S_vec, AR_vec, h_opt_g{p}/1000, 8, ...
                           'k-', 'LineWidth',0.8);
    clabel(C_h, h_hc, 'FontSize',8, 'LabelSpacing',200);
    hold off;

    cb = colorbar;
    cb.Label.String = 'Cruise Altitude  [km]';
    cb.FontSize = 10;

    xlabel('Wing Area  S  [m^2]',   'FontSize',12);
    ylabel('Aspect Ratio  AR  [-]', 'FontSize',12);
    title(strrep(cfg_names{p},'\times','×'), 'FontSize',11);

    ax = gca;
    ax.FontSize  = 10;
    ax.GridAlpha = 0.20;
    grid on;
    xlim([S_vec(1) S_vec(end)]);
    ylim([AR_vec(1) AR_vec(end)]);

end

sgtitle({ ...
    'Optimum Cruise Altitude  —  Where P_A(h) = P_R(h) at 90% Throttle', ...
    sprintf('V_{cruise} = 130 KTAS  |  Lapse: P_A(h) = 0.9\\cdot\\eta\\cdot P_{shp}\\cdot(\\rho(h)/\\rho_{SL})  |  W = W_{To} (conservative)'), ...
    'Altitude capped at 6000 m; cells showing 6.0 km indicate PA > PR throughout the search range' }, ...
    'FontSize',10.5);

linkaxes(ax_f3);

% =========================================================================
%  LOCAL FUNCTION — buildRGB
% =========================================================================

function rgb = buildRGB(d_To, d_land, d_req, d_marg)
% buildRGB  Classify each (AR, S) cell into a constraint zone and return
%           a 3-channel RGB image array for use with imagesc.
%
%   Zones (mutually exclusive and exhaustive):
%     GREEN     : both d_TO  ≤ d_marg AND d_land ≤ d_marg   (5% margin met)
%     YELLOW    : both d_TO  ≤ d_req  AND d_land ≤ d_req,   but NOT green
%     LIGHT RED : XOR(d_TO ≤ d_req, d_land ≤ d_req)         (one fails strict)
%     DARK RED  : d_TO > d_req AND d_land > d_req            (both fail strict)
%
%   Inf cells automatically fall into LIGHT RED or DARK RED because Inf > d_req.

    % Boolean masks
    pass_TO_strict  = d_To   <= d_req;
    pass_ld_strict  = d_land <= d_req;
    pass_TO_marg    = d_To   <= d_marg;
    pass_ld_marg    = d_land <= d_marg;

    zone_green = pass_TO_marg  & pass_ld_marg;
    zone_yel   = (pass_TO_strict & pass_ld_strict) & ~zone_green;
    zone_lred  = xor(pass_TO_strict, pass_ld_strict);
    zone_dred  = ~pass_TO_strict & ~pass_ld_strict;

    % Build RGB image (zones are mutually exclusive, so direct addition is safe)
    rgb = zeros([size(d_To), 3]);
    rgb(:,:,1) = 0.65*zone_dred + 0.95*zone_lred + 0.95*zone_yel + 0.20*zone_green;
    rgb(:,:,2) = 0.10*zone_dred + 0.60*zone_lred + 0.90*zone_yel + 0.70*zone_green;
    rgb(:,:,3) = 0.10*zone_dred + 0.60*zone_lred + 0.10*zone_yel + 0.30*zone_green;
end
