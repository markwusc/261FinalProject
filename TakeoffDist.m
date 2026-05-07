%% TakeoffDist.m
%{
Takeoff distance trade study for Arctic STOL aircraft.
Generates a contour heatmap of total takeoff distance as a function of
wing area (S) and aspect ratio (AR).

Takeoff distance is broken into two phases:
    d_To = d_Lo + d_climb

    d_Lo    : Ground roll from rest to liftoff speed V_Lo = 1.2*V_stall.
              Uses the Anderson energy method (factor 1.44).
              Thrust is derived from shaft power at V_avg = 0.7*V_Lo, so
              T_A varies at every (AR, S) cell as V_Lo changes with S.
              Ground-effect reduction on induced drag (Raymer eq. 12.6).

    d_climb : Linearised constant-speed climb from liftoff to 50 ft obstacle
              clearance. Based on the excess-thrust rate-of-climb method.

CD0 in takeoff configuration is built up from:
    CD0_takeoff = CD0_clean + CD0_flaps
    CD0_flaps   = 0.0023 * flapSpanFraction * flapDeflection_deg

Method references:
    Anderson, "Introduction to Flight", Ch. 6 (Takeoff Performance)
    Raymer,   "Aircraft Design: A Conceptual Approach", Ch. 12 & 17

ITERATION NOTE:
    W_To, SHP, CD0_clean, and CL_max_TO are estimates at this stage.
    Re-run this script every weight loop iteration as these values converge.
%}

clear; clc;

%% ============================================================
%%  SECTION 1 — USER INPUTS  (update every design iteration)
%% ============================================================

% --- Weight ---
W_To = 40000;   % Max takeoff gross weight [N]
                 % Update each iteration as weight estimate refines

% --- Powerplant ---
% Shaft horsepower is the primary engine sizing input for a turboprop.
% Converting to available propulsive power via propeller efficiency eta_prop.
SHP      = 1050;   % Total shaft horsepower, BOTH engines [hp]
                    % e.g. 2x PT6A-27 (~620 SHP each) → 1240 SHP
                    %      2x PT6A-20 (~578 SHP each) → 1156 SHP
                    % Refine after engine selection.
eta_prop = 0.80;   % Propeller efficiency during ground roll / initial climb
                    % 0.80 is a reasonable mid-range value for a fixed-pitch
                    % or coarse-pitch setting during the takeoff roll.
                    % Fine-pitch / variable-pitch props may reach 0.85 at V_Lo.

% Convert SHP to available shaft power [W], then to propulsive power [W]
P_shaft = SHP * 745.7;          % shaft power [W]  (1 hp = 745.7 W exactly)
P_A     = eta_prop * P_shaft;   % available propulsive power [W]
% NOTE: T_A = P_A / V is computed INSIDE the sweep loop because V_avg changes
% with S (via V_stall → V_Lo → V_avg = 0.7*V_Lo). This is the key difference
% from TradestudyTakeoff_S_AR.m, which used a fixed T_A for all cells.

% --- Drag: clean configuration ---
CD0_clean = 0.027;   % Parasite drag coefficient in clean (cruise) config [--]
                      % Typical for this aircraft class: 0.025–0.035.
                      % Refine after drag buildup (see Appendix A).

% --- Drag: flap contribution ---
% Empirical formula for flap parasite drag increment:
%   CD0_flaps = 0.0023 * (flapSpanFraction) * (flapDeflection_deg)
% Source: Raymer, "Aircraft Design: A Conceptual Approach", flap drag tables.
% This is a first-order estimate; a full drag polar (from Appendix A) will
% be more accurate once flap chord and hinge geometry are defined.
flapSpanFraction   = 0.80;   % Fraction of half-span covered by flaps [--]
                               % 0.80 means flaps extend over 80% of semi-span
flapDeflection_deg = 30;     % Flap deflection angle for takeoff setting [deg]
                               % Typical takeoff: 15–30 deg (partial flap)
                               % Do NOT use full landing deflection here
CD0_flaps   = 0.0023 * flapSpanFraction * flapDeflection_deg;   % [--]
CD0_takeoff = CD0_clean + CD0_flaps;   % Total parasite drag at takeoff [--]

% --- Lift: takeoff configuration ---
% CL_max_takeoff uses a 80% margin on the airfoil/flap CLmax per RFP §4.3.
% The base value 3.2 assumes an aggressive high-lift airfoil + Fowler flaps;
% update once airfoil selection is finalised.
CL_max_TO = 3.2 * 0.80;   % Aircraft CL_max in takeoff config [--] = 2.56

% --- Aerodynamic efficiency ---
e = 0.9;   % Oswald efficiency factor [--]
            % 0.9 for clean config per RFP; reduce to 0.8 if wing stores added

% --- Wing height above ground (ground-effect correction) ---
% Raymer eq. 12.6: phi → 0 deep in ground effect (low wing height relative to span)
%                  phi → 1 out of ground effect (high wing)
% High-wing STOL configuration assumed.
h_wing = 3.0;   % Wing height above ground [m] — update from CAD geometry

% --- Rolling friction ---
% ⚠️  DESIGN PHILOSOPHY WARNING:
%     mu_r = 0.1 is used here as requested. Note that TakeoffDistance.m and
%     the constraint analysis (CA_TWvsWS_To_Land.m) both use mu_r = 0.40 for
%     gravel. Published values for gravel/unprepared surfaces are typically
%     0.04–0.10 (hard-packed) to 0.3–0.5 (soft/loose gravel).
%     mu_r = 0.4 in the other scripts appears excessively high for a hard-packed
%     gravel strip (more representative of soft sand or turf) and likely
%     over-penalises the ground roll.
%     mu_r = 0.1 (hard-packed gravel/compacted dirt) is more consistent with
%     published data for Arctic gravel strips (e.g., USAF AFSC runway friction
%     data for compacted gravel: mu_r ≈ 0.05–0.12).
%     RECOMMENDATION: resolve this discrepancy between scripts before finalising
%     the constraint analysis — it will significantly shift feasible region boundaries.
mu_r = 0.1;   % Rolling friction coefficient (hard-packed gravel, takeoff roll) [--]

% --- Heatmap sweep ranges ---
AR_range = linspace(6,  14, 80);   % Aspect ratio sweep [--]
S_range  = linspace(20, 65, 80);   % Wing reference area [m^2]

%% ============================================================
%%  SECTION 2 — DERIVED PARAMETERS AND CONSOLE HEADER
%% ============================================================

g     = 9.81;    % gravitational acceleration [m/s^2]
rho   = 1.225;   % sea-level ISA air density [kg/m^3]
h_obs = 15.24;   % 50 ft obstacle in metres [m]

d_To_req  = 152.4;   % 500 ft field-length requirement [m]
d_To_marg = 152.4 * 0.90;   % 10% design margin target (= 137.2 m) [m]

fprintf('=== TakeoffDist.m — Parameter Summary ===\n')
fprintf('  W_To          : %6.0f N  (%.0f kg)\n',   W_To, W_To/g)
fprintf('  SHP (total)   : %6.0f hp  →  P_shaft = %.1f kW\n', SHP, P_shaft/1e3)
fprintf('  eta_prop      : %.2f  →  P_A = %.1f kW\n', eta_prop, P_A/1e3)
fprintf('  CD0_clean     : %.4f\n', CD0_clean)
fprintf('  flapSpan%%     : %.0f%%\n', flapSpanFraction*100)
fprintf('  flapDeflect   : %.0f deg\n', flapDeflection_deg)
fprintf('  CD0_flaps     : %.4f  (= 0.0023 × %.2f × %.0f)\n', ...
    CD0_flaps, flapSpanFraction, flapDeflection_deg)
fprintf('  CD0_takeoff   : %.4f  (clean + flaps)\n', CD0_takeoff)
fprintf('  CL_max_TO     : %.3f  (= 3.2 × 0.80)\n', CL_max_TO)
fprintf('  e             : %.2f\n', e)
fprintf('  h_wing        : %.1f m\n', h_wing)
fprintf('  mu_r          : %.2f  (hard-packed gravel)\n', mu_r)
fprintf('  d_req         : %.1f m  (500 ft)\n', d_To_req)
fprintf('  d_margin      : %.1f m  (~10%% margin)\n\n', d_To_marg)

%% ============================================================
%%  SECTION 3 — TAKEOFF DISTANCE SWEEP  (AR x S double loop)
%% ============================================================

d_To_map    = zeros(length(AR_range), length(S_range));
d_Lo_map    = zeros(length(AR_range), length(S_range));    % component breakout
d_climb_map = zeros(length(AR_range), length(S_range));    % component breakout
T_A_map     = zeros(length(AR_range), length(S_range));    % available thrust map

for i = 1:length(AR_range)
    for j = 1:length(S_range)

        AR = AR_range(i);
        S  = S_range(j);

        % Wing span and induced drag factor
        b = sqrt(AR * S);            % wing span [m]
        k = 1 / (pi * AR * e);      % induced drag factor [--]

        % ---- GROUND-EFFECT CORRECTION ----
        % Raymer eq. 12.6: reduces induced drag while wing is near the ground.
        % phi → 0 means large ground-effect benefit (low h_wing / large span)
        % phi → 1 means no ground effect (high wing or long span)
        phi = (16*h_wing/b)^2 / ((16*h_wing/b)^2 + 1);   % [--]

        % ---- CHARACTERISTIC SPEEDS ----
        % Stall speed at takeoff weight, sea-level, full-flap CL_max
        V_stall = sqrt(2*W_To / (rho*S*CL_max_TO));   % [m/s]

        % Liftoff speed per FAR Part 23: V_Lo = 1.2 * V_stall
        V_Lo = 1.2 * V_stall;   % [m/s]

        % Average ground roll speed (energy method): V_avg = 0.7 * V_Lo
        % This is the speed at which all average forces are evaluated
        V_avg = 0.7 * V_Lo;   % [m/s]

        % ---- AVAILABLE THRUST (speed-dependent) ----
        % For a constant-power turboprop: T = P_A / V
        % Thrust is evaluated at V_avg because that is the representative
        % speed over the entire ground roll (energy method assumption).
        % IMPORTANT: This T_A will be different for every (AR, S) cell
        % because V_Lo (and thus V_avg) changes with S through V_stall.
        % Larger S → lower V_stall → lower V_avg → higher T_A.
        % This captures the physical behaviour of a constant-power engine:
        % it produces more thrust at lower speeds, benefiting larger wings.
        T_A = P_A / V_avg;   % available thrust at V_avg [N]

        % ---- DYNAMIC PRESSURES ----
        q_avg = 0.5 * rho * V_avg^2;   % at average ground roll speed [Pa]
        q_Lo  = 0.5 * rho * V_Lo^2;   % at liftoff speed [Pa]

        % ---- AERODYNAMIC FORCES DURING GROUND ROLL ----
        % Lift at V_avg — reduces normal force and therefore rolling friction
        L_avg = q_avg * S * CL_max_TO;   % [N]

        % Drag at V_avg — includes ground-effect correction (phi) on induced drag
        % phi < 1 reduces k_eff → lower induced drag in ground effect → less drag
        D_gnd = q_avg*S*CD0_takeoff + phi*k*W_To^2/(q_avg*S);   % [N]

        % ---- NET GROUND ROLL FORCE ----
        % Net propulsive force: thrust minus aero drag minus rolling friction.
        % Rolling friction acts on the effective normal force (W - L), which
        % decreases as lift builds — the benefit of high CL_max on takeoff.
        F_net = T_A - D_gnd - mu_r*(W_To - L_avg);   % [N]

        if F_net <= 0
            % Cannot accelerate to V_Lo — cell is infeasible (underpowered /
            % too small a wing for this weight and rolling friction)
            d_To_map(i,j)    = Inf;
            d_Lo_map(i,j)    = Inf;
            d_climb_map(i,j) = Inf;
            T_A_map(i,j)     = T_A;
            continue   % skip to next cell
        end

        % ---- GROUND ROLL DISTANCE (Anderson energy method) ----
        % d_Lo = 1.44 * W^2 / (rho * g * S * CL_max * F_net_avg)
        % Factor 1.44 accounts for the integral from V=0 to V=V_Lo,
        % assuming F_net is approximately constant at its V_avg value.
        d_Lo = (1.44 * W_To^2) / (rho * g * S * CL_max_TO * F_net);   % [m]

        % ---- AERODYNAMIC FORCES AT LIFTOFF (for climb segment) ----
        % Ground effect is neglected at liftoff (conservative — more drag,
        % less climb performance). Thrust re-evaluated at V_Lo for the climb.
        T_A_Lo   = P_A / V_Lo;                                        % thrust at V_Lo [N]
        D_Lo     = q_Lo*S*CD0_takeoff + k*W_To^2/(q_Lo*S);           % drag at V_Lo [N]
        T_excess = T_A_Lo - D_Lo;                                     % excess thrust [N]

        if T_excess <= 0
            % Insufficient excess thrust to sustain climb at V_Lo
            d_To_map(i,j)    = Inf;
            d_Lo_map(i,j)    = d_Lo;
            d_climb_map(i,j) = Inf;
            T_A_map(i,j)     = T_A;
            continue
        end

        % ---- CLIMB DISTANCE TO CLEAR 50 FT OBSTACLE ----
        % Rate of climb from excess-thrust method at V_Lo:
        %   ROC = V_Lo * T_excess / W_To
        % Horizontal distance to climb h_obs at constant V_Lo:
        %   d_climb = h_obs * V_Lo / ROC = h_obs * W_To / T_excess
        ROC     = V_Lo * T_excess / W_To;   % rate of climb [m/s]
        d_climb = h_obs * V_Lo / ROC;       % obstacle clearance distance [m]

        % ---- TOTAL TAKEOFF FIELD LENGTH ----
        d_To_map(i,j)    = d_Lo + d_climb;
        d_Lo_map(i,j)    = d_Lo;
        d_climb_map(i,j) = d_climb;
        T_A_map(i,j)     = T_A;   % store for diagnostics

    end
end

%% ============================================================
%%  SECTION 4 — HEATMAP PLOT
%% ============================================================

% Cap Inf values for colour scaling — cells above d_cap are infeasible
d_cap  = 600;
d_plot = min(d_To_map, d_cap);

[S_grid, AR_grid] = meshgrid(S_range, AR_range);

figure('Name', 'Takeoff Distance Heatmap', 'Position', [100 100 900 650]);

% Filled colour field — use contourf to match LandingDist.m style
contourf(S_grid, AR_grid, d_plot, 30, 'LineColor', 'none');
colormap(jet);
cb = colorbar;
cb.Label.String = 'Takeoff Distance  d_{TO}  [m]';
cb.FontSize = 11;
clim([50 200]);   % fix colour axis: 0 m (blue) to 200 m (red); cells above 200 m saturate red
hold on;

% Labelled contour lines for readability
[C_all, h_all] = contour(S_grid, AR_grid, d_To_map, 10, ...
    'LineColor', [0.15 0.15 0.15], 'LineWidth', 0.7);
clabel(C_all, h_all, 'FontSize', 8, 'Color', [0.1 0.1 0.1]);

% 500 ft (152.4 m) requirement boundary — primary design constraint
[C_req, h_req] = contour(S_grid, AR_grid, d_To_map, ...
    [d_To_req d_To_req], 'r-', 'LineWidth', 2.5);
clabel(C_req, h_req, 'FontSize', 10, 'Color', 'r', ...
    'FontWeight', 'bold', 'LabelSpacing', 300);

% 10% design margin contour in white dashed
[C_marg, h_marg] = contour(S_grid, AR_grid, d_To_map, ...
    [d_To_marg d_To_marg], 'w--', 'LineWidth', 1.8);
clabel(C_marg, h_marg, 'FontSize', 9, 'Color', 'w', 'LabelSpacing', 300);

xlabel('Wing Reference Area,  S  [m^2]', 'FontSize', 12);
ylabel('Aspect Ratio,  AR',              'FontSize', 12);
title({sprintf('Takeoff Distance [m]  |  W_{TO} = %.1f kN,  P_A = %.0f kW  (%.0f SHP × \\eta_{prop}=%.2f)', ...
        W_To/1e3, P_A/1e3, SHP, eta_prop), ...
       sprintf('CL_{max,TO} = %.3f  (3.2×0.80),  CD0_{TO} = %.4f  (clean %.4f + flap %.4f),  \\mu_r = %.2f', ...
        CL_max_TO, CD0_takeoff, CD0_clean, CD0_flaps, mu_r)}, ...
    'FontSize', 10);

h_r = plot(NaN, NaN, 'r-',  'LineWidth', 2.5);
h_m = plot(NaN, NaN, 'w--', 'LineWidth', 1.8);
legend([h_r, h_m], ...
    {sprintf('500 ft limit  (%.1f m)', d_To_req), ...
     sprintf('10%% margin target  (%.0f m)', d_To_marg)}, ...
    'Location', 'northeast', 'FontSize', 10);

annotation('textbox', [0.12 0.01 0.78 0.035], ...
    'String', ['NOTE: W_{TO}, SHP, CD0, CL_{max} are preliminary estimates. ' ...
               'Re-run after each weight loop iteration and engine selection.'], ...
    'FontSize', 8, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'Color', [0.35 0.35 0.35]);

hold off;

%% ============================================================
%%  SECTION 5 — PRINTED SUMMARY
%% ============================================================

fprintf('=== Takeoff Distance Results ===\n')
fprintf('  500 ft requirement    : <= %.1f m\n',  d_To_req)
fprintf('  10%% margin target     : <= %.1f m\n', d_To_marg)

% Filter out Inf before computing min
feasible_vals = d_To_map(isfinite(d_To_map));
if ~isempty(feasible_vals)
    fprintf('  Min distance (sweep)  : %.1f m  (%.0f ft)\n', ...
        min(feasible_vals), min(feasible_vals)/0.3048)
    fprintf('  Max finite distance   : %.1f m  (%.0f ft)\n', ...
        max(feasible_vals), max(feasible_vals)/0.3048)
    n_pass = nnz(d_To_map(:) <= d_To_req);
    n_tot  = numel(d_To_map);
    fprintf('  Feasible cells        : %d / %d  (%.0f%%)\n\n', ...
        n_pass, n_tot, 100*n_pass/n_tot)
else
    fprintf('  WARNING: No feasible cells found. Check SHP and W_To.\n\n')
end

% Mid-range sample point — component breakdown printed to command window
AR_s = AR_range(round(end/2));
S_s  = S_range(round(end/2));
b_s  = sqrt(AR_s * S_s);
k_s  = 1/(pi*AR_s*e);
phi_s = (16*h_wing/b_s)^2 / ((16*h_wing/b_s)^2 + 1);
Vs_s  = sqrt(2*W_To/(rho*S_s*CL_max_TO));
VLo_s = 1.2*Vs_s;
Vav_s = 0.7*VLo_s;
TA_s  = P_A/Vav_s;
qav_s = 0.5*rho*Vav_s^2;
qLo_s = 0.5*rho*VLo_s^2;
La_s  = qav_s*S_s*CL_max_TO;
Dg_s  = qav_s*S_s*CD0_takeoff + phi_s*k_s*W_To^2/(qav_s*S_s);
Fn_s  = TA_s - Dg_s - mu_r*(W_To - La_s);
TALo_s = P_A/VLo_s;
DLo_s  = qLo_s*S_s*CD0_takeoff + k_s*W_To^2/(qLo_s*S_s);
Tex_s  = TALo_s - DLo_s;

fprintf('  --- Sample point: AR = %.1f, S = %.1f m^2 ---\n', AR_s, S_s)
fprintf('  b             = %.2f m\n',      b_s)
fprintf('  phi (GE)      = %.4f\n',        phi_s)
fprintf('  V_stall       = %.2f m/s  (%.1f kts)\n', Vs_s,  Vs_s/0.5144)
fprintf('  V_Lo          = %.2f m/s  (%.1f kts)\n', VLo_s, VLo_s/0.5144)
fprintf('  V_avg         = %.2f m/s  (%.1f kts)\n', Vav_s, Vav_s/0.5144)
fprintf('  T_A (V_avg)   = %.1f N  (%.2f kN)\n',   TA_s,  TA_s/1e3)
fprintf('  T_A (V_Lo)    = %.1f N  (%.2f kN)\n',   TALo_s,TALo_s/1e3)
fprintf('  F_net (avg)   = %.1f N\n', Fn_s)
if Fn_s > 0
    dLo_s   = 1.44*W_To^2/(rho*g*S_s*CL_max_TO*Fn_s);
    if Tex_s > 0
        ROC_s   = VLo_s*Tex_s/W_To;
        dCl_s   = h_obs*VLo_s/ROC_s;
        fprintf('  d_Lo (ground) = %.1f m  (%.0f ft)\n', dLo_s, dLo_s/0.3048)
        fprintf('  ROC at V_Lo   = %.2f m/s  (%.0f ft/min)\n', ROC_s, ROC_s/0.00508)
        fprintf('  d_climb       = %.1f m  (%.0f ft)\n', dCl_s, dCl_s/0.3048)
        fprintf('  d_To TOTAL    = %.1f m  (%.0f ft)\n', dLo_s+dCl_s, (dLo_s+dCl_s)/0.3048)
        if dLo_s+dCl_s <= d_To_req
            fprintf('  STATUS: PASS  (%.1f m below limit)\n', d_To_req-(dLo_s+dCl_s))
        else
            fprintf('  STATUS: FAIL  (%.1f m over limit)\n', (dLo_s+dCl_s)-d_To_req)
        end
    else
        fprintf('  STATUS: INFEASIBLE — insufficient climb thrust at V_Lo\n')
    end
else
    fprintf('  STATUS: INFEASIBLE — insufficient net force for ground roll\n')
end
