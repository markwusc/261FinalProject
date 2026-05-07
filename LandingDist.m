%% LandingDist.m
%{
Landing distance trade study for Arctic STOL aircraft.
Generates a contour heatmap of total landing distance as a function of
wing area (S) and aspect ratio (AR).

Landing distance is broken into two phases:
    d_landing = d_air + d_ground

    d_air    : Analytical air-phase distance (obstacle to touchdown).
               Combines 50 ft obstacle clearance and flare geometry,
               scaled by the approach L/D ratio (E = CL_max / CD).
               NOTE: High L/D (efficient wing) produces a LONGER d_air
               because the glide slope is flatter. This is the fundamental
               STOL tension — aerodynamic efficiency works against short-field
               landing performance.

    d_ground : Ground roll from V_TD = 1.3*Vstall to rest.
               Retarding forces: wheel brakes + aerodynamic drag + reverse thrust.
               Aero forces and T_rev evaluated at average speed 0.7 * V_TD.
               T_rev uses the same shaft-power method as TakeoffDist:
                   T_A  = P_A / (0.7 * V_TD)    [forward thrust at avg speed]
                   T_rev = eta_rev * T_A          [reverse thrust, beta-pitch]
               T_A and T_rev therefore change at every (AR, S) cell as
               V_TD varies with S through V_stall.

CD0 is split into two terms — one per flight phase:
    CD0_approach   = CD0_clean + CD0_flaps          [used for d_air only]
    CD0_groundroll = CD0_approach + CD0_spoilers    [used for d_ground only]
    CD0_flaps      = 0.0023 * flapSpanFraction * flapDeflection_deg
    CD0_spoilers   = Cd_spoiler_normal * (S_spoiler/S_ref) * sin(theta_spoiler)

Method references:
    Anderson, "Introduction to Flight", Landing Performance
    Raymer,   "Aircraft Design: A Conceptual Approach", Ch. 17
    Raymer,   Ch. 13 (propulsive reversal, turboprop reverse-pitch efficiency)

ITERATION NOTE:
    W_To, SHP, CD0_clean, and CL_max are estimates at this stage.
    Re-run every weight loop iteration as these values converge.
%}

clear; clc;

%% ============================================================
%%  SECTION 1 — USER INPUTS  (update every design iteration)
%% ============================================================

% --- Takeoff weight and fuel accounting ---
% W_To is the primary driver of V_stall and all force terms.
W_To          = 40000;   % Max takeoff gross weight [N]
                          % Update each iteration as weight estimate refines
fuel_fraction = 0.132;   % W_fuel / W_To  (total usable fuel as fraction of MTOW)
                          % Updated from Breguet range calculation each iteration
W_fuel_reserve = 500;     % Weight of fuel retained at landing [N]
                          %   Represents trapped/unusable fuel in lines and tanks
                          %   Not a fraction — set as an absolute weight so it
                          %   stays physically fixed regardless of fuel_fraction
                          %   Increase to model IFR divert reserve (heavier landing)

% --- Powerplant ---
% Same engine as TakeoffDist — SHP does not change between missions.
% T_A at landing is lower than at takeoff because V_TD > V_avg_TO,
% i.e. the turboprop produces less thrust at the higher landing speed.
SHP      = 1050;   % Total shaft horsepower, BOTH engines [hp]
                    % e.g. 2x PT6A-27 (~620 SHP each) — update after engine sel.
eta_prop = 0.80;   % Propeller efficiency during ground roll [--]
                    % Conservative value for partial-reverse beta range

% Convert SHP to available propulsive power [W]
P_shaft = SHP * 745.7;          % shaft power [W]  (1 hp = 745.7 W exactly)
P_A     = eta_prop * P_shaft;   % available propulsive power [W]
% NOTE: T_A = P_A / (0.7*V_TD) is computed INSIDE the sweep loop because
% V_TD changes with S at every cell (via V_stall → V_TD = 1.3*V_stall).

% --- Thrust reversal (PT6A-series turboprop, reverse-pitch propeller) ---
% PT6A turboprops achieve reverse thrust by driving blades into the
% beta / reverse-pitch range. No cascade hardware required.
% eta_rev = 0.60 per Raymer, "Aircraft Design: A Conceptual Approach",
% Ch. 13 (propulsion reversal analysis for turboprop/propeller installations).
% This is higher than the 0.45 used in the old script; 0.60 is Raymer's
% specific value for reverse-pitch propellers versus 0.45 for jet thrust
% reversers, making it the more physically appropriate source for this design.
eta_rev = 0.60;   % Reverse thrust effectiveness, PT6A beta-pitch [--]
% T_rev = eta_rev * T_A  is computed inside the sweep loop (T_A varies per cell)

% --- Drag: clean configuration ---
CD0_clean = 0.027;   % Parasite drag coefficient in clean (cruise) config [--]
                      % Refined from drag buildup (see Appendix A)

% --- Drag: flap contribution (landing — full deflection) ---
% Empirical formula for flap parasite drag increment:
%   CD0_flaps = 0.0023 * (flapSpanFraction) * (flapDeflection_deg)
% Source: Raymer, "Aircraft Design: A Conceptual Approach", flap drag tables.
% Landing uses full flap deflection (50 deg) vs. takeoff partial (30 deg),
% producing a significantly larger CD0 increment. This is physically correct
% and intentional — high drag on approach shortens d_air by steepening the
% glide slope (reducing E = CL/CD), which is the desired STOL behaviour.
flapSpanFraction   = 0.80;   % Fraction of semi-span covered by flaps [--]
flapDeflection_deg = 50;     % Full landing flap deflection [deg]
                               % Higher than takeoff (30 deg) — full deployment
CD0_flaps    = 0.0023 * flapSpanFraction * flapDeflection_deg;   % [--]
CD0_approach = CD0_clean + CD0_flaps;   % Total parasite drag at approach [--]
                                         % Used for d_air only (spoilers not yet deployed)

% --- Drag: spoiler contribution (ground roll only) ---
% Spoilers deploy on touchdown to kill lift and maximise braking normal force.
% They are NOT active during the airborne approach phase (d_air uses CD0_approach).
% Formula (Raymer drag buildup, spoiler panel):
%   Delta_CD0 = (Cd,spoiler,normal x S_spoiler,total x sin(theta)) / S_ref
% Because S_spoiler,total = (S_spoiler/S_ref) x S_ref, S_ref cancels:
%   Delta_CD0 = Cd,spoiler,normal x (S_spoiler/S_ref) x sin(theta_spoiler)
% CD0_spoilers is therefore a geometric constant independent of the S sweep.
Cd_spoiler_normal = 1.28;    % Normal-force drag coeff of spoiler panel [--]
                               % Flat-plate perpendicular-to-flow value
S_spoiler_ratio   = 0.1;     % S_spoiler,total / S_ref  [--]  (10% of wing area)
theta_spoiler_deg = 60;      % Spoiler deflection angle at touchdown [deg]
theta_spoiler_rad = theta_spoiler_deg * pi/180;

CD0_spoilers   = Cd_spoiler_normal * S_spoiler_ratio * sin(theta_spoiler_rad);   % [--]
CD0_groundroll = CD0_approach + CD0_spoilers;   % Parasite drag during ground roll [--]
                                                  % = CD0_clean + CD0_flaps + CD0_spoilers

% --- Lift kill factor (ground roll) ---
% Spoiler deployment collapses the lift distribution, reducing effective CL
% during the ground roll. This increases gear normal force and therefore
% available braking friction. liftKillFactor = fraction of CL_max retained.
% 0.20 means only 20% of approach lift acts on the wing during ground roll.
% Consistent with CA_TWvsWS_To_Land.m spoiler assumption.
liftKillFactor = 0.20;   % Fraction of CL_max_land retained with spoilers [--]

% --- Lift: landing configuration ---
% CL_max_land uses an 80% margin on the airfoil/flap CLmax per RFP §4.3.
% Base value 3.4 reflects a high-lift STOL airfoil + full Fowler flap;
% update once airfoil selection is finalised.
CL_max_land = 3.4 * 0.80;   % Aircraft CL_max in landing config [--] = 2.72

% --- Aerodynamic efficiency ---
e = 0.9;   % Oswald efficiency factor [--] (per RFP §4.3, clean config)

% --- Ground roll braking ---
mu_brake = 0.5;   % Braking friction coefficient (gravel runway, per RFP) [--]
                   % Consider running mu_brake = 0.3 as a wet/icy gravel
                   % sensitivity check for winter operations

% --- Atmosphere ---
rho_sea = 1.225;   % Sea-level ISA air density [kg/m^3]
g       = 9.81;    % Gravitational acceleration [m/s^2]

% --- Heatmap sweep ranges ---
AR_range = linspace(6,  14, 80);   % Aspect ratio sweep [--]
S_range  = linspace(20, 65, 80);   % Wing reference area [m^2]

%% ============================================================
%%  SECTION 2 — DERIVED LANDING WEIGHT AND CONSOLE HEADER
%% ============================================================

W_fuel_total  = fuel_fraction * W_To;                   % Total usable fuel [N]
W_fuel_burned = W_fuel_total  - W_fuel_reserve;         % Fuel consumed on mission [N]
W_landing     = W_To          - W_fuel_burned;          % Aircraft landing weight [N]
% Expanded: W_landing = W_To*(1 - fuel_fraction) + W_fuel_reserve

d_land_req  = 152.4;           % 500 ft field-length requirement [m]
d_land_marg = d_land_req * 0.90;   % 10% design margin target (= 137.2 m) [m]

fprintf('=== LandingDist.m — Parameter Summary ===\n')
fprintf('  W_To             : %6.0f N  (%.0f kg)\n',   W_To, W_To/g)
fprintf('  fuel_fraction    : %.3f  →  W_fuel = %.0f N\n', fuel_fraction, W_fuel_total)
fprintf('  W_fuel_reserve   : %.0f N  (trapped/unusable fuel)\n', W_fuel_reserve)
fprintf('  W_fuel_burned    : %.0f N\n', W_fuel_burned)
fprintf('  LANDING WEIGHT   : %.0f N  (%.0f kg,  %.1f%% of MTOW)\n', ...
    W_landing, W_landing/g, 100*W_landing/W_To)
fprintf('  SHP (total)      : %6.0f hp  →  P_shaft = %.1f kW\n', SHP, P_shaft/1e3)
fprintf('  eta_prop         : %.2f  →  P_A = %.1f kW\n', eta_prop, P_A/1e3)
fprintf('  eta_rev          : %.2f  (Raymer Ch.13, reverse-pitch turboprop)\n', eta_rev)
fprintf('  CD0_clean        : %.4f\n', CD0_clean)
fprintf('  flapSpan%%        : %.0f%%\n', flapSpanFraction*100)
fprintf('  flapDeflect      : %.0f deg  (full landing flap)\n', flapDeflection_deg)
fprintf('  CD0_flaps        : %.4f  (= 0.0023 × %.2f × %.0f)\n', ...
    CD0_flaps, flapSpanFraction, flapDeflection_deg)
fprintf('  CD0_approach     : %.4f  (clean + flaps, air phase)\n', CD0_approach)
fprintf('  Cd_spoiler_norm  : %.2f\n', Cd_spoiler_normal)
fprintf('  S_spoiler/S_ref  : %.2f\n', S_spoiler_ratio)
fprintf('  theta_spoiler    : %.0f deg\n', theta_spoiler_deg)
fprintf('  CD0_spoilers     : %.4f  (= %.2f x %.2f x sin(%.0f deg))\n', ...
    CD0_spoilers, Cd_spoiler_normal, S_spoiler_ratio, theta_spoiler_deg)
fprintf('  CD0_groundroll   : %.4f  (clean + flaps + spoilers, ground roll)\n', CD0_groundroll)
fprintf('  liftKillFactor   : %.2f  (CL fraction retained with spoilers)\n', liftKillFactor)
fprintf('  CL_max_land      : %.4f  (= 3.4 × 0.80)\n', CL_max_land)
fprintf('  e                : %.2f\n', e)
fprintf('  mu_brake         : %.2f  (gravel runway, per RFP)\n', mu_brake)
fprintf('  d_req            : %.1f m  (500 ft)\n', d_land_req)
fprintf('  d_margin         : %.1f m  (10%% margin)\n\n', d_land_marg)

%% ============================================================
%%  SECTION 3 — LANDING DISTANCE SWEEP  (AR x S double loop)
%% ============================================================

d_landing_map = zeros(length(AR_range), length(S_range));
d_air_map     = zeros(length(AR_range), length(S_range));   % component breakout
d_ground_map  = zeros(length(AR_range), length(S_range));   % component breakout
T_A_map       = zeros(length(AR_range), length(S_range));   % available thrust map
T_rev_map     = zeros(length(AR_range), length(S_range));   % reverse thrust map

for i = 1:length(AR_range)
    for j = 1:length(S_range)

        AR = AR_range(i);
        S  = S_range(j);

        % Induced drag factor
        k = 1 / (pi * AR * e);   % [--]

        % ---- STALL AND APPROACH SPEEDS ----
        % Stall speed at landing weight, sea level, full-flap CL_max
        V_stall = sqrt(2*W_landing / (rho_sea*S*CL_max_land));   % [m/s]

        % Touchdown speed: 1.3 * Vstall (FAR 23 landing convention)
        V_TD = 1.3 * V_stall;   % [m/s]

        % Average ground roll speed — same 0.7 factor as TakeoffDist
        V_avg_gnd = 0.7 * V_TD;   % [m/s]

        % ---- AVAILABLE AND REVERSE THRUST (speed-dependent) ----
        % Mirroring TakeoffDist methodology: T = P_A / V at average speed.
        % Using V_avg_gnd = 0.7*V_TD as the representative ground roll speed.
        % T_A is HIGHER at lower V (turboprop physics), so it changes at every
        % cell as V_TD shifts with S. Larger S → lower V_stall → lower V_TD
        % → lower V_avg_gnd → higher T_A and T_rev.
        T_A   = P_A / V_avg_gnd;          % forward thrust at avg gnd speed [N]
        T_rev = eta_rev * T_A;             % effective reverse thrust [N]
                                            % (Raymer Ch.13: 0.60 for beta-pitch)

        % ---- DRAG POLAR (landing / approach configuration) ----
        % Total CD at CL_max_land — operating point on approach and ground roll
        CD = CD0_approach + k*(CL_max_land^2);   % [--]

        % Approach L/D at CL_max.
        % STOL trade-off: high E = flat glide = longer d_air.
        % Full landing flap (50 deg) deliberately increases CD0_approach to
        % reduce E and steepen the glide — shortening d_air at the cost of
        % higher approach drag. This is the correct STOL design intent.
        E = CL_max_land / CD;   % [--]

        % ---- AIR PHASE  (d_air) ----
        % '15' = 50 ft obstacle height (15.24 m, rounded per convention)
        % '0.133*Vstall^2/(2g)' captures the flare geometry contribution
        % Scaled by E: a high-L/D aircraft covers more ground in the air
        d_air = E * (15 + (0.133*(V_stall^2)/(2*g)));   % [m]

        % ---- GROUND ROLL  (d_ground) ----
        % Separate drag polar for ground roll: CD0_groundroll includes spoiler term.
        % Spoilers are deployed at touchdown — NOT during the airborne approach.
        % d_air used CD0_approach; d_ground uses CD0_groundroll.
        CD_gnd = CD0_groundroll + k*(CL_max_land^2);   % ground roll drag polar [--]

        % Evaluate aero forces at V_avg_gnd = 0.7*V_TD
        q_landing = 0.5 * rho_sea * (V_avg_gnd^2);   % dynamic pressure [Pa]

        % Lift during ground roll — multiplied by liftKillFactor (0.20) to
        % account for spoiler deployment collapsing the lift distribution.
        % Reduced lift → higher normal force on gear → more braking friction.
        L_landing = q_landing * S * CL_max_land * liftKillFactor;   % [N]

        % Aerodynamic drag during ground roll — uses CD0_groundroll (with spoilers)
        D_landing = q_landing * S * CD_gnd;   % [N]

        % Denominator guard: all three retarding forces must be positive.
        % If T_rev alone exceeds the other terms this is non-physical —
        % should not occur at realistic SHP values but worth checking.
        denom = D_landing + T_rev + mu_brake*(W_landing - L_landing);

        if denom <= 0
            % Cannot decelerate — guard against non-physical parameter combinations
            d_landing_map(i,j) = Inf;
            d_air_map(i,j)     = d_air;
            d_ground_map(i,j)  = Inf;
            T_A_map(i,j)       = T_A;
            T_rev_map(i,j)     = T_rev;
            continue
        end

        % Ground roll distance.
        % Factor 1.69 is the landing analogue of the 1.44 takeoff factor,
        % corrected for starting at V_TD rather than accelerating from rest.
        % T_rev is in the denominator: larger reverse thrust → shorter d_ground.
        d_ground = 1.69 * W_landing^2 / ...
            (rho_sea * g * S * CL_max_land * denom);   % [m]

        % ---- TOTAL LANDING DISTANCE ----
        d_landing_map(i,j) = d_air + d_ground;

        % Store components for command-window diagnostics
        d_air_map(i,j)    = d_air;
        d_ground_map(i,j) = d_ground;
        T_A_map(i,j)      = T_A;
        T_rev_map(i,j)    = T_rev;

    end
end

%% ============================================================
%%  SECTION 4 — HEATMAP PLOT
%% ============================================================

[S_grid, AR_grid] = meshgrid(S_range, AR_range);

figure('Name', 'Landing Distance Heatmap', 'Position', [100 100 900 650]);

% Filled colour field — matching TakeoffDist.m style
contourf(S_grid, AR_grid, d_landing_map, 30, 'LineColor', 'none');
colormap(jet);
cb = colorbar;
cb.Label.String = 'Landing Distance  d_{land}  [m]';
cb.FontSize = 11;
clim([50 200]);   % 50 m lower bound (no realistic landing is shorter);
                   % 200 m upper bound saturates infeasible/long cells red

hold on;

% Labelled contour lines for readability
[C_all, h_all] = contour(S_grid, AR_grid, d_landing_map, 10, ...
    'LineColor', [0.15 0.15 0.15], 'LineWidth', 0.7);
clabel(C_all, h_all, 'FontSize', 8, 'Color', [0.1 0.1 0.1]);

% 500 ft (152.4 m) requirement boundary — primary design constraint
[C_req, h_req] = contour(S_grid, AR_grid, d_landing_map, ...
    [d_land_req d_land_req], 'r-', 'LineWidth', 2.5);
clabel(C_req, h_req, 'FontSize', 10, 'Color', 'r', ...
    'FontWeight', 'bold', 'LabelSpacing', 300);

% 10% design margin contour in white dashed
[C_marg, h_marg] = contour(S_grid, AR_grid, d_landing_map, ...
    [d_land_marg d_land_marg], 'w--', 'LineWidth', 1.8);
clabel(C_marg, h_marg, 'FontSize', 9, 'Color', 'w', 'LabelSpacing', 300);

xlabel('Wing Reference Area,  S  [m^2]', 'FontSize', 12);
ylabel('Aspect Ratio,  AR',              'FontSize', 12);
title({sprintf('Landing Distance [m]  |  W_{land} = %.1f kN  (W_{TO} = %.1f kN),  P_A = %.0f kW  (%.0f SHP × \\eta_{prop}=%.2f)', ...
        W_landing/1e3, W_To/1e3, P_A/1e3, SHP, eta_prop), ...
       sprintf('CL_{max,land} = %.3f  (3.4×0.80),  CD0_{app} = %.4f  (clean %.4f + flap %.4f),  \\eta_{rev} = %.2f,  \\mu_{brake} = %.2f', ...
        CL_max_land, CD0_approach, CD0_clean, CD0_flaps, eta_rev, mu_brake)}, ...
    'FontSize', 10);

h_r = plot(NaN, NaN, 'r-',  'LineWidth', 2.5);
h_m = plot(NaN, NaN, 'w--', 'LineWidth', 1.8);
legend([h_r, h_m], ...
    {sprintf('500 ft limit  (%.1f m)', d_land_req), ...
     sprintf('10%% margin target  (%.0f m)', d_land_marg)}, ...
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

fprintf('=== Landing Distance Results ===\n')
fprintf('  500 ft requirement    : <= %.1f m\n', d_land_req)
fprintf('  10%% margin target     : <= %.1f m\n', d_land_marg)

finite_vals = d_landing_map(isfinite(d_landing_map));
if ~isempty(finite_vals)
    fprintf('  Min distance (sweep)  : %.1f m  (%.0f ft)\n', ...
        min(finite_vals), min(finite_vals)/0.3048)
    fprintf('  Max finite distance   : %.1f m  (%.0f ft)\n', ...
        max(finite_vals), max(finite_vals)/0.3048)
    n_pass = nnz(d_landing_map(:) <= d_land_req);
    n_tot  = numel(d_landing_map);
    fprintf('  Feasible cells        : %d / %d  (%.0f%%)\n\n', ...
        n_pass, n_tot, 100*n_pass/n_tot)
else
    fprintf('  WARNING: No feasible cells found. Check SHP and W_To.\n\n')
end

% Mid-range sample point — full component breakdown to command window
AR_s  = AR_range(round(end/2));
S_s   = S_range(round(end/2));
k_s   = 1/(pi*AR_s*e);
Vs_s  = sqrt(2*W_landing/(rho_sea*S_s*CL_max_land));
VTD_s = 1.3  * Vs_s;
Vag_s = 0.7  * VTD_s;
TA_s  = P_A  / Vag_s;
Tr_s  = eta_rev * TA_s;
% Air phase — uses CD0_approach (no spoilers airborne)
CD_s      = CD0_approach + k_s*CL_max_land^2;
E_s       = CL_max_land / CD_s;
da_s      = E_s*(15 + 0.133*Vs_s^2/(2*g));

% Ground roll — uses CD0_groundroll (spoilers) and liftKillFactor
CD_gnd_s  = CD0_groundroll + k_s*CL_max_land^2;
ql_s      = 0.5*rho_sea*Vag_s^2;
Ll_s      = ql_s*S_s*CL_max_land * liftKillFactor;   % lift killed by spoilers
Dl_s      = ql_s*S_s*CD_gnd_s;
dn_s      = Dl_s + Tr_s + mu_brake*(W_landing - Ll_s);
dg_s      = 1.69*W_landing^2 / (rho_sea*g*S_s*CL_max_land*dn_s);

fprintf('  --- Sample point: AR = %.1f, S = %.1f m^2 ---\n', AR_s, S_s)
fprintf('  V_stall       = %.2f m/s  (%.1f kts)\n', Vs_s,  Vs_s/0.5144)
fprintf('  V_TD          = %.2f m/s  (%.1f kts)\n', VTD_s, VTD_s/0.5144)
fprintf('  V_avg_gnd     = %.2f m/s  (%.1f kts)\n', Vag_s, Vag_s/0.5144)
fprintf('  T_A (V_avg)   = %.1f N  (%.2f kN)\n',   TA_s,  TA_s/1e3)
fprintf('  T_rev         = %.1f N  (%.2f kN)  (eta_rev=%.2f)\n', Tr_s, Tr_s/1e3, eta_rev)
fprintf('  --- Air phase (no spoilers) ---\n')
fprintf('  CD0_approach  = %.4f\n', CD0_approach)
fprintf('  CD (approach) = %.4f\n', CD_s)
fprintf('  L/D (appch)   = %.3f\n', E_s)
gamma_s = atan(1/E_s) * (180/pi);   % cot(gamma) = E  →  tan(gamma) = 1/E
fprintf('  Glide angle   = %.2f deg  (cot(gamma) = L/D = %.3f)\n', gamma_s, E_s)
fprintf('  d_air         = %.1f m  (%.0f ft)\n', da_s, da_s/0.3048)
fprintf('  --- Ground roll (spoilers + liftKill=%.2f) ---\n', liftKillFactor)
fprintf('  CD0_groundroll= %.4f  (appch + spoilers %.4f)\n', CD0_groundroll, CD0_spoilers)
fprintf('  CD (gnd roll) = %.4f\n', CD_gnd_s)
fprintf('  L_landing     = %.1f N  (%.0f%% of no-spoiler lift)\n', Ll_s, liftKillFactor*100)
fprintf('  D_landing     = %.1f N\n', Dl_s)
fprintf('  d_ground      = %.1f m  (%.0f ft)\n', dg_s, dg_s/0.3048)
fprintf('  d_landing     = %.1f m  (%.0f ft)\n', da_s+dg_s, (da_s+dg_s)/0.3048)
if da_s+dg_s <= d_land_req
    fprintf('  STATUS: PASS  (%.1f m below limit)\n', d_land_req-(da_s+dg_s))
else
    fprintf('  STATUS: FAIL  (%.1f m over limit)\n', (da_s+dg_s)-d_land_req)
end
