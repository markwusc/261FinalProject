% DragBuildUp.m
% =========================================================================
% DRAG BUILD-UP — Parasite Drag Coefficient (CD_0) Estimation
%
% Method follows the equivalent parasite drag area approach from Appendix A:
%   D_o = (A_fuse + A_wing + A_htail + A_vtail + A_aux) * q_inf
%   CD_0 = sum(A_i) / S_ref
%
% Each component contributes an equivalent parasite drag area A_i [m^2]:
%   Lifting surfaces (wing, tails):   A_i = 2 * Cf * S_planform
%   Fuselage (prolate ellipsoid):     A_i = (1 + 1.5*(l/d)^-1.5 + 7*(l/d)^-3) * Cf * S_wet
%
% Airfoil:  NASA GA(W)-1  (General Aviation Whitcomb)
%   — Relatively thick low-speed airfoil (t/c ≈ 0.17), so a
%     thickness correction (1 + 2*(t/c)) is applied to lifting surfaces.
%   — Ref: Raymer, Aircraft Design §12.5 / Hoerner Ch. 6
%
% Skin friction model: turbulent flat-plate (conservative; STOL aircraft
%   at low-Re are transitional but turbulent BL is the safer assumption).
%     Cf = 0.074 / Re^0.2    [Prandtl turbulent flat-plate]
%
% Reynolds number uses the characteristic length of each component:
%   Wing / tail: mean aerodynamic chord (MAC)
%   Fuselage:    total length l
%
% OUTPUTS (printed to console)
%   Component-by-component breakdown table
%   Total CD_0 (clean configuration)
%   Comparison against the placeholder used in ConstraintAnalysis_TO_Land_v2.m
%
% USAGE
%   1. Fill in every value in SECTION 1 (geometry from your CAD model).
%   2. Run.  No additional .m files required.
%   3. Feed the resulting CD_0_clean into ConstraintAnalysis_TO_Land_v2.m.
%
% NOTES
%   * All quantities in SI (metres, Newtons, Watts, kg) unless noted.
%   * A_aux is an additive lump for protuberances (antennas, fairings, gaps).
%     A reasonable first estimate is 5–10 % of (A_fuse + A_wing + A_htail + A_vtail).
%     Set aux_frac below accordingly and revise after detail design.
%   * Compressibility drag is NOT included here (low-speed aircraft).
%   * Interference drag is NOT explicitly modelled; the aux factor partially
%     accounts for it.  Wing-fuselage junction fairings can reduce it by ~30 %.
% =========================================================================

clear; clc; close all;

% =========================================================================
%  SECTION 1 — AIRCRAFT GEOMETRY  (← fill these from CAD)
% =========================================================================

% ── Flight condition for Cf evaluation ────────────────────────────────────
% Use cruise (clean) condition.  Cf is insensitive to small V changes.
V_cruise = 130 * 0.5144;   % cruise true airspeed  130 KTAS → [m/s]
rho_SL   = 1.225;           % sea-level ISA density              [kg/m^3]
mu_air   = 1.789e-5;        % dynamic viscosity of air at SL     [Pa·s]
% NOTE: Cf evaluated at SL (conservative — lower altitude → higher Re → lower Cf,
%       but Cf is a weak function of Re, so SL is a safe starting point).

% ─────────────────────────────────────────────────────────────────────────
%  FUSELAGE  (modelled as a prolate ellipsoid — Appendix A)
% ─────────────────────────────────────────────────────────────────────────
%  Pull l and d from your CAD bounding box / outer-mould-line.
%  l = total fuselage length (nose to tail)
%  d = maximum fuselage diameter (or equivalent circular diameter)

l_fuse = 10.00;   % fuselage length            [m]   ← REPLACE
d_fuse =  1.50;   % fuselage max diameter      [m]   ← REPLACE

% ─────────────────────────────────────────────────────────────────────────
%  WING
% ─────────────────────────────────────────────────────────────────────────
%  S_wing  = exposed planform area (one side × 2 for both surfaces,
%            but the "2" is already baked into the formula below)
%  MAC     = mean aerodynamic chord  (∫c^2 dy / S_half)
%  t_c_w   = thickness-to-chord ratio of NASA GA(W)-1

S_wing  = 30.00;   % total wing planform area   [m^2]  ← REPLACE
MAC_w   =  1.80;   % mean aerodynamic chord      [m]   ← REPLACE
t_c_w   =  0.17;   % GA(W)-1 t/c ≈ 0.17          [-]   (verify against your section)

% ─────────────────────────────────────────────────────────────────────────
%  HORIZONTAL TAIL (stabiliser)
% ─────────────────────────────────────────────────────────────────────────
S_htail = 5.00;   % total h-tail planform area  [m^2]  ← REPLACE
MAC_ht  = 0.90;   % h-tail mean aero chord       [m]   ← REPLACE
t_c_ht  = 0.12;   % h-tail airfoil t/c           [-]   ← REPLACE (symmetric section)

% ─────────────────────────────────────────────────────────────────────────
%  VERTICAL TAIL (fin)
% ─────────────────────────────────────────────────────────────────────────
S_vtail = 3.50;   % v-tail planform area         [m^2]  ← REPLACE
MAC_vt  = 1.00;   % v-tail mean aero chord        [m]   ← REPLACE
t_c_vt  = 0.12;   % v-tail airfoil t/c            [-]   ← REPLACE

% ─────────────────────────────────────────────────────────────────────────
%  AUXILIARY PROTUBERANCES  (antennas, fairings, landing-gear sponsons, etc.)
% ─────────────────────────────────────────────────────────────────────────
%  aux_frac: fraction of the four primary component drag areas to add as a
%            lump for all auxiliary sources.  Typical range: 0.05 – 0.15.
%            Use 0.10 (10 %) as a first estimate; refine after detail design.
aux_frac = 0.10;   % auxiliary lump fraction   [-]

% ─────────────────────────────────────────────────────────────────────────
%  REFERENCE AREA  (for CD_0 normalisation)
% ─────────────────────────────────────────────────────────────────────────
%  By convention CD_0 is referenced to the wing planform area S_wing.
S_ref = S_wing;   % [m^2]

% =========================================================================
%  SECTION 2 — PHYSICAL CONSTANTS  (do not edit)
% =========================================================================

% Turbulent flat-plate skin friction exponent (Prandtl formula)
% Cf = 0.074 / Re^(1/5)
Cf_coeff = 0.074;
Cf_exp   = 0.200;   % 1/5

% GA(W)-1 thickness correction for lifting surfaces:
%   CD_0,surface ≈ 2 * Cf * (1 + 2*(t/c)) * S_planform   [Raymer §12.5]
% The factor (1 + 2*(t/c)) accounts for the pressure drag of a thick airfoil
% above the flat-plate baseline.  For t/c = 0.17 this adds ~34 % to Cf.

% =========================================================================
%  SECTION 3 — REYNOLDS NUMBER AND SKIN FRICTION COEFFICIENT
% =========================================================================

% ── Reynolds number for each component ───────────────────────────────────
Re_fuse = rho_SL * V_cruise * l_fuse  / mu_air;   % fuselage Re  (based on l)
Re_wing = rho_SL * V_cruise * MAC_w   / mu_air;   % wing Re      (based on MAC)
Re_ht   = rho_SL * V_cruise * MAC_ht  / mu_air;   % h-tail Re
Re_vt   = rho_SL * V_cruise * MAC_vt  / mu_air;   % v-tail Re

% ── Turbulent flat-plate Cf for each component ───────────────────────────
Cf_fuse = Cf_coeff / Re_fuse^Cf_exp;
Cf_wing = Cf_coeff / Re_wing^Cf_exp;
Cf_ht   = Cf_coeff / Re_ht^Cf_exp;
Cf_vt   = Cf_coeff / Re_vt^Cf_exp;

% =========================================================================
%  SECTION 4 — FUSELAGE PARASITE DRAG
% =========================================================================
%  Appendix A prolate-ellipsoid formula (wetted-area form):
%    CD_0,fuse = [1 + 1.5*(l/d)^-1.5 + 7*(l/d)^-3] * Cf
%    S_wet,fuse = (pi*d/2) * (d + l)                     [exact ellipsoid]
%    A_fuse = CD_0,fuse * S_wet,fuse

FR = l_fuse / d_fuse;   % fineness ratio l/d  [-]

% Form factor (Hoerner / Appendix A Eq. 1)
FF_fuse = 1 + 1.5 * FR^(-1.5) + 7 * FR^(-3);

% Wetted area of prolate ellipsoid
S_wet_fuse = (pi * d_fuse / 2) * (d_fuse + l_fuse);   % [m^2]

% Equivalent parasite drag area
A_fuse = FF_fuse * Cf_fuse * S_wet_fuse;   % [m^2]

% Component CD_0 referenced to S_ref (for reporting)
CD0_fuse = A_fuse / S_ref;

% =========================================================================
%  SECTION 5 — WING PARASITE DRAG
% =========================================================================
%  Both upper and lower surfaces contribute → factor of 2 in front of Cf.
%  Thickness correction for GA(W)-1 thick section included.
%    A_wing = 2 * Cf * (1 + 2*(t/c)) * S_wing

FF_wing = 1 + 2 * t_c_w;   % airfoil form factor  [-]

A_wing = 2 * Cf_wing * FF_wing * S_wing;   % [m^2]

CD0_wing = A_wing / S_ref;

% =========================================================================
%  SECTION 6 — HORIZONTAL TAIL PARASITE DRAG
% =========================================================================

FF_ht = 1 + 2 * t_c_ht;   % form factor for h-tail section  [-]

A_htail = 2 * Cf_ht * FF_ht * S_htail;   % [m^2]

CD0_htail = A_htail / S_ref;

% =========================================================================
%  SECTION 7 — VERTICAL TAIL PARASITE DRAG
% =========================================================================

FF_vt = 1 + 2 * t_c_vt;   % form factor for v-tail section  [-]

A_vtail = 2 * Cf_vt * FF_vt * S_vtail;   % [m^2]

CD0_vtail = A_vtail / S_ref;

% =========================================================================
%  SECTION 8 — AUXILIARY DRAG AREA LUMP
% =========================================================================
%  A_aux is a fraction of the four primary areas to account for
%  protuberances, interference drag, and detail items not explicitly modelled.

A_primary = A_fuse + A_wing + A_htail + A_vtail;

A_aux    = aux_frac * A_primary;   % [m^2]
CD0_aux  = A_aux / S_ref;

% =========================================================================
%  SECTION 9 — TOTAL PARASITE DRAG COEFFICIENT (CLEAN)
% =========================================================================

A_total   = A_primary + A_aux;    % total equivalent parasite drag area [m^2]
CD0_clean = A_total / S_ref;      % referenced to S_ref = S_wing         [-]

% =========================================================================
%  SECTION 10 — CONSOLE OUTPUT
% =========================================================================

fprintf('\n');
fprintf('==========================================================================\n');
fprintf('  DRAG BUILD-UP SUMMARY\n');
fprintf('==========================================================================\n');
fprintf('\n');

% ── Flight condition ──────────────────────────────────────────────────────
fprintf('  Flight condition (Cf evaluation)\n');
fprintf('    V_cruise   = %6.2f m/s  (%5.1f KTAS)\n', V_cruise, V_cruise/0.5144);
fprintf('    rho        = %6.4f kg/m^3  (ISA sea level)\n', rho_SL);
fprintf('\n');

% ── Reynolds numbers ──────────────────────────────────────────────────────
fprintf('  Reynolds Numbers\n');
fprintf('    Fuselage   (L = %.2f m)    Re = %.3e\n', l_fuse,  Re_fuse);
fprintf('    Wing       (MAC = %.2f m)  Re = %.3e\n', MAC_w,   Re_wing);
fprintf('    H-Tail     (MAC = %.2f m)  Re = %.3e\n', MAC_ht,  Re_ht);
fprintf('    V-Tail     (MAC = %.2f m)  Re = %.3e\n', MAC_vt,  Re_vt);
fprintf('\n');

% ── Skin friction coefficients ────────────────────────────────────────────
fprintf('  Skin Friction Coefficients  [turbulent flat-plate, Cf = 0.074/Re^0.2]\n');
fprintf('    Fuselage   Cf = %.5f\n', Cf_fuse);
fprintf('    Wing       Cf = %.5f\n', Cf_wing);
fprintf('    H-Tail     Cf = %.5f\n', Cf_ht);
fprintf('    V-Tail     Cf = %.5f\n', Cf_vt);
fprintf('\n');

% ── Component drag breakdown table ───────────────────────────────────────
fprintf('  Component Drag Breakdown\n');
fprintf('  %-14s  %8s  %8s  %8s  %10s  %8s\n', ...
        'Component', 'Area[m^2]', 'Cf', 'FF', 'A_i [m^2]', 'CD0_i');
fprintf('  %s\n', repmat('-', 1, 64));

fprintf('  %-14s  %8.2f  %8.5f  %8.4f  %10.5f  %8.5f\n', ...
        'Fuselage', S_wet_fuse, Cf_fuse, FF_fuse, A_fuse, CD0_fuse);
fprintf('  %-14s  %8.2f  %8.5f  %8.4f  %10.5f  %8.5f\n', ...
        'Wing', S_wing, Cf_wing, FF_wing, A_wing, CD0_wing);
fprintf('  %-14s  %8.2f  %8.5f  %8.4f  %10.5f  %8.5f\n', ...
        'H-Tail', S_htail, Cf_ht, FF_ht, A_htail, CD0_htail);
fprintf('  %-14s  %8.2f  %8.5f  %8.4f  %10.5f  %8.5f\n', ...
        'V-Tail', S_vtail, Cf_vt, FF_vt, A_vtail, CD0_vtail);
fprintf('  %-14s  %8s  %8s  %8s  %10.5f  %8.5f\n', ...
        sprintf('Aux (%d%%)', round(aux_frac*100)), '—', '—', '—', A_aux, CD0_aux);
fprintf('  %s\n', repmat('-', 1, 64));
fprintf('  %-14s  %8s  %8s  %8s  %10.5f  %8.5f\n', ...
        'TOTAL (clean)', '—', '—', '—', A_total, CD0_clean);
fprintf('\n');

% ── Comparison against constraint analysis placeholder ────────────────────
CD0_placeholder = 0.030;   % value used in ConstraintAnalysis_TO_Land_v2.m
delta_pct = (CD0_clean - CD0_placeholder) / CD0_placeholder * 100;

fprintf('  Comparison against ConstraintAnalysis placeholder\n');
fprintf('    Placeholder CD_0_clean = %.4f\n', CD0_placeholder);
fprintf('    Build-up    CD_0_clean = %.4f\n', CD0_clean);
fprintf('    Difference             = %+.1f %%\n', delta_pct);
if abs(delta_pct) > 20
    fprintf('    *** WARNING: >20%% deviation — revisit geometry inputs ***\n');
end
fprintf('\n');
fprintf('==========================================================================\n');
fprintf('  To propagate: set CD_0_clean = %.5f in ConstraintAnalysis script.\n', CD0_clean);
fprintf('==========================================================================\n\n');

% =========================================================================
%  SECTION 11 — BAR CHART: COMPONENT CONTRIBUTION TO CD_0
% =========================================================================

components  = {'Fuselage', 'Wing', 'H-Tail', 'V-Tail', ...
               sprintf('Aux (%d%%)', round(aux_frac*100))};
CD0_parts   = [CD0_fuse, CD0_wing, CD0_htail, CD0_vtail, CD0_aux];
pct_parts   = 100 * CD0_parts / CD0_clean;

figure('Name','Drag Build-Up — CD_0 Breakdown', ...
       'Units','normalized', 'Position',[0.10 0.15 0.55 0.65]);

bar_colors = [0.27 0.51 0.71;   % fuselage  — steel blue
              0.20 0.63 0.17;   % wing      — green
              0.89 0.47 0.13;   % h-tail    — orange
              0.75 0.22 0.17;   % v-tail    — red
              0.60 0.60 0.60];  % aux       — grey

b = bar(CD0_parts, 'FaceColor','flat');
b.CData = bar_colors;

% Annotate each bar with its percentage contribution
for k = 1:length(CD0_parts)
    text(k, CD0_parts(k) + 0.0002, ...
         sprintf('%.1f %%', pct_parts(k)), ...
         'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold');
end

% Reference line at total CD_0
yline(CD0_clean, 'k--', 'LineWidth', 1.4, ...
      'Label', sprintf('CD_0 = %.4f (total)', CD0_clean), ...
      'LabelHorizontalAlignment','left', 'FontSize', 9);

xticks(1:length(components));
xticklabels(components);
ylabel('Parasite Drag Coefficient  CD_0  [-]', 'FontSize', 12);
title({ 'Drag Build-Up — Component Contributions to CD_{0,clean}', ...
        sprintf('Airfoil: NASA GA(W)-1  |  Turbulent Cf  |  S_{ref} = %.1f m^2', S_ref) }, ...
      'FontSize', 11);
ax = gca;
ax.FontSize  = 11;
ax.GridAlpha = 0.30;
grid on;
ylim([0, max(CD0_parts) * 1.35]);
xlim([0.4, length(components) + 0.6]);
box on;