% DragBuildUp.m
% =========================================================================
% DRAG BUILD-UP — Parasite Drag Coefficient (CD_0) Estimation
%
% Method: Raymer §12.5 "Component Buildup Method" (Eq. 12.24):
%
%   (CD_0)_subsonic = Σ(Cf_c · FF_c · Q_c · Swet_c) / Sref
%                     + CD_misc + CD_L&P
%
%   where for each component (subscript c):
%     Cf  = flat-plate skin friction coefficient  (Raymer Eq. 12.27)
%     FF  = form factor (pressure drag correction) (Raymer Eqs. 12.30–12.31)
%     Q   = interference factor                   (Raymer §12.5, p.285)
%     Swet= component wetted area
%
%   CD_misc captures landing-gear drag.
%   CD_L&P  captures leakage and protuberance drag.
%
% Airfoil: NASA GA(W)-1  (General Aviation Whitcomb, also known as NASA LS(1)-0417)
%   — t/c = 0.17, (x/c)_m ≈ 0.40 (max thickness near 40% chord, typical of this
%     series). Ref: McGhee & Beasley, NASA TN D-7428 (1973).
%   — Unswept, low-speed design → sweep term in Eq. (12.30) collapses to ~1.0.
%
% Fuselage cross-section: 1.75 m × 2.00 m rectangle with 0.10 m corner fillets.
%   — Equivalent circular diameter computed from cross-sectional area
%     using Raymer Eq. 12.33: d = sqrt(4/pi * A_max).
%   — Rectangular fuselage form factor correction from Raymer p.284:
%     "A square-sided fuselage has a form factor about 40% higher than the
%     [ellipsoidal] value estimated with Eq.(12.31) due to additional separation
%     caused by the corners. This can be somewhat reduced by rounding the corners."
%     With 0.10 m fillets, rounding ratio r/side ≈ 0.053 (modest). A 20%
%     FF penalty is applied — midpoint between sharp-cornered (+40%) and
%     perfectly circular (0%). Engineering judgement per Raymer p.284.
%
% Tail airfoils: NACA 0012 selected for both horizontal and vertical tail.
%   — NACA 0012: t/c = 0.12, (x/c)_m = 0.30 (standard 4-digit NACA: max
%     thickness at 30% chord). Classic choice for low-speed tail surfaces.
%   — Ref: Abbott & von Doenhoff, "Theory of Wing Sections," Dover (1959).
%   — Tail form factors increased 10% for hinge gaps per Raymer p.283.
%
% Landing gear: Fixed tricycle gear (2 main + 1 nose).
%   — Drag estimated using Raymer Table 12.5 D/q values per component.
%   — Total D/q multiplied by 1.2 for mutual interference (Raymer p.287).
%
% Leakage & Protuberances:
%   — 5% of primary component drag (Raymer p.289, propeller-aircraft upper bound).
%
% Skin friction: Raymer Eq. 12.27 (turbulent, Mach-corrected) throughout.
%
% KEY REFERENCE
%   Raymer, D. P., "Aircraft Design: A Conceptual Approach," AIAA, 1989.
%   All equation and page numbers refer to this edition.
%
% USAGE
%   1. Section 2 contains all geometry — replace placeholder values with CAD outputs.
%   2. Run. No additional .m files needed.
%   3. Feed resulting CD_0_clean into ConstraintAnalysis_TO_Land_v2.m.
%
% All quantities SI (metres, kg, Pa, m^2) unless stated.
% =========================================================================

clear; clc; close all;

ft2_to_m2 = 0.0929;   % unit conversion factor: 1 ft^2 = 0.0929 m^2

% =========================================================================
%  SECTION 1 — FLIGHT CONDITION FOR Cf EVALUATION
% =========================================================================
% Cf evaluated at cruise (clean config). Raymer Eq.(12.27) is Mach-corrected
% so M_cruise is included even though the correction is negligible at low speed.

V_cruise  = 130 * 0.5144;      % 130 KTAS → m/s
M_cruise  = V_cruise / 340.3;  % Mach number (SL ISA speed of sound = 340.3 m/s)
rho_SL    = 1.225;              % ISA sea-level density              [kg/m^3]
mu_air    = 1.789e-5;           % dynamic viscosity air, SL 15 °C   [Pa·s]

% =========================================================================
%  SECTION 2 — AIRCRAFT GEOMETRY
% =========================================================================

% ─────────────────────────────────────────────────────────────────────────
%  WING
% ─────────────────────────────────────────────────────────────────────────
S_wing  = 36.80;            % total wing planform area              [m^2]
b_wing  = 15.20;            % wing span                              [m]
MAC_w   = S_wing / b_wing;  % MAC — untapered → MAC = S/b           [m]

% NASA GA(W)-1 airfoil (Raymer Eq. 12.30 parameters)
t_c_w      = 0.17;    % thickness-to-chord ratio                   [-]
xcm_w      = 0.40;    % chordwise location of max thickness (x/c)_m  [-]
%   McGhee & Beasley, NASA TN D-7428 (1973): GA(W)-1 max t at ~40% chord.
Lambda_m_w = 0.0;     % sweep of max-thickness line  [rad]  (unswept)

% ─────────────────────────────────────────────────────────────────────────
%  HORIZONTAL TAIL
% ─────────────────────────────────────────────────────────────────────────
S_htail   = 11.34;              % total planform area                [m^2]
b_htail   =  6.73;              % span                                [m]
MAC_ht    = S_htail / b_htail;  % MAC — untapered                    [m]

% NACA 0012 horizontal tail (Abbott & von Doenhoff, 1959)
t_c_ht    = 0.12;     % t/c                                         [-]
xcm_ht    = 0.30;     % (x/c)_m — standard 4-digit NACA at 30% chord  [-]
Lambda_m_ht = 0.0;    % unswept                                    [rad]

% ─────────────────────────────────────────────────────────────────────────
%  VERTICAL TAIL
% ─────────────────────────────────────────────────────────────────────────
S_vtail   = 4.22;               % planform area                      [m^2]
b_vtail   = 2.60;               % fin height                          [m]
MAC_vt    = S_vtail / b_vtail;  % MAC — untapered                    [m]

% NACA 0012 vertical tail — same section as h-tail
t_c_vt    = 0.12;
xcm_vt    = 0.30;
Lambda_m_vt = 0.0;

% ─────────────────────────────────────────────────────────────────────────
%  FUSELAGE
% ─────────────────────────────────────────────────────────────────────────
l_fuse   = 9.9001;    % total fuselage length                       [m]
W_fuse   = 1.75;      % fuselage width                              [m]
H_fuse   = 2.00;      % fuselage height                             [m]
r_fillet = 0.10;      % corner fillet radius                        [m]

% ── Cross-sectional area (filleted rectangle) ─────────────────────────────
% Area = rectangle − 4 corner squares + 4 quarter-circle fillets
A_rect            = W_fuse * H_fuse;
A_corner_removed  = 4 * (r_fillet^2 - pi * r_fillet^2 / 4);
A_max_fuse        = A_rect - A_corner_removed;

% ── Equivalent circular diameter (Raymer Eq. 12.33) ──────────────────────
% Raymer defines d for a non-circular cross-section as the diameter of a
% circle with the same area as the maximum cross-section.
%   f = l/d = l / sqrt(4/pi * A_max)
d_fuse = sqrt(4 / pi * A_max_fuse);   % equivalent circular diameter  [m]

% ── Fineness ratio ────────────────────────────────────────────────────────
f_fuse = l_fuse / d_fuse;   % used in Eq.(12.31)                    [-]

% ── Wetted area of fuselage ───────────────────────────────────────────────
% Perimeter of the filleted rectangle × fuselage length.
% Perimeter = full rectangle perimeter − 4 straight corner segments
%             + 4 quarter-circle arcs
P_rect_corners = 4 * r_fillet;
P_fillet_arcs  = 4 * (pi / 2 * r_fillet);
P_wet_fuse     = 2*(W_fuse + H_fuse) - P_rect_corners + P_fillet_arcs;
S_wet_fuse     = P_wet_fuse * l_fuse;                                % [m^2]

% ── Rectangular cross-section FF penalty (Raymer p.284) ──────────────────
% Raymer p.284: sharp square → +40%; rounded corners reduce this.
% 0.10 m fillets on ~1.875 m sides → r/side ≈ 0.053 → modest rounding.
% 20% penalty applied (conservative midpoint between sharp and fully rounded).
rect_FF_penalty = 1.20;

% ─────────────────────────────────────────────────────────────────────────
%  LANDING GEAR — Fixed Tricycle  (Raymer Table 12.5 & p.287)
% ─────────────────────────────────────────────────────────────────────────
% Component D/q values from Raymer Table 12.5 [ft^2 per component unit]:
%   Regular wheel and tire : 0.25 ft^2
%   Round strut or wire    : 0.30 ft^2
% Tricycle layout: 2 main legs + 1 nose leg, each with 1 wheel and 1 strut.

n_legs   = 3;        % total gear legs (2 main + 1 nose)
Dq_wheel = 0.25;     % D/q per wheel+tire,  ft^2  (Table 12.5)
Dq_strut = 0.30;     % D/q per round strut, ft^2  (Table 12.5)

% Raw sum before interference correction
Dq_gear_raw = n_legs * (Dq_wheel + Dq_strut);   % [ft^2]

% Raymer p.287: multiply total gear drag by 1.2 for mutual interference
Dq_gear     = 1.2 * Dq_gear_raw;                % [ft^2]
Dq_gear_m2  = Dq_gear * ft2_to_m2;              % [m^2]

% ─────────────────────────────────────────────────────────────────────────
%  REFERENCE AREA
% ─────────────────────────────────────────────────────────────────────────
S_ref = S_wing;   % conventional normalisation (Raymer §12.5)        [m^2]

% =========================================================================
%  SECTION 3 — SKIN FRICTION COEFFICIENTS  (Raymer Eq. 12.27)
% =========================================================================
% Turbulent flat-plate, Mach-number corrected (Raymer p.282, Eq. 12.27):
%   Cf = 0.455 / [(log10 Re)^2.58 × (1 + 0.144 M^2)^0.65]
%
% Characteristic length: fuselage → l_fuse; lifting surfaces → MAC.

Re_fuse = rho_SL * V_cruise * l_fuse  / mu_air;
Re_wing = rho_SL * V_cruise * MAC_w   / mu_air;
Re_ht   = rho_SL * V_cruise * MAC_ht  / mu_air;
Re_vt   = rho_SL * V_cruise * MAC_vt  / mu_air;

% Anonymous function: Raymer Eq. (12.27)
turb_Cf = @(Re, M) 0.455 / ((log10(Re))^2.58 * (1 + 0.144*M^2)^0.65);

Cf_fuse = turb_Cf(Re_fuse, M_cruise);
Cf_wing = turb_Cf(Re_wing, M_cruise);
Cf_ht   = turb_Cf(Re_ht,   M_cruise);
Cf_vt   = turb_Cf(Re_vt,   M_cruise);

% =========================================================================
%  SECTION 4 — FORM FACTORS
% =========================================================================

% ── Wing and tail surfaces: Raymer Eq. (12.30) ────────────────────────────
% FF = [1 + 0.6/(x/c)_m × (t/c) + 100(t/c)^4] × [1.34 M^0.18 (cosΛ_m)^0.28]
%
% At M_cruise ≈ 0.195 and Λ_m = 0° (unswept), the second bracket ≈ 0.990 ≈ 1.0.
% The first bracket captures the airfoil-shape contribution to pressure drag.
%
% Tail +10% for hinge gap: Raymer p.283 — "A tail surface with a hinged
% rudder or elevator will have a form factor about 10% higher than predicted
% by Eq.(12.30) due to the extra drag of the gap between the tail surface
% and its control surface."

FF_wing = (1 + (0.6/xcm_w) * t_c_w  + 100*t_c_w^4) ...
        * (1.34 * M_cruise^0.18 * cos(Lambda_m_w)^0.28);

FF_ht   = (1 + (0.6/xcm_ht) * t_c_ht + 100*t_c_ht^4) ...
        * (1.34 * M_cruise^0.18 * cos(Lambda_m_ht)^0.28) ...
        * 1.10;   % +10% hinge-gap penalty (Raymer p.283)

FF_vt   = (1 + (0.6/xcm_vt) * t_c_vt + 100*t_c_vt^4) ...
        * (1.34 * M_cruise^0.18 * cos(Lambda_m_vt)^0.28) ...
        * 1.10;   % +10% hinge-gap penalty (Raymer p.283)

% ── Fuselage: Raymer Eq. (12.31), with rectangular FF penalty ─────────────
% FF_ellipsoid = 1 + 60/f^3 + f/400    (smooth, axisymmetric body)
% FF_fuse = FF_ellipsoid × rect_FF_penalty  (Raymer p.284, see Section 2)

FF_fuse = (1 + 60/f_fuse^3 + f_fuse/400) * rect_FF_penalty;

% =========================================================================
%  SECTION 5 — INTERFERENCE FACTORS  Q  (Raymer §12.5, p.285)
% =========================================================================
% Q multiplies each component's friction + form drag to account for
% aerodynamic interference at component junctions.
%
% Fuselage  Q = 1.00 — Raymer p.285: "The fuselage has a negligible
%   interference factor (Q = 1.0) in most cases."
%
% Wing Q = 1.00 — Raymer p.285: "For a high-wing, a mid-wing, or a
%   well-filletted low wing, the interference will be negligible so the
%   Q factor will be about 1.0."
%
% H-tail Q = 1.05 — Raymer p.285: "For tail surfaces, interference ranges
%   from about three percent (Q = 1.03) for a clean V-tail to about eight
%   percent for an H-tail." Midpoint chosen (5%) as conservative for a
%   conventional H-tail on a fuselage-mounted stabiliser.
%
% V-tail Q = 1.03 — Low end of range (clean fin-fuselage junction).

Q_fuse  = 1.00;
Q_wing  = 1.00;
Q_htail = 1.05;
Q_vtail = 1.03;

% =========================================================================
%  SECTION 6 — WETTED AREAS FOR LIFTING SURFACES
% =========================================================================
% Both upper and lower surfaces exposed → factor of 2.
% Thickness correction: S_wet ≈ 2 × S_plan × (1 + 0.2 × t/c)
% This accounts for the increase in wetted area of a thick wing compared
% to a flat plate. For t/c = 0.17 this is a 3.4% correction. Conservative
% and consistent with common conceptual-design practice.

S_wet_wing  = 2 * S_wing  * (1 + 0.2 * t_c_w);
S_wet_htail = 2 * S_htail * (1 + 0.2 * t_c_ht);
S_wet_vtail = 2 * S_vtail * (1 + 0.2 * t_c_vt);

% =========================================================================
%  SECTION 7 — COMPONENT DRAG AREAS  (Cf × FF × Q × Swet)
% =========================================================================

A_fuse  = Cf_fuse * FF_fuse  * Q_fuse  * S_wet_fuse;
A_wing  = Cf_wing * FF_wing  * Q_wing  * S_wet_wing;
A_htail = Cf_ht   * FF_ht    * Q_htail * S_wet_htail;
A_vtail = Cf_vt   * FF_vt    * Q_vtail * S_wet_vtail;

% =========================================================================
%  SECTION 8 — MISCELLANEOUS DRAG: LANDING GEAR  CD_misc
% =========================================================================
% Fixed landing gear is the dominant misc. drag source for this aircraft.
% D/q computed in Section 2 from Raymer Table 12.5 and converted to CD.

CD_misc_gear = Dq_gear_m2 / S_ref;

% =========================================================================
%  SECTION 9 — LEAKAGE & PROTUBERANCE DRAG  CD_L&P  (Raymer p.289)
% =========================================================================
% Raymer p.289: "For a normal production aircraft, leaks and protuberance
% drags can be estimated as about 2–5% of the parasite drag for jet
% transports or bombers, 5–10% for propeller aircraft."
%
% NOTE: Raymer quotes 5–10% for propeller aircraft. 5% is used here as the
% lower end of the propeller-aircraft range — appropriate for an aircraft
% that is not a fighter and will have careful attention to surface finish.
% This is applied to the primary friction/form drag sum only (gear is already
% explicitly accounted for in CD_misc).

leak_frac  = 0.05;

A_primary  = A_fuse + A_wing + A_htail + A_vtail;
CD_primary = A_primary / S_ref;
CD_LP      = leak_frac * CD_primary;

% =========================================================================
%  SECTION 10 — TOTAL PARASITE DRAG  (Raymer Eq. 12.24)
% =========================================================================

CD0_fuse  = A_fuse  / S_ref;
CD0_wing  = A_wing  / S_ref;
CD0_htail = A_htail / S_ref;
CD0_vtail = A_vtail / S_ref;

% Raymer Eq. (12.24):
% (CD_0)_subsonic = Σ(Cf·FF·Q·Swet)/Sref + CD_misc + CD_L&P
CD0_clean = CD_primary + CD_misc_gear + CD_LP;

% =========================================================================
%  SECTION 11 — CONSOLE OUTPUT
% =========================================================================

fprintf('\n');
fprintf('==========================================================================\n');
fprintf('  DRAG BUILD-UP SUMMARY  (Raymer §12.5 Component Buildup, Eq. 12.24)\n');
fprintf('==========================================================================\n\n');

fprintf('  Derived Geometry\n');
fprintf('    Wing  MAC    = %.4f m   S_wet = %.4f m^2\n', MAC_w,  S_wet_wing);
fprintf('    H-tail MAC   = %.4f m   S_wet = %.4f m^2\n', MAC_ht, S_wet_htail);
fprintf('    V-tail MAC   = %.4f m   S_wet = %.4f m^2\n', MAC_vt, S_wet_vtail);
fprintf('    Fuse  A_max  = %.4f m^2  (%.2f×%.2f rect, r=%.2f m fillets)\n', ...
        A_max_fuse, W_fuse, H_fuse, r_fillet);
fprintf('    Fuse  d_eq   = %.4f m   (Raymer Eq.12.33)\n', d_fuse);
fprintf('    Fuse  l/d    = %.4f    (fineness ratio, Eq.12.31)\n', f_fuse);
fprintf('    Fuse  S_wet  = %.4f m^2  (perimeter × length)\n\n', S_wet_fuse);

fprintf('  Flight Condition\n');
fprintf('    V = %.2f m/s (%.0f KTAS)   M = %.4f   rho = %.4f kg/m^3\n\n', ...
        V_cruise, V_cruise/0.5144, M_cruise, rho_SL);

fprintf('  Reynolds Numbers  (Raymer Eq.12.26: Re = rho·V·l/mu)\n');
fprintf('    Fuselage  (L=%.4f m)  Re = %.4e\n', l_fuse, Re_fuse);
fprintf('    Wing      (L=%.4f m)  Re = %.4e\n', MAC_w,  Re_wing);
fprintf('    H-Tail    (L=%.4f m)  Re = %.4e\n', MAC_ht, Re_ht);
fprintf('    V-Tail    (L=%.4f m)  Re = %.4e\n\n', MAC_vt, Re_vt);

fprintf('  Skin Friction Coeff.  (Raymer Eq.12.27, turbulent)\n');
fprintf('    Fuselage  Cf = %.5f\n', Cf_fuse);
fprintf('    Wing      Cf = %.5f\n', Cf_wing);
fprintf('    H-Tail    Cf = %.5f\n', Cf_ht);
fprintf('    V-Tail    Cf = %.5f\n\n', Cf_vt);

fprintf('  Form Factors\n');
fprintf('    Fuselage  FF = %.4f  (Eq.12.31 base=%.4f × %.2f rect penalty, Raymer p.284)\n', ...
        FF_fuse, FF_fuse/rect_FF_penalty, rect_FF_penalty);
fprintf('    Wing      FF = %.4f  (Eq.12.30: GA(W)-1, t/c=%.2f, (x/c)m=%.2f, Λ=0°)\n', ...
        FF_wing, t_c_w, xcm_w);
fprintf('    H-Tail    FF = %.4f  (Eq.12.30 × 1.10 hinge-gap, Raymer p.283)\n', FF_ht);
fprintf('    V-Tail    FF = %.4f  (Eq.12.30 × 1.10 hinge-gap, Raymer p.283)\n\n', FF_vt);

fprintf('  Interference Factors Q  (Raymer p.285)\n');
fprintf('    Fuselage Q=%.2f  Wing Q=%.2f  H-tail Q=%.2f  V-tail Q=%.2f\n\n', ...
        Q_fuse, Q_wing, Q_htail, Q_vtail);

% Component breakdown table
fprintf('  Component Drag Breakdown\n');
hdr = '  %-15s  %9s  %7s  %7s  %6s  %10s  %8s  %7s';
row = '  %-15s  %9.4f  %7.5f  %7.4f  %6.2f  %10.6f  %8.5f  %6.1f%%';
row_misc = '  %-15s  %9s  %7s  %7s  %6s  %10s  %8.5f  %6.1f%%';
fprintf([hdr '\n'], 'Component','Swet[m^2]','Cf','FF','Q','Ai[m^2]','CD0_i','%Total');
fprintf('  %s\n', repmat('-',1,82));
fprintf([row '\n'], 'Fuselage',  S_wet_fuse,   Cf_fuse, FF_fuse,  Q_fuse,  A_fuse,  CD0_fuse,  100*CD0_fuse/CD0_clean);
fprintf([row '\n'], 'Wing',      S_wet_wing,   Cf_wing, FF_wing,  Q_wing,  A_wing,  CD0_wing,  100*CD0_wing/CD0_clean);
fprintf([row '\n'], 'H-Tail',    S_wet_htail,  Cf_ht,   FF_ht,    Q_htail, A_htail, CD0_htail, 100*CD0_htail/CD0_clean);
fprintf([row '\n'], 'V-Tail',    S_wet_vtail,  Cf_vt,   FF_vt,    Q_vtail, A_vtail, CD0_vtail, 100*CD0_vtail/CD0_clean);
fprintf([row_misc '\n'], 'Gear (misc)','—','—','—','—','—', CD_misc_gear, 100*CD_misc_gear/CD0_clean);
fprintf([row_misc '\n'], 'Leak & Protub','—','—','—','—','—', CD_LP, 100*CD_LP/CD0_clean);
fprintf('  %s\n', repmat('-',1,82));
fprintf([row_misc '\n'], 'TOTAL (clean)','—','—','—','—','—', CD0_clean, 100.0);
fprintf('\n');

% Landing gear detail
fprintf('  Landing Gear Detail  (Raymer Table 12.5)\n');
fprintf('    %d legs × wheel(%.2f ft²) + strut(%.2f ft²) = %.3f ft²  (raw)\n', ...
        n_legs, Dq_wheel, Dq_strut, Dq_gear_raw);
fprintf('    × 1.20 interference (Raymer p.287) → D/q = %.3f ft² = %.5f m²\n', ...
        Dq_gear, Dq_gear_m2);
fprintf('    CD_misc_gear = %.5f\n\n', CD_misc_gear);

% Comparison
CD0_placeholder = 0.030;
delta_pct = (CD0_clean - CD0_placeholder) / CD0_placeholder * 100;
fprintf('  Comparison vs. ConstraintAnalysis placeholder\n');
fprintf('    Placeholder   CD_0 = %.4f\n', CD0_placeholder);
fprintf('    Build-up      CD_0 = %.4f\n', CD0_clean);
fprintf('    Difference         = %+.1f%%\n', delta_pct);
if abs(delta_pct) > 20
    fprintf('    *** WARNING: >20%% deviation — update ConstraintAnalysis! ***\n');
end
fprintf('\n');
fprintf('==========================================================================\n');
fprintf('  ACTION: Set CD_0_clean = %.5f in ConstraintAnalysis_TO_Land_v2.m\n', CD0_clean);
fprintf('==========================================================================\n\n');

% =========================================================================
%  SECTION 12 — BAR CHART
% =========================================================================

comp_labels = {'Fuselage', 'Wing', 'H-Tail', 'V-Tail', 'Gear', 'Leak & Protub'};
CD0_parts   = [CD0_fuse, CD0_wing, CD0_htail, CD0_vtail, CD_misc_gear, CD_LP];
pct_parts   = 100 * CD0_parts / CD0_clean;

bar_colors  = [0.27 0.51 0.71;   % fuselage  — steel blue
               0.20 0.63 0.17;   % wing      — green
               0.89 0.47 0.13;   % h-tail    — orange
               0.75 0.22 0.17;   % v-tail    — red
               0.55 0.27 0.07;   % gear      — brown
               0.60 0.60 0.60];  % leakage   — grey

figure('Name','Drag Build-Up', 'Units','normalized', 'Position',[0.08 0.15 0.60 0.65]);

b = bar(CD0_parts, 'FaceColor','flat');
b.CData = bar_colors;

for k = 1:length(CD0_parts)
    text(k, CD0_parts(k) + 0.0002, sprintf('%.1f%%', pct_parts(k)), ...
         'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold');
end

yline(CD0_clean, 'k--', 'LineWidth',1.5, ...
      'Label', sprintf('CD_0 = %.4f (total clean)', CD0_clean), ...
      'LabelHorizontalAlignment','left', 'FontSize',9);

xticks(1:length(comp_labels));
xticklabels(comp_labels);
ylabel('Parasite Drag Coefficient  CD_0  [-]', 'FontSize',12);
title({ ...
    'Drag Build-Up — Component Contributions to CD_{0,clean}', ...
    sprintf('Raymer §12.5 | NASA GA(W)-1 | NACA 0012 tails | Fixed Tricycle Gear | S_{ref} = %.2f m^2', S_ref)}, ...
    'FontSize',11);

ax = gca;
ax.FontSize  = 11;
ax.GridAlpha = 0.30;
grid on;
ylim([0, max(CD0_parts)*1.40]);
xlim([0.4, length(comp_labels)+0.6]);
box on;
