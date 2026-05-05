%% RaymerWeightEst_v3.m
% =========================================================================
% Arctic STOL Aircraft — Initial Takeoff Weight Estimation
% Method: Raymer statistical EWF (single & twin), iterated to convergence.
%
% KEY CHANGES FROM v2:
%   1. User-configurable wing geometry block at the top (S, AR).
%   2. Loiter fuel fraction computed from the EXACT Breguet endurance
%      equation (screenshot formula), not the simplified L/D shorthand.
%      Because this formula depends on W0 at the start of loiter (which
%      changes each iteration), ff_loiter is recomputed inside the Raymer
%      sizing loop — not precomputed as a fixed constant.
%   3. STOL EWF penalty of 1.04x applied to both Raymer configurations to
%      account for heavier STOL-specific structure (reinforced gear for
%      gravel, large flap system, de-icing provisions, structural margins).
%
% BREGUET ENDURANCE FORMULA (screenshot):
%   E = (η / SFC_p) · (CL^(3/2) / CD) · √(2ρS) · [W1^(-1/2) - W0^(-1/2)]
%
%   - W0 = weight at START of loiter (heavier)  [N]
%   - W1 = weight at END of loiter (lighter)    [N]
%   - W1 < W0, so [W1^(-1/2) - W0^(-1/2)] > 0  (endurance is positive)
%
%   NOTE ON g:  When W is in Newtons and SFC_p is in kg/(W·s) the
%   derivation (Anderson §6.6) introduces a factor 1/g in the denominator.
%   Some textbooks absorb this by expressing W in kg-force (1 kgf = 9.81 N),
%   making g disappear. This script uses SI throughout — W in Newtons —
%   so g = 9.81 m/s² appears explicitly in K_loiter below.
%
%   Rearranging for the fuel fraction ff = W1/W0:
%
%       W1^(-1/2) = E/K + W0^(-1/2)
%       W1        = (E/K + W0^(-1/2))^(-2)
%       ff        = W1/W0 = (E/K + W0^(-1/2))^(-2) / W0
%
%   where  K = (η / (g · SFC_p)) · (CL^(3/2)/CD) · √(2ρS)   [N^(1/2)]
%
%   Because K contains √S and ff contains W0 (which changes each Raymer
%   step), ff_loiter is recalculated at EVERY iteration of the sizing loop.
%
%   Best-endurance CL and CD are derived from the parabolic drag polar:
%       CD = CD0 + k·CL²   with k = 1/(π·AR·e)
%   Optimal point: d/dCL [CL^(3/2)/CD] = 0  →  CL_BE = √(3·CD0/k)
%                                               CD_BE = 4·CD0
%   This is the aircraft operating condition during loiter.
%
% Units: SI (kg, m, N, W, s) throughout except inside Raymer's formula
%        which requires W0 in POUNDS (conversion applied, clearly labelled).
%
% Authors: AME 261 Design Team
% Date:    April 2026
% Version: 3.0
% =========================================================================

clear; clc; close all;

%% =========================================================================
%  >>>  USER INPUTS — EDIT THIS BLOCK EACH DESIGN ITERATION  <<<
% =========================================================================
%  These are the geometry and aerodynamic parameters that are NOT yet
%  locked in. Update them as the design converges. All other constants
%  (payload, crew, mission, Raymer coefficients) are in Section 2 below.

% --- Wing geometry ---
S   = 44.9;     % Wing reference area          [m²]   ESTIMATE
                % Caravan: 26 m², Kodiak: 23 m², Twin Otter: 39 m²
                % Use ~30-40 m² for initial sizing of a 9-pax STOL turboprop

AR  = 7.2;     % Aspect ratio                  [-]    ESTIMATE
                % STOL aircraft typically AR = 8-12.
                % Higher AR: better L/D, worse STOL (larger span → more
                % induced drag penalty near ground, heavier wing structure).
                % Start at 10 — in the middle of the STOL-compatible range.

% --- Aerodynamics ---
CD0 = 0.030;    % Parasite drag coefficient, clean config  [-]  ESTIMATE
                % Typical range for clean turboprop: 0.025–0.035.
                % Will be refined with full drag buildup (fuselage + wetted
                % area calculation, April 21 deliverable).

e   = 0.90;     % Oswald efficiency factor                  [-]
                % Per RFP §4.3: use 0.9 clean, 0.8 with wing stores.

% =========================================================================
%  END OF USER INPUT BLOCK
% =========================================================================


%% =========================================================================
%  SECTION 1 — REFERENCE AIRCRAFT SURVEY
%  Purpose: ground-truth band for EWF. Raymer gives one number; real
%  aircraft give the range we must land inside.
%  Sources: manufacturer data sheets, Jane's All the World's Aircraft.
% =========================================================================

ref(1).name    = 'Cessna 208B Grand Caravan';
ref(1).pax     = 9;
ref(1).MTOW_kg = 3969;  ref(1).OEW_kg = 2145;  ref(1).engines = 1;
ref(1).notes   = 'Baseline single-engine turboprop; workhorse of Arctic/Alaska ops';

ref(2).name    = 'Quest/Daher Kodiak 100';
ref(2).pax     = 9;
ref(2).MTOW_kg = 3062;  ref(2).OEW_kg = 1934;  ref(2).engines = 1;
ref(2).notes   = 'Purpose-built STOL; most mission-relevant single-engine comparator';

ref(3).name    = 'Pilatus PC-12 NGX';
ref(3).pax     = 9;
ref(3).MTOW_kg = 4740;  ref(3).OEW_kg = 2812;  ref(3).engines = 1;
ref(3).notes   = 'Pressurized single turboprop; upper bound of single-engine band';

ref(4).name    = 'DHC-6 Twin Otter Series 400';
ref(4).pax     = 19;    % larger pax — retained as STOL twin structural ref
ref(4).MTOW_kg = 5670;  ref(4).OEW_kg = 3363;  ref(4).engines = 2;
ref(4).notes   = 'Classic Arctic STOL twin; oversized for mission but best twin ref';

ref(5).name    = 'Britten-Norman BN-2 Islander';
ref(5).pax     = 9;
ref(5).MTOW_kg = 2994;  ref(5).OEW_kg = 1816;  ref(5).engines = 2;
ref(5).notes   = 'Light twin STOL; lower bound for twin EWF';

ref(6).name    = 'Tecnam P2012 Traveller';
ref(6).pax     = 9;
ref(6).MTOW_kg = 3900;  ref(6).OEW_kg = 2400;  ref(6).engines = 2;
ref(6).notes   = 'Modern twin-piston 9-pax utility; piston EWF > turboprop';

fprintf('=== SECTION 1: REFERENCE AIRCRAFT SURVEY ===\n');
fprintf('%-35s | Eng | %9s | %8s | %5s\n', 'Aircraft', 'MTOW (kg)', 'OEW (kg)', 'EWF');
fprintf('%s\n', repmat('-', 1, 75));
for i = 1:length(ref)
    ref(i).EWF = ref(i).OEW_kg / ref(i).MTOW_kg;
    fprintf('%-35s |  %d  | %9.0f | %8.0f | %5.3f\n', ...
        ref(i).name, ref(i).engines, ref(i).MTOW_kg, ref(i).OEW_kg, ref(i).EWF);
end

% Separate single-engine (9-pax) and twin-engine reference bands
SE_mask = ([ref.engines] == 1) & ([ref.pax] == 9);
TE_mask = ([ref.engines] == 2);
SE_EWF  = [ref(SE_mask).EWF];
TE_EWF  = [ref(TE_mask).EWF];

fprintf('\nSingle-engine 9-pax EWF band:   %.3f – %.3f  (mean %.3f)\n', ...
    min(SE_EWF), max(SE_EWF), mean(SE_EWF));
fprintf('Twin-engine reference EWF band: %.3f – %.3f  (mean %.3f)\n\n', ...
    min(TE_EWF), max(TE_EWF), mean(TE_EWF));


%% =========================================================================
%  SECTION 2 — FIXED DESIGN PARAMETERS
%  Values that do not change between iterations (mission, propulsion,
%  payload). Update only if the RFP requirements change.
% =========================================================================

% --- Payload: passengers + baggage ONLY, crew NOT included ---
% RFP Config 1: 9 pax × (200 lb body + 50 lb baggage) = 2,250 lb = 1,021 kg
% Per design decision: use 1,100 kg as the maximum design payload.
% Crew is excluded from payload — it is part of OEW (see below).
W_payload_kg = 1399;              % [kg]

% --- Crew: 2 pilots, placed in OEW (not payload) ---
% Crew weight does NOT change from flight to flight — it is a fixed part of
% the operating configuration, not a mission-variable load.
% RFP: 2 pilots × (180 lb body + 20 lb baggage) = 400 lb total = 181.4 kg
W_crew_kg = 2 * 200 * 0.4536;      % [kg]  = 181.44 kg exactly

% --- Combined weight that drives the sizing denominator ---
% W0 = W_payload + W_crew + W_fuel + W_structural_empty
%    = W_fixed   +          Wf_frac*W0   +  EWF*W0
% → W0*(1 - EWF - Wf_frac) = W_fixed
W_fixed_kg = W_payload_kg + W_crew_kg;   % [kg]

% --- Propulsion (user-specified, consistent with PT6A turboprop class) ---
SFCp_imp = 0.548;            % specific fuel consumption     [lb/(hp·hr)]
eta_p    = 0.80;             % propeller efficiency          [-]

% Convert SFCp to SI: [kg/(W·s)]
%   1 lb/(hp·hr) × 0.4536 kg/lb ÷ (745.7 W/hp × 3600 s/hr)
SFCp_SI = SFCp_imp * 0.4536 / (745.7 * 3600);   % [kg/(W·s)]

% --- Mission parameters (from RFP Section 1.3) ---
R_nmi       = 450;              % minimum cruise range          [nmi]
R_m         = R_nmi * 1852;    % converted to metres           [m]
t_loiter_s  = 45 * 60;         % loiter reserve duration       [s]

% --- Atmospheric constants ---
g       = 9.81;     % gravitational acceleration    [m/s²]
rho_SL  = 1.225;    % sea-level ISA air density     [kg/m³]
                    % Loiter assumed at sea level — conservative (max fuel burn).
                    % Re-evaluate if loiter altitude is specified in mission.

% --- Cruise aerodynamics (baseline; swept in Section 6) ---
k = 1/(pi*e*AR);
E_m = sqrt(1/(4*CD0*k));
LD_cruise  = E_m;   % L/D at cruise condition      [-]  ESTIMATE
                     % Conservative for STOL wing; refined with drag buildup.

% --- STOL EWF penalty ---
% STOL aircraft carry structural weight not captured by Raymer's GA database:
%   - Large-chord double-slotted flap systems
%   - Reinforced gravel-rated landing gear (higher impact loads, gravel guards)
%   - De-icing provisions (FIKI required by RFP)
%   - Structural reinforcement at gravel-runway stress concentrations
% Empirical correction: +5% on Raymer EWF based on Kodiak vs generic GA data.
% Apply to both single- and twin-engine configs since both are STOL.
STOL_penalty = 1.05;    % multiplicative correction on EWF  [-]

% --- Fuel reserve ---
fuel_reserve = 0.03;     % 3% additional reserve over mission fuel  [-]

% --- Raymer convergence settings ---
tol_kg   = 0.5;      % convergence tolerance [kg]
max_iter = 300;      % maximum iterations
lb_per_kg = 2.20462; % unit conversion constant [lb/kg]

fprintf('=== SECTION 2: DESIGN INPUTS ===\n');
fprintf('  Wing area S          : %.1f m²\n', S);
fprintf('  Aspect ratio AR      : %.1f\n', AR);
fprintf('  CD0 (clean)          : %.4f\n', CD0);
fprintf('  Oswald e             : %.2f\n', e);
fprintf('  Payload (no crew)    : %.1f kg  (%.1f lb)\n', W_payload_kg, W_payload_kg/0.4536);
fprintf('  Crew (in OEW)        : %.2f kg  (%.1f lb)\n', W_crew_kg, W_crew_kg/0.4536);
fprintf('  Total fixed weight   : %.2f kg\n', W_fixed_kg);
fprintf('  SFCp                 : %.3f lb/(hp·hr)  →  %.4e kg/(W·s)\n', SFCp_imp, SFCp_SI);
fprintf('  eta_p                : %.2f\n', eta_p);
fprintf('  Cruise L/D (baseline): %.1f  (swept in Section 6)\n', LD_cruise);
fprintf('  STOL EWF penalty     : %.2fx\n', STOL_penalty);
fprintf('  Fuel reserve         : %.0f%%\n\n', fuel_reserve*100);


%% =========================================================================
%  SECTION 3 — LOITER AERODYNAMICS: BEST-ENDURANCE OPERATING POINT
%
%  For maximum endurance (propeller aircraft), we maximise CL^(3/2)/CD.
%  Using a parabolic drag polar  CD = CD0 + k·CL²  with  k = 1/(π·AR·e):
%
%    d/dCL [CL^(3/2)/CD] = 0
%    → (3/2)·CL^(1/2)·CD - CL^(3/2)·2k·CL = 0
%    → (3/2)·CD = 2k·CL²
%    → CL_BE = √(3·CD0/k)
%       CD_BE = CD0 + k·(3·CD0/k) = 4·CD0
%       (induced drag = 3× parasite drag at best endurance — NOT L/D_max point)
%
%  L/D_max occurs at CL = √(CD0/k) (induced = parasite), giving CD = 2·CD0.
%  Best endurance flies at a higher CL (lower speed) than L/D_max.
%
%  CL_BE is checked against CL_max here. If CL_BE > CL_max the wing stalls
%  before reaching best endurance — flag this and use CL_max instead.
% =========================================================================

k_ind   = 1 / (pi * AR * e);       % induced drag factor           [-]
CL_BE   = sqrt(3 * CD0 / k_ind);   % best-endurance lift coeff     [-]
CD_BE   = 4 * CD0;                  % best-endurance drag coeff     [-]
BE_param = CL_BE^1.5 / CD_BE;      % endurance parameter CL^1.5/CD [-]

% Warn if CL_BE exceeds a typical maximum lift coefficient
CL_max_typical = 2.4;   % typical STOL CL_max with flaps deployed
if CL_BE > CL_max_typical
    warning(['CL_BE = %.3f exceeds CL_max = %.3f.\n' ...
             'Loiter is constrained by stall, not by aerodynamic optimum.\n' ...
             'Replace CL_BE and CD_BE with values at actual CL_max.'], ...
             CL_BE, CL_max_typical);
    CL_BE    = CL_max_typical;
    CD_BE    = CD0 + k_ind * CL_BE^2;   % re-evaluate CD at CL_max
    BE_param = CL_BE^1.5 / CD_BE;
end

% K_loiter: the weight-and-geometry-independent part of the Breguet formula
% Units: K has units of √N (Newton^(1/2)) — see derivation note in header.
% When E [s] is divided by K [N^(1/2)] and added to W0^(-1/2) [N^(-1/2)],
% dimensions are consistent: W1 = (E/K + W0^(-1/2))^(-2) has units N. ✓
K_loiter = (eta_p / (g * SFCp_SI)) * BE_param * sqrt(2 * rho_SL * S);

fprintf('=== SECTION 3: LOITER AERODYNAMICS ===\n');
fprintf('  Induced drag factor k : %.5f\n', k_ind);
fprintf('  Best-endurance CL_BE  : %.4f\n', CL_BE);
fprintf('  Best-endurance CD_BE  : %.4f  (= 4×CD0 if not stall-limited)\n', CD_BE);
fprintf('  Endurance param (BE)  : CL^1.5/CD = %.4f\n', BE_param);
fprintf('  K_loiter              : %.4e  [N^(1/2)]\n\n', K_loiter);


%% =========================================================================
%  SECTION 4 — FIXED MISSION SEGMENT FUEL FRACTIONS
%  These segments use Raymer Table 3.2 historical data. They do NOT depend
%  on W0, so they are precomputed once before the iteration loop.
%  The cruise segment uses the Breguet range equation (recomputed in the
%  L/D sensitivity loop but fixed at baseline here).
%  The loiter segment is handled INSIDE the Raymer loop (Section 5).
% =========================================================================

%  Breguet range equation for propeller aircraft (SI form):
%    R = (η_p / (g × SFCp_SI)) × (L/D) × ln(W_i / W_{i+1})
%    → W_{i+1}/W_i = exp(−R × g × SFCp_SI / (η_p × L/D))
%
%  NOTE: some textbooks write R[statute miles] = (375×η_p/SFCp)×(L/D)×ln(...)
%  The factor 375 applies ONLY in statute miles. For nautical miles the
%  correct factor is ~325.9. The SI form avoids this ambiguity entirely.

ff_cruise  = exp(-R_m * g * SFCp_SI / (eta_p * LD_cruise));

ff_warmup  = 0.990;   % warm-up and taxi (10 min)
ff_takeoff = 0.995;   % STOL takeoff — slightly penalised vs FAR-25 standard
ff_climb   = 0.980;   % climb to cruise altitude
ff_descent = 0.990;   % descent to sea level
ff_landing = 0.992;   % landing rollout and taxi

% Product of all segments EXCEPT loiter — used to find W0_loiter inside loop
ff_pre_loiter = ff_warmup * ff_takeoff * ff_climb * ff_cruise * ff_descent;

fprintf('=== SECTION 4: FIXED FUEL FRACTIONS ===\n');
fprintf('  %-24s : %.4f\n', 'Warm-up & taxi', ff_warmup);
fprintf('  %-24s : %.4f\n', 'STOL Takeoff', ff_takeoff);
fprintf('  %-24s : %.4f\n', 'Climb', ff_climb);
fprintf('  %-24s : %.4f  (Breguet, R=%.0f nmi, L/D=%.1f)\n', ...
        'Cruise', ff_cruise, R_nmi, LD_cruise);
fprintf('  %-24s : %.4f\n', 'Descent', ff_descent);
fprintf('  %-24s : computed inside iteration loop\n', 'Loiter 45 min');
fprintf('  %-24s : %.4f\n\n', 'Landing & taxi', ff_landing);


%% =========================================================================
%  SECTION 5 — RAYMER ITERATIVE WEIGHT SIZING  (SINGLE & TWIN ENGINE)
%
%  Raymer formula (Table 3.1, "Aircraft Design: A Conceptual Approach"):
%    EWF_raw = A × W0_lbs^C
%    EWF     = EWF_raw × STOL_penalty     (applied after Raymer formula)
%
%  Coefficients:
%    Single-engine prop:  A = 2.36,  C = −0.18   (Raymer Table 3.1)
%    Twin-engine prop:    A = 1.51,  C = −0.10   (Raymer Table 3.1)
%
%  The weight equation (see derivation in header) is:
%    W0 = W_fixed / (1 − EWF − Wf_frac)
%
%  where W_fixed = W_payload + W_crew   (both non-fuel, fixed weights)
%  and   Wf_frac = total fuel weight / W0  (recomputed each iteration)
%
%  The loiter fuel fraction cannot be precomputed because ff_loiter depends
%  on W0_loiter = W0_guess × g × ff_pre_loiter  [N].  This changes with
%  each update to W0_guess, so the full Wf_frac is updated inside the loop.
%
%  Iteration procedure:
%    1. Guess W0 [kg]
%    2. Compute EWF from Raymer × STOL_penalty
%    3. Compute W0_loiter = W0_guess × g × ff_pre_loiter  [N]
%    4. Apply screenshot formula: W1_loiter = (t/K + W0_loiter^(-0.5))^(-2)
%    5. ff_loiter = W1_loiter / W0_loiter
%    6. Compute Wf_frac from all segment fractions (including ff_loiter)
%    7. Solve: W0_new = W_fixed / (1 - EWF - Wf_frac)
%    8. If |W0_new - W0_guess| < tol → converged. Else blend and repeat.
% =========================================================================

% Raymer coefficient sets
configs(1).label = 'Single-Engine Prop';
configs(1).A     = 2.36;
configs(1).C     = -0.18;
configs(1).W0_0  = 4000;   % initial guess [kg]

configs(2).label = 'Twin-Engine Prop';
configs(2).A     = 1.51;
configs(2).C     = -0.10;
configs(2).W0_0  = 5000;   % initial guess — twins are structurally heavier

fprintf('=== SECTION 5: RAYMER ITERATIVE WEIGHT SIZING ===\n');
fprintf('    (STOL EWF penalty = %.2fx applied to both configurations)\n\n', STOL_penalty);

for c = 1:length(configs)
    A = configs(c).A;
    C = configs(c).C;
    W0_guess = configs(c).W0_0;

    fprintf('--- %s  (A = %.2f, C = %.2f) ---\n', configs(c).label, A, C);
    fprintf('  Iter | W0_guess [kg] | EWF_raw | EWF+STOL | ff_loiter | W0_calc [kg] | Delta\n');
    fprintf('  %s\n', repmat('-', 1, 78));

    converged = false;
    ff_loiter_final = NaN;

    for iter = 1:max_iter

        % --- Step 1: Raymer EWF with STOL correction ---
        W0_lbs  = W0_guess * lb_per_kg;
        EWF_raw = A * (W0_lbs ^ C);
        EWF     = EWF_raw * STOL_penalty;

        % --- Step 2: Loiter fuel fraction (exact Breguet endurance formula) ---
        % Weight at start of loiter: apply preceding segment fractions to MTOW.
        % Convert from kg to N (Newtons) for the formula — W must be a force.
        W0_loiter_N = W0_guess * g * ff_pre_loiter;   % [N]

        % Solve for W1_loiter using rearranged screenshot formula:
        %   W1 = (E/K + W0^(-1/2))^(-2)
        W1_loiter_N = 1 / (t_loiter_s / K_loiter + W0_loiter_N^(-0.5))^2;

        % Fuel fraction for loiter segment
        ff_loiter = W1_loiter_N / W0_loiter_N;

        % --- Step 3: Total mission fuel fraction ---
        ff_total = ff_pre_loiter * ff_loiter * ff_landing;
        Mff      = 1 - ff_total;
        Wf_frac  = Mff * (1 + fuel_reserve);

        % --- Step 4: Feasibility check ---
        denom = 1 - EWF - Wf_frac;
        if denom <= 0
            % EWF + fuel fraction >= 1: structurally infeasible at this weight.
            % This means the aircraft cannot carry any payload — try heavier guess.
            fprintf('  INFEASIBLE at W0 = %.0f kg  (EWF + Wf = %.4f >= 1)\n', ...
                W0_guess, EWF + Wf_frac);
            W0_guess = W0_guess * 1.2;
            continue
        end

        % --- Step 5: Updated MTOW estimate ---
        W0_calc = W_fixed_kg / denom;
        delta   = abs(W0_calc - W0_guess);

        % Print first 10 iterations and final convergence row
        if iter <= 10 || delta < 2
            fprintf('  %4d | %12.1f | %7.4f | %8.4f | %9.6f | %12.1f | %5.1f\n', ...
                iter, W0_guess, EWF_raw, EWF, ff_loiter, W0_calc, delta);
        end

        if delta < tol_kg
            converged = true;
            ff_loiter_final = ff_loiter;
            break
        end

        % Blend old and new to prevent oscillation around the fixed point
        W0_guess = 0.5 * W0_guess + 0.5 * W0_calc;
    end

    % Store converged results
    configs(c).W0_kg         = W0_calc;
    configs(c).EWF_raw        = EWF_raw;
    configs(c).EWF            = EWF;           % includes STOL penalty
    configs(c).W_struct_kg    = EWF * W0_calc; % structural empty (no crew)
    configs(c).W_OEW_kg       = EWF * W0_calc + W_crew_kg;  % OEW = struct + crew
    configs(c).W_fuel_kg      = Wf_frac * W0_calc;
    configs(c).ff_loiter      = ff_loiter_final;
    configs(c).ff_total       = ff_total;
    configs(c).Wf_frac        = Wf_frac;
    configs(c).W_check_kg     = configs(c).W_OEW_kg + configs(c).W_fuel_kg + W_payload_kg;
    configs(c).converged      = converged;

    fprintf('\n  Converged: %s  |  Iterations: %d\n\n', mat2str(converged), iter);
end


%% =========================================================================
%  SECTION 5b — FINAL WEIGHT STATEMENTS
% =========================================================================

fprintf('=== SECTION 5b: FINAL WEIGHT STATEMENTS ===\n\n');

for c = 1:length(configs)
    W0 = configs(c).W0_kg;
    fprintf('┌─────────────────────────────────────────────────────────────────\n');
    fprintf('│  Configuration : %s\n', configs(c).label);
    fprintf('│  Raymer Coefficients : A = %.2f, C = %.2f  |  STOL penalty = %.2fx\n', ...
        configs(c).A, configs(c).C, STOL_penalty);
    fprintf('├─────────────────────────────────────────────────────────────────\n');
    fprintf('│  Max Takeoff Weight (W0)   : %7.1f kg  (%8.1f lb) (%8.1f N)\n', W0, W0*lb_per_kg, W0*g);
    fprintf('│\n');
    fprintf('│  OEW Breakdown:\n');
    fprintf('│    Structural empty weight : %7.1f kg  (%8.1f lb)  (EWF_raw=%.4f, EWF+STOL=%.4f)\n', ...
        configs(c).W_struct_kg, configs(c).W_struct_kg*lb_per_kg, ...
        configs(c).EWF_raw, configs(c).EWF);
    fprintf('│    Crew weight (in OEW)    : %7.1f kg  (%8.1f lb)  (2 pilots, RFP)\n', ...
        W_crew_kg, W_crew_kg*lb_per_kg);
    fprintf('│    OEW total (struct+crew) : %7.1f kg  (%8.1f lb)  (%.1f%% of W0)\n', ...
        configs(c).W_OEW_kg, configs(c).W_OEW_kg*lb_per_kg, ...
        100*configs(c).W_OEW_kg/W0);
    fprintf('│\n');
    fprintf('│  Fuel weight (incl 5%% res) : %7.1f kg  (%8.1f lb)  (%.1f%% of W0)\n', ...
        configs(c).W_fuel_kg, configs(c).W_fuel_kg*lb_per_kg, ...
        100*configs(c).W_fuel_kg/W0);
    fprintf('│    ff_cruise               : %.6f\n', ff_cruise);
    fprintf('│    ff_loiter (exact Breguet): %.6f  (W0_loiter = %.1f N)\n', ...
        configs(c).ff_loiter, configs(c).W0_kg * g * ff_pre_loiter);
    fprintf('│    Mission fuel fraction   : %.4f  (%.2f%% of W0)\n', ...
        configs(c).Wf_frac, 100*configs(c).Wf_frac);
    fprintf('│\n');
    fprintf('│  Payload (no crew)         : %7.1f kg  (%8.1f lb)  (%.1f%% of W0)\n', ...
        W_payload_kg, W_payload_kg*lb_per_kg, 100*W_payload_kg/W0);
    fprintf('│  Weight closure error      : %.3f kg\n', ...
        abs(configs(c).W_check_kg - W0));
    fprintf('└─────────────────────────────────────────────────────────────────\n\n');
end

% Quick comparison to reference aircraft band
fprintf('─── Comparison with reference aircraft ──────────────────────────────\n');
fprintf('  %-38s: %5.0f kg  (single-engine)\n', 'Cessna 208B Grand Caravan', 3969);
fprintf('  %-38s: %5.0f kg  (single-engine STOL)\n', 'Quest/Daher Kodiak 100', 3062);
fprintf('  %-38s: %5.0f kg  (single-engine)\n', 'Pilatus PC-12 NGX', 4740);
fprintf('  %-38s: %5.0f kg  (twin-engine STOL)\n', 'DHC-6 Twin Otter Series 400', 5670);
for c = 1:length(configs)
    fprintf('  ─── OUR ESTIMATE (%s):   %5.0f kg\n', ...
        configs(c).label, configs(c).W0_kg);
end
fprintf('\n');


%% =========================================================================
%  SECTION 6 — L/D CRUISE SENSITIVITY TABLE
%
%  Sweep L/D_cruise from 8 to 16. At each point:
%    - Recompute ff_cruise via Breguet range equation
%    - Run full Raymer iteration (including updated ff_loiter) for both configs
%    - Report MTOW, EWF, fuel weight, and ff_loiter for each config
%
%  Loiter L/D is implicitly fixed by the aerodynamics (best-endurance CL_BE
%  derived from CD0 and AR in Section 3). It does NOT change with cruise L/D.
%  ff_loiter does change slightly because W0_loiter changes as MTOW shifts.
%
%  Physical interpretation:
%    Lower cruise L/D → more fuel burned → higher Wf_frac → higher W0
%    Twin-engine is MORE sensitive because its higher EWF (two engines,
%    nacelles, duplicated systems) leaves a smaller denominator
%    (1 − EWF − Wf), so a given change in Wf causes a larger fractional
%    change in W0.
% =========================================================================

LD_sweep  = 8:1:16;
nLD       = length(LD_sweep);

W0_sens      = zeros(nLD, 2);
EWF_sens     = zeros(nLD, 2);
Wfuel_sens   = zeros(nLD, 2);
ffloiter_sens = zeros(nLD, 2);
Wf_frac_sens = zeros(nLD, 2);

for idx = 1:nLD
    LD_test = LD_sweep(idx);

    % Breguet cruise fraction at this L/D
    ff_c = exp(-R_m * g * SFCp_SI / (eta_p * LD_test));
    ff_pre_test = ff_warmup * ff_takeoff * ff_climb * ff_c * ff_descent;

    for c = 1:2
        A = configs(c).A;
        C = configs(c).C;
        W0_g = configs(c).W0_kg;   % warm-start from converged baseline

        for it = 1:max_iter
            W0_lbs  = W0_g * lb_per_kg;
            EWF_g   = A * (W0_lbs ^ C) * STOL_penalty;

            % Loiter ff: exact formula, nested as before
            W0_lot_N = W0_g * g * ff_pre_test;
            W1_lot_N = 1 / (t_loiter_s / K_loiter + W0_lot_N^(-0.5))^2;
            ff_lot   = W1_lot_N / W0_lot_N;

            ff_tot_t = ff_pre_test * ff_lot * ff_landing;
            Wf_t     = (1 - ff_tot_t) * (1 + fuel_reserve);

            denom = 1 - EWF_g - Wf_t;
            if denom <= 0; W0_g = W0_g * 1.1; continue; end

            W0_c = W_fixed_kg / denom;
            if abs(W0_c - W0_g) < tol_kg; break; end
            W0_g = 0.5 * W0_g + 0.5 * W0_c;
        end

        W0_sens(idx, c)       = W0_c;
        EWF_sens(idx, c)      = EWF_g;
        Wfuel_sens(idx, c)    = Wf_t * W0_c;
        ffloiter_sens(idx, c) = ff_lot;
        Wf_frac_sens(idx, c)  = Wf_t;
    end
end

% Print sensitivity table
fprintf('=== SECTION 6: L/D CRUISE SENSITIVITY TABLE ===\n');
fprintf('  SFCp = %.3f lb/(hp·hr),  eta_p = %.2f,  S = %.1f m²,  AR = %.1f\n', ...
    SFCp_imp, eta_p, S, AR);
fprintf('  Loiter: exact Breguet formula, CL_BE = %.4f, CD_BE = %.4f\n\n', CL_BE, CD_BE);

% Header
fprintf('  %3s | %6s | %8s | %6s | %6s | %8s || %8s | %6s | %6s | %8s\n', ...
    'L/D', 'Wf/W0', 'SE W0(kg)', 'SE EWF', 'SE ff_l', 'SE Fuel(kg)', ...
    'TE W0(kg)', 'TE EWF', 'TE ff_l', 'TE Fuel(kg)');
fprintf('  %s\n', repmat('-', 1, 100));

baseline_idx = find(LD_sweep == LD_cruise);
for idx = 1:nLD
    mark = '';
    if LD_sweep(idx) == LD_cruise; mark = ' ← baseline'; end
    fprintf('  %3.0f | %6.4f | %8.1f | %6.4f | %6.4f | %8.1f || %8.1f | %6.4f | %6.4f | %8.1f%s\n', ...
        LD_sweep(idx), Wf_frac_sens(idx,1), ...
        W0_sens(idx,1), EWF_sens(idx,1), ffloiter_sens(idx,1), Wfuel_sens(idx,1), ...
        W0_sens(idx,2), EWF_sens(idx,2), ffloiter_sens(idx,2), Wfuel_sens(idx,2), mark);
end

% Delta table from baseline
fprintf('\n  MTOW change relative to L/D = %.0f baseline:\n', LD_cruise);
fprintf('  %3s | %11s | %11s\n', 'L/D', 'ΔW0 SE (kg)', 'ΔW0 TE (kg)');
fprintf('  %s\n', repmat('-', 1, 32));
for idx = 1:nLD
    fprintf('  %3.0f | %+11.1f | %+11.1f\n', LD_sweep(idx), ...
        W0_sens(idx,1) - W0_sens(baseline_idx,1), ...
        W0_sens(idx,2) - W0_sens(baseline_idx,2));
end
fprintf('\n');


%% =========================================================================
%  SECTION 7 — CRITICAL DESIGN NOTES
% =========================================================================

fprintf('=== SECTION 7: DESIGN NOTES ===\n\n');
fprintf([...
  '1. RAYMER UNDERESTIMATES EWF FOR THIS AIRCRAFT CLASS.\n', ...
  '   Raymer single-engine GA coefficients were fit to light piston GA data.\n', ...
  '   Our STOL penalty (%.2fx) partially corrects this, but the reference\n', ...
  '   aircraft survey (Section 1) shows true EWF = 0.54–0.63 vs Raymer\n', ...
  '   prediction of ~0.47–0.50. Use historical mean EWF = %.3f as a check:\n'], ...
  STOL_penalty, mean(SE_EWF));

EWF_hist_check = mean(SE_EWF);
W0_hist_check  = W_fixed_kg / (1 - EWF_hist_check - configs(1).Wf_frac);
fprintf('   Historical EWF estimate: W0 = %.0f kg  vs  Raymer+STOL: W0 = %.0f kg\n', ...
    W0_hist_check, configs(1).W0_kg);
fprintf('   Gap: +%.0f kg (+%.1f%%) — Raymer is a LOWER BOUND on actual MTOW.\n\n', ...
    W0_hist_check - configs(1).W0_kg, ...
    100*(W0_hist_check - configs(1).W0_kg)/configs(1).W0_kg);

fprintf([...
  '2. LOITER FUEL FRACTION IS WEIGHT-DEPENDENT (key change from v2).\n', ...
  '   The exact Breguet formula gives ff_loiter = f(W0_loiter, S, AR, CD0).\n', ...
  '   For S = %.1f m², AR = %.1f: ff_loiter varied %.4f – %.4f across\n', ...
  '   the full MTOW sweep. This is small (~0.3 pp) but grows if S or AR\n', ...
  '   changes significantly — update S and AR in the user block each iter.\n\n'], ...
  S, AR, min(ffloiter_sens(:,1)), max(ffloiter_sens(:,1)));

fprintf([...
  '3. CREW WEIGHT (%.1f kg) IS IN OEW, NOT PAYLOAD.\n', ...
  '   It IS included in W_fixed_kg = W_payload + W_crew = %.1f kg,\n', ...
  '   which drives the sizing denominator. Heavier crew → heavier W0.\n\n'], ...
  W_crew_kg, W_fixed_kg);

fprintf([...
  '4. STOL PENALTY (%.2fx on EWF) IS AN ESTIMATE.\n', ...
  '   It was calibrated from Kodiak (EWF ~0.63) vs Raymer GA prediction.\n', ...
  '   The actual penalty depends on flap type (Fowler vs slotted), gear\n', ...
  '   spec (gravel vs paved), and de-icing system mass. Refine once\n', ...
  '   component weight estimates are available.\n\n'], STOL_penalty);

fprintf([...
  '5. L/D = %.1f IS A CONSERVATIVE BASELINE.\n', ...
  '   The sensitivity table shows each 1-unit improvement in L/D saves\n', ...
  '   ~%.0f kg MTOW (single) and ~%.0f kg (twin). Drag buildup (April 21)\n', ...
  '   will set a tighter bound. Expected range for STOL turboprop: 10–14.\n'], ...
  LD_cruise, ...
  abs(W0_sens(baseline_idx+1,1) - W0_sens(baseline_idx-1,1))/2, ...
  abs(W0_sens(baseline_idx+1,2) - W0_sens(baseline_idx-1,2))/2);
