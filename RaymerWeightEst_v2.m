%% RaymerWeightEst_v2.m
% =========================================================================
% Arctic STOL Aircraft — Initial Takeoff Weight Estimation
% Method: Raymer statistical EWF, iterated to convergence
%
% Key changes from v1:
%   - Crew (181 kg) moved OUT of payload and INTO OEW, as crew weight does
%     not change flight-to-flight and is part of the operating configuration.
%   - Max payload set to 1100 kg (9 pax × 250 lb RFP allocation, no crew).
%   - Both Raymer single-engine AND twin-engine prop coefficients computed
%     side-by-side for direct comparison.
%   - SFCp = 0.548 lb/(hp·hr), η_p = 0.80 (fixed per user input).
%   - Cruise fuel fraction via Breguet range equation (propeller form, SI).
%   - 45-min loiter reserve included via Breguet endurance equation.
%   - Sensitivity table: L/D 8–16 vs MTOW for both engine configurations.
%
% IMPORTANT LIMITATIONS (see Section 6 for full discussion):
%   - Raymer's single-engine GA coefficients (A=2.36, C=-0.18) are derived
%     from piston-engine GA data. They UNDERESTIMATE EWF for turboprops by
%     ~0.08–0.10 points. Reference aircraft survey (Section 1) shows the
%     true EWF band for this class is 0.54–0.63.
%   - Treat these results as a LOWER BOUND on MTOW, not a best estimate.
%   - Re-run once engine is selected and component weights are available.
%
% Units: SI (kg, m, N, W) throughout. Imperial used only inside Raymer's
%        formula, clearly labelled. MATLAB does not enforce units, so read
%        every comment before changing values.
%
% Authors: AME 261 Design Team
% Date:    April 2026
% Version: 2.0
% =========================================================================

clear; clc; close all;

%% =========================================================================
%  SECTION 1 — REFERENCE AIRCRAFT SURVEY
%  Purpose: Establish a reality-check band for EWF before running Raymer.
%           The Raymer formula gives one number; real aircraft give the range
%           we should expect to land in.
% =========================================================================

% Struct array: name, pax cap, MTOW [kg], OEW [kg], engine count, notes
% OEW here is Operating Empty Weight (includes crew by convention).
% Sources: manufacturer data sheets, Jane's All the World's Aircraft.

ref(1).name    = 'Cessna 208B Grand Caravan';
ref(1).pax     = 9;
ref(1).MTOW_kg = 3969;
ref(1).OEW_kg  = 2145;
ref(1).engines = 1;
ref(1).notes   = 'Baseline single-engine turboprop; widely used in Arctic / Alaska ops';

ref(2).name    = 'Quest/Daher Kodiak 100';
ref(2).pax     = 9;
ref(2).MTOW_kg = 3062;
ref(2).OEW_kg  = 1934;
ref(2).engines = 1;
ref(2).notes   = 'Purpose-built STOL; most mission-relevant comparator';

ref(3).name    = 'Pilatus PC-12 NGX';
ref(3).pax     = 9;
ref(3).MTOW_kg = 4740;
ref(3).OEW_kg  = 2812;
ref(3).engines = 1;
ref(3).notes   = 'Pressurized single turboprop; upper end of weight range';

ref(4).name    = 'DHC-6 Twin Otter Series 400';
ref(4).pax     = 19;   % oversized pax count, kept for structural reference
ref(4).MTOW_kg = 5670;
ref(4).OEW_kg  = 3363;
ref(4).engines = 2;
ref(4).notes   = 'Classic Arctic twin; too large for 9-pax but best STOL twin reference';

ref(5).name    = 'Britten-Norman BN-2 Islander';
ref(5).pax     = 9;
ref(5).MTOW_kg = 2994;
ref(5).OEW_kg  = 1816;
ref(5).engines = 2;
ref(5).notes   = 'Light twin, STOL-capable; lower bound for twin config EWF';

ref(6).name    = 'Tecnam P2012 Traveller';
ref(6).pax     = 9;
ref(6).MTOW_kg = 3900;
ref(6).OEW_kg  = 2400;
ref(6).engines = 2;
ref(6).notes   = 'Modern twin-piston 9-pax utility; piston EWF is higher than turboprop';

% Compute and print EWF for each reference aircraft
fprintf('=== SECTION 1: REFERENCE AIRCRAFT SURVEY ===\n');
fprintf('%-35s | Eng | %9s | %8s | %5s\n', 'Aircraft', 'MTOW (kg)', 'OEW (kg)', 'EWF');
fprintf('%s\n', repmat('-', 1, 75));
for i = 1:length(ref)
    ref(i).EWF = ref(i).OEW_kg / ref(i).MTOW_kg;
    fprintf('%-35s |  %d  | %9.0f | %8.0f | %5.3f\n', ...
        ref(i).name, ref(i).engines, ref(i).MTOW_kg, ref(i).OEW_kg, ref(i).EWF);
end

% Separate single-engine and twin-engine reference EWF bands
SE_idx = [ref.engines] == 1 & [ref.pax] == 9;  % single-eng, 9-pax only
TE_idx = [ref.engines] == 2;
SE_EWF = [ref(SE_idx).EWF];
TE_EWF = [ref(TE_idx).EWF];

fprintf('\nSingle-engine 9-pax band:   EWF = %.3f – %.3f  (mean %.3f)\n', ...
    min(SE_EWF), max(SE_EWF), mean(SE_EWF));
fprintf('Twin-engine reference band: EWF = %.3f – %.3f  (mean %.3f)\n\n', ...
    min(TE_EWF), max(TE_EWF), mean(TE_EWF));


%% =========================================================================
%  SECTION 2 — FIXED DESIGN PARAMETERS
%  These values drive the sizing loop. Update each design iteration.
% =========================================================================

% --- Payload (mission-variable weight: passengers + baggage only) ---
% RFP Configuration 1 (design mission): 9 pax × (200 lb body + 50 lb bag)
% Max payload from RFP medevac config is 1,150 lb ~ 522 kg, but the
% passenger config (2,250 lb = 1021 kg) governs MTOW for sizing.
% Per user direction, payload is set at the maximum value of 1100 kg.
% CREW IS NOT PAYLOAD — crew weight is placed in OEW below.
W_payload_kg = 1100;   % [kg]  maximum design payload, no crew

% --- Crew (goes into OEW — does not change between flights) ---
% RFP: 2 pilots × (180 lb body + 20 lb baggage) = 200 lb/pilot = 400 lb total
% 400 lb × 0.4536 kg/lb = 181.4 kg
W_crew_kg = 2 * 200 * 0.4536;   % [kg]  = 181.4 kg

% --- Total fixed non-fuel weight (drives sizing denominator) ---
% This is the weight the empty aircraft + fuel must support.
W_fixed_kg = W_payload_kg + W_crew_kg;   % [kg]

% --- Propulsion parameters ---
% SFCp = 0.548 lb/(hp·hr)   — from user input; consistent with PT6A-class turboprops
% η_p  = 0.80                — propeller efficiency (fixed per user input)
SFCp_imp = 0.548;            % specific fuel consumption [lb/(hp·hr)]
eta_p    = 0.80;             % propeller efficiency [-]

% Convert SFCp to SI: kg per Watt per second [kg/(W·s)]
%   1 lb/(hp·hr) × (0.4536 kg/lb) / (745.7 W/hp × 3600 s/hr)
%   Denominator = 745.7 × 3600 = 2,684,520 W·s/hp·hr
SFCp_SI = SFCp_imp * 0.4536 / (745.7 * 3600);   % [kg/(W·s)]

% --- Aerodynamic parameters (baseline, swept in Section 5) ---
LD_cruise = 12.0;   % L/D at cruise — typical STOL turboprop; conservative for STOL wing
LD_loiter = 14.0;   % L/D at loiter — best endurance flies at higher CL than cruise;
                    % approximately L/D_max for this aircraft class

% --- Mission parameters ---
R_nmi       = 450;    % required cruise range             [nmi]  (RFP Section 1.3)
R_m         = R_nmi * 1852;   % convert to metres         [m]
t_loiter_s  = 45 * 60;        % 45-min loiter reserve     [s]
V_loiter_ms = 90 * 0.5144;    % best endurance speed ~90 ktas → [m/s]
g           = 9.81;            % gravitational acceleration [m/s²]
fuel_reserve = 0.05;           % 5% additional fuel reserve (RFP + operational margin)

fprintf('=== SECTION 2: DESIGN INPUTS ===\n');
fprintf('  Payload (no crew) : %.1f kg  (%.1f lb)\n', W_payload_kg, W_payload_kg/0.4536);
fprintf('  Crew weight       : %.1f kg  (%.1f lb)  [placed in OEW]\n', W_crew_kg, W_crew_kg/0.4536);
fprintf('  Total fixed wt    : %.1f kg\n', W_fixed_kg);
fprintf('  SFCp              : %.3f lb/(hp·hr)  =  %.4e kg/(W·s)\n', SFCp_imp, SFCp_SI);
fprintf('  eta_p             : %.2f\n', eta_p);
fprintf('  Cruise L/D        : %.1f  (baseline; swept in Section 5)\n', LD_cruise);
fprintf('  Loiter L/D        : %.1f\n\n', LD_loiter);


%% =========================================================================
%  SECTION 3 — FUEL FRACTION ESTIMATION
%  Using Breguet equations in SI for cruise and loiter.
%  All other segments use Raymer Table 3.2 historical fractions.
%
%  Breguet range (propeller aircraft):
%    R = (η_p / (g × SFCp_SI)) × (L/D) × ln(W_i / W_{i+1})
%    → W_{i+1}/W_i = exp( -R × g × SFCp_SI / (η_p × L/D) )
%
%  Breguet endurance (propeller aircraft):
%    E = (η_p / (V × g × SFCp_SI)) × (L/D) × ln(W_i / W_{i+1})
%    → W_{i+1}/W_i = exp( -E × V × g × SFCp_SI / (η_p × L/D) )
%
%  NOTE on the "375 shorthand": Some textbooks write R[miles] = (375×η_p/SFCp)×(L/D)×ln(...)
%  The factor 375 is ONLY valid when R is in STATUTE miles. For nautical miles
%  the correct factor is ~325.9. The SI form used here avoids this ambiguity.
% =========================================================================

% Breguet cruise fuel fraction
ff_cruise = exp( -R_m * g * SFCp_SI / (eta_p * LD_cruise) );

% Breguet loiter fuel fraction (45-min reserve at best endurance speed)
ff_loiter = exp( -t_loiter_s * V_loiter_ms * g * SFCp_SI / (eta_p * LD_loiter) );

% Fixed historical fractions (Raymer Table 3.2) for non-cruise segments
ff_warmup   = 0.990;   % warm-up and taxi (10 min)
ff_takeoff  = 0.995;   % STOL takeoff — slightly worse than FAR-25 (higher thrust time)
ff_climb    = 0.980;   % climb to cruise altitude
ff_descent  = 0.990;   % descent to sea level
ff_landing  = 0.992;   % landing rollout and taxi

% Total mission fuel fraction (product of all segment fractions)
% W_final / W_initial  — fraction of initial weight REMAINING after all segments
ff_total = ff_warmup   * ff_takeoff  * ff_climb   * ff_cruise ...
         * ff_descent  * ff_loiter   * ff_landing;

% Mission fuel used as a fraction of MTOW
Mff = 1 - ff_total;

% Add fuel reserve: total fuel fraction of W0
Wf_frac = Mff * (1 + fuel_reserve);

fprintf('=== SECTION 3: FUEL FRACTION BREAKDOWN ===\n');
fprintf('  Segment          |  W_{i+1}/W_i  | Cumulative\n');
fprintf('  %s\n', repmat('-', 1, 45));
cum = 1.0;
segs  = {ff_warmup, ff_takeoff, ff_climb, ff_cruise, ff_descent, ff_loiter, ff_landing};
names = {'Warm-up & taxi', 'STOL Takeoff', 'Climb', 'Cruise (Breguet)', ...
         'Descent', 'Loiter 45min (Breguet)', 'Landing & taxi'};
for s = 1:length(segs)
    cum = cum * segs{s};
    fprintf('  %-24s | %13.4f | %10.4f\n', names{s}, segs{s}, cum);
end
fprintf('\n  Mission fuel fraction (Mff):     %.4f  (%.2f%% of W0)\n', Mff, Mff*100);
fprintf('  With %.0f%% reserve (Wf/W0):      %.4f  (%.2f%% of W0)\n\n', ...
    fuel_reserve*100, Wf_frac, Wf_frac*100);


%% =========================================================================
%  SECTION 4 — RAYMER ITERATIVE WEIGHT SIZING (SINGLE & TWIN ENGINE)
%
%  Raymer formula (Table 3.1, "Aircraft Design: A Conceptual Approach"):
%    EWF = A × W0_lbs^C
%
%  Coefficients used:
%    Single-engine prop:  A = 2.36, C = -0.18  (Raymer Table 3.1 row 1)
%    Twin-engine prop:    A = 1.51, C = -0.10  (Raymer Table 3.1 row 2)
%
%  These are the GA-class coefficients. Both sets apply to propeller aircraft.
%  W0 must be in POUNDS for the formula; conversion applied at each iteration.
%
%  Weight equation:
%    W0 = W_structural_empty + W_crew + W_fuel + W_payload
%    W0 = EWF × W0   +   W_crew   +   Wf_frac × W0   +   W_payload
%
%  Note: Raymer's GA EWF reflects the manufacturer's empty weight (no crew).
%  Crew is therefore added separately as a constant, consistent with treating
%  crew as part of OEW but outside the EWF scaling relationship.
%
%  Solving for W0:
%    W0 × (1 - EWF - Wf_frac) = W_crew + W_payload = W_fixed
%    W0 = W_fixed / (1 - EWF - Wf_frac)
%
%  Because EWF itself depends on W0 (power law), iterate to convergence.
%  Blend 50/50 old/new to prevent oscillation.
% =========================================================================

% Raymer coefficients
configs(1).label = 'Single-Engine Prop';
configs(1).A     = 2.36;
configs(1).C     = -0.18;
configs(1).W0_0  = 4000;   % initial guess [kg]

configs(2).label = 'Twin-Engine Prop';
configs(2).A     = 1.51;
configs(2).C     = -0.10;
configs(2).W0_0  = 5000;   % initial guess — twins typically heavier

tol_kg   = 0.5;    % convergence tolerance [kg]
max_iter = 300;
lb_per_kg = 2.20462;

fprintf('=== SECTION 4: RAYMER ITERATIVE WEIGHT SIZING ===\n\n');

for c = 1:length(configs)
    A = configs(c).A;
    C = configs(c).C;
    W0_guess = configs(c).W0_0;

    fprintf('--- %s  (A = %.2f, C = %.2f) ---\n', configs(c).label, A, C);
    fprintf('  Iter | W0_guess [kg] |    EWF   | W0_calc [kg] | Delta [kg]\n');
    fprintf('  %s\n', repmat('-', 1, 60));

    converged = false;
    for iter = 1:max_iter
        W0_lbs = W0_guess * lb_per_kg;               % convert to lbs for Raymer
        EWF    = A * (W0_lbs ^ C);                   % Raymer statistical EWF

        % Check physical feasibility: if EWF + Wf_frac >= 1, no fuel/payload
        % can be carried — the aircraft is structurally impossible at this weight.
        denom = 1 - EWF - Wf_frac;
        if denom <= 0
            fprintf('  INFEASIBLE: EWF + Wf_frac = %.3f >= 1 at W0 = %.0f kg\n', ...
                EWF + Wf_frac, W0_guess);
            W0_guess = W0_guess * 1.2;   % try a larger initial guess
            continue
        end

        W0_calc = W_fixed_kg / denom;   % W_fixed = W_payload + W_crew
        delta   = abs(W0_calc - W0_guess);

        % Print first 10 iterations and final convergence steps
        if iter <= 10 || delta < 5
            fprintf('  %4d | %12.1f | %8.4f | %12.1f | %9.2f\n', ...
                iter, W0_guess, EWF, W0_calc, delta);
        end

        if delta < tol_kg
            converged = true;
            break
        end
        % Blend to avoid oscillation around the fixed point
        W0_guess = 0.5 * W0_guess + 0.5 * W0_calc;
    end

    % Store results
    configs(c).W0_kg      = W0_calc;
    configs(c).EWF        = EWF;
    configs(c).W_struct_kg = EWF    * W0_calc;   % structural empty (no crew)
    configs(c).W_OEW_kg   = EWF * W0_calc + W_crew_kg;  % OEW = structural + crew
    configs(c).W_fuel_kg  = Wf_frac * W0_calc;
    configs(c).W_check_kg = configs(c).W_OEW_kg + configs(c).W_fuel_kg + W_payload_kg;
    configs(c).converged  = converged;

    fprintf('\n  Converged: %s  |  Iterations: %d\n\n', mat2str(converged), iter);
end


%% =========================================================================
%  SECTION 4b — FINAL WEIGHT STATEMENTS
% =========================================================================

fprintf('=== SECTION 4b: FINAL WEIGHT STATEMENTS ===\n\n');

for c = 1:length(configs)
    W0 = configs(c).W0_kg;
    fprintf('┌─────────────────────────────────────────────────────────\n');
    fprintf('│  Configuration: %s\n', configs(c).label);
    fprintf('│  Raymer Coefficients: A = %.2f, C = %.2f\n', configs(c).A, configs(c).C);
    fprintf('├─────────────────────────────────────────────────────────\n');
    fprintf('│  Max Takeoff Weight (W0):  %7.1f kg  (%7.1f lb)\n', ...
        W0, W0*lb_per_kg);
    fprintf('│  ─ Structural Empty Wt:   %7.1f kg  (%7.1f lb)  (EWF = %.4f of W0)\n', ...
        configs(c).W_struct_kg, configs(c).W_struct_kg*lb_per_kg, configs(c).EWF);
    fprintf('│  ─ Crew Weight (in OEW):  %7.1f kg  (%7.1f lb)\n', ...
        W_crew_kg, W_crew_kg*lb_per_kg);
    fprintf('│  ─ OEW (struct + crew):   %7.1f kg  (%7.1f lb)  (%.1f%% of W0)\n', ...
        configs(c).W_OEW_kg, configs(c).W_OEW_kg*lb_per_kg, ...
        100*configs(c).W_OEW_kg/W0);
    fprintf('│  ─ Fuel Weight:           %7.1f kg  (%7.1f lb)  (%.1f%% of W0)\n', ...
        configs(c).W_fuel_kg, configs(c).W_fuel_kg*lb_per_kg, ...
        100*configs(c).W_fuel_kg/W0);
    fprintf('│  ─ Payload:               %7.1f kg  (%7.1f lb)  (%.1f%% of W0)\n', ...
        W_payload_kg, W_payload_kg*lb_per_kg, 100*W_payload_kg/W0);
    fprintf('│  Weight check (sum):      %7.1f kg  (closure error: %.2f kg)\n', ...
        configs(c).W_check_kg, abs(configs(c).W_check_kg - W0));
    fprintf('└─────────────────────────────────────────────────────────\n\n');
end

% Comparison with reference aircraft
fprintf('─── Reference aircraft MTOW band ──────────────────────────\n');
fprintf('  %-35s: %5.0f kg  (single-engine)\n', 'Cessna 208B Grand Caravan', 3969);
fprintf('  %-35s: %5.0f kg  (single-engine, STOL)\n', 'Quest/Daher Kodiak 100', 3062);
fprintf('  %-35s: %5.0f kg  (single-engine)\n', 'Pilatus PC-12 NGX', 4740);
fprintf('  %-35s: %5.0f kg  (twin-engine STOL)\n', 'DHC-6 Twin Otter Series 400', 5670);
for c = 1:length(configs)
    fprintf('  ─── OUR ESTIMATE (%s): %5.0f kg\n', configs(c).label, configs(c).W0_kg);
end
fprintf('\n');


%% =========================================================================
%  SECTION 5 — L/D SENSITIVITY TABLE
%  Sweep L/D_cruise from 8 to 16 and re-run the full Raymer iteration for
%  both configurations. This shows how sensitive MTOW is to the aerodynamic
%  assumption, which is the least certain input at this design stage.
%
%  Loiter L/D is held fixed at LD_loiter = 14 throughout, since best-endurance
%  L/D is less sensitive to wing design than cruise L/D.
%
%  Physical interpretation:
%    Lower L/D → more fuel burned → higher Wf_frac → heavier W0
%    The twin-engine config has higher EWF, so it is more sensitive to L/D
%    changes (the denominator 1-EWF-Wf is smaller, so any change in Wf has
%    a larger fractional effect on W0).
% =========================================================================

LD_sweep = 8:1:16;   % L/D range to sweep [–]
nLD = length(LD_sweep);

% Pre-allocate result matrices [nLD × 2]
W0_sens       = zeros(nLD, 2);   % MTOW [kg]
EWF_sens      = zeros(nLD, 2);   % converged EWF [–]
Wfuel_sens    = zeros(nLD, 2);   % fuel weight [kg]
Wf_frac_sens  = zeros(nLD, 1);   % fuel fraction (same for both configs at same L/D)

for idx = 1:nLD
    LD_test = LD_sweep(idx);

    % Recompute Breguet cruise fraction at this L/D
    ff_c = exp( -R_m * g * SFCp_SI / (eta_p * LD_test) );
    % Loiter L/D held fixed at LD_loiter (best endurance, independent of cruise)
    ff_l = exp( -t_loiter_s * V_loiter_ms * g * SFCp_SI / (eta_p * LD_loiter) );

    ff_tot  = ff_warmup * ff_takeoff * ff_climb * ff_c * ff_descent * ff_l * ff_landing;
    Mff_t   = 1 - ff_tot;
    Wf_t    = Mff_t * (1 + fuel_reserve);
    Wf_frac_sens(idx) = Wf_t;

    % Iterate Raymer for each config
    for c = 1:2
        A = configs(c).A;
        C = configs(c).C;
        W0_g = configs(c).W0_kg;   % start from previously converged value
        for it = 1:max_iter
            W0_lbs = W0_g * lb_per_kg;
            EWF_g  = A * (W0_lbs ^ C);
            denom  = 1 - EWF_g - Wf_t;
            if denom <= 0; W0_g = W0_g * 1.1; continue; end
            W0_c   = W_fixed_kg / denom;
            if abs(W0_c - W0_g) < tol_kg; break; end
            W0_g   = 0.5*W0_g + 0.5*W0_c;
        end
        W0_sens(idx, c)    = W0_c;
        EWF_sens(idx, c)   = EWF_g;
        Wfuel_sens(idx, c) = Wf_t * W0_c;
    end
end

% Print sensitivity table
fprintf('=== SECTION 5: L/D SENSITIVITY TABLE ===\n');
fprintf('(Loiter L/D fixed at %.1f; cruise L/D swept; SFCp = %.3f lb/(hp·hr), eta_p = %.2f)\n\n', ...
    LD_loiter, SFCp_imp, eta_p);
fprintf('  %4s | %6s || %10s | %7s | %9s || %10s | %7s | %9s\n', ...
    'L/D', 'Wf/W0', 'SE W0 [kg]', 'SE EWF', 'SE Fuel[kg]', ...
    'TE W0 [kg]', 'TE EWF', 'TE Fuel[kg]');
fprintf('  %s\n', repmat('-', 1, 90));
for idx = 1:nLD
    LD_mark = '';
    if LD_sweep(idx) == LD_cruise; LD_mark = ' ← baseline'; end
    fprintf('  %4.0f | %6.4f || %10.1f | %7.4f | %9.1f || %10.1f | %7.4f | %9.1f%s\n', ...
        LD_sweep(idx), Wf_frac_sens(idx), ...
        W0_sens(idx,1), EWF_sens(idx,1), Wfuel_sens(idx,1), ...
        W0_sens(idx,2), EWF_sens(idx,2), Wfuel_sens(idx,2), LD_mark);
end

% Delta from baseline (LD = 12) for each config
baseline_idx = find(LD_sweep == LD_cruise);
fprintf('\n  Delta from L/D = %.0f baseline:\n', LD_cruise);
fprintf('  %4s | %10s | %10s\n', 'L/D', 'ΔW0 SE [kg]', 'ΔW0 TE [kg]');
fprintf('  %s\n', repmat('-', 1, 32));
for idx = 1:nLD
    fprintf('  %4.0f | %+10.1f | %+10.1f\n', LD_sweep(idx), ...
        W0_sens(idx,1) - W0_sens(baseline_idx,1), ...
        W0_sens(idx,2) - W0_sens(baseline_idx,2));
end
fprintf('\n');


%% =========================================================================
%  SECTION 6 — CRITICAL DESIGN NOTES
%  (Read before presenting these numbers in a design review.)
% =========================================================================

fprintf('=== SECTION 6: CRITICAL DESIGN NOTES ===\n');
fprintf([...
  '1. RAYMER EWF UNDERESTIMATES THIS CLASS OF AIRCRAFT.\n'...
  '   Raymer single-engine GA coefficients (A=2.36, C=-0.18) were fit to\n'...
  '   light piston aircraft. Our reference aircraft show EWF = 0.54–0.63;\n'...
  '   Raymer predicts ~0.49. This means our Raymer MTOW is a LOWER BOUND.\n'...
  '   Use the reference aircraft mean EWF = %.3f as a parallel check:\n'], ...
  mean(SE_EWF));
EWF_hist = mean(SE_EWF);
W0_hist  = W_fixed_kg / (1 - EWF_hist - Wf_frac);
fprintf('   Historical-EWF estimate:  W0 = %.0f kg  vs  Raymer: W0 = %.0f kg\n', ...
    W0_hist, configs(1).W0_kg);
fprintf('   Difference: +%.0f kg  (+%.1f%%) — Raymer UNDERESTIMATES by this margin.\n\n', ...
    W0_hist - configs(1).W0_kg, 100*(W0_hist-configs(1).W0_kg)/configs(1).W0_kg);
fprintf([...
  '2. CREW IN OEW: W_crew = %.1f kg is correctly placed in OEW, not payload.\n'...
  '   It IS included in the sizing denominator (W_fixed = W_payload + W_crew).\n'...
  '   This means a heavier crew makes the aircraft heavier, not just the payload.\n\n'...
  '3. FUEL FRACTION CONFIDENCE: Breguet L/D = %.1f is a rough estimate.\n'...
  '   The sensitivity table shows MTOW changes ~%.0f kg per L/D unit (single-eng).\n'...
  '   Refine L/D once the wing and drag buildup are complete (April 21 deliverable).\n\n'...
  '4. STOL PENALTY NOT APPLIED here (removed for clean Raymer comparison).\n'...
  '   Real STOL aircraft (Kodiak) show EWF ~0.63 — approximately 0.13 above\n'...
  '   the raw Raymer prediction. Heavier flap systems, gravel-rated gear,\n'...
  '   de-icing equipment, and structural reinforcement all contribute.\n\n'...
  '5. TWIN-ENGINE RESULT: Raymer predicts a heavier W0 for the twin config.\n'...
  '   This is driven by the twin''s higher EWF (two engines, nacelles, systems).\n'...
  '   The DHC-6 Twin Otter (the primary real-world twin STOL reference) has\n'...
  '   W0 = 5670 kg, consistent with the twin Raymer result.\n'...
  '   The twin is heavier but offers redundancy and better STOL climb gradient.\n'], ...
  W_crew_kg, LD_cruise, ...
  abs(W0_sens(baseline_idx+1,1) - W0_sens(baseline_idx-1,1))/2);
