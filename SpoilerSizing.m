% SpoilerSizing.m
% =========================================================================
% SPOILER AREA SIZING — Landing CD0 Increase via Upper-Surface Spoilers
%
% Goal:
%   Estimate the total spoiler planform area required to achieve a desired
%   landing parasite-drag coefficient (CD0) increase.
%
% Model:
%   1) Compute the additional drag coefficient needed in landing:
%        Delta_CD0 = CD0_landing_target - CD0_landing_baseline
%
%   2) Convert that to an equivalent drag area using the wing reference area:
%        A_spoiler_eq = Delta_CD0 * S_ref
%
%   3) Model the deployed spoiler as a flat plate at a deflection angle theta
%      above the wing, with drag acting on its projected area:
%        A_proj = S_spoiler_total * sin(theta)
%        A_spoiler_eq = Cd_spoiler_normal * A_proj
%
%   4) Solve for the required total spoiler planform area:
%        S_spoiler_total = A_spoiler_eq / (Cd_spoiler_normal * sin(theta))
%
% Notes:
%   - The 60 deg spoiler angle is assumed to be measured from the wing
%     surface / local chord line toward the free stream. If your convention
%     is measured from the normal direction instead, swap sin() for cos().
%   - Cd_spoiler_normal is an engineering estimate for a bluff plate normal
%     to the flow. Replace this with wind-tunnel, CFD, or handbook data if
%     you have spoiler-specific coefficients.
%
% Sources used beyond the attached files:
%   NASA Glenn Research Center, "Drag Coefficient" — Cd definition and
%   reference-area convention:
%     Cd = D / (0.5 * rho * V^2 * A)
%   NASA Glenn Research Center, "Shape Effects on Drag" — flat plate as a
%   high-drag bluff-body baseline and dependence on inclination.
%   Typical flat-plate-normal drag coefficient used here by default:
%     Cd_spoiler_normal = 1.28 (common textbook value for a flat plate
%     normal to the flow; update if your spoiler geometry is different).
%
% All quantities SI unless stated otherwise.
% =========================================================================

clear; clc; close all;

% =========================================================================
%  SECTION 1 — INPUTS
% =========================================================================
  
% Landing drag target and baseline (without spoilers).
% Set these to match your airplane's landing configuration.
CD0_landing_target   = 0.060;   % desired total landing CD0            [-]
CD0_landing_baseline = 0.02757;   % current landing CD0 without spoilers  [-]

% Wing reference area used to normalize CD0.
% Use the same S_ref used in your drag build-up.
S_ref = 36.80;                  % wing reference area                   [m^2]

% Spoiler geometry and drag model.
spoiler_deflection_deg = 60.0;  % spoiler angle above wing surface       [deg]
Cd_spoiler_normal      = 1.28;  % normal-to-flow drag coefficient        [-]

% Optional split into multiple equal-area spoiler panels.
% The script solves the total area, then divides by this count.
n_spoilers = 2;                % number of spoiler panels               [-]

% =========================================================================
%  SECTION 2 — REQUIRED DRAG INCREMENT
% =========================================================================

Delta_CD0_req = CD0_landing_target - CD0_landing_baseline;

if Delta_CD0_req <= 0
    error('Target CD0 must be greater than baseline CD0. Current Delta_CD0 = %.5f', Delta_CD0_req);
end

% Equivalent parasite-drag area required from spoilers.
A_spoiler_eq = Delta_CD0_req * S_ref;     % [m^2]

% =========================================================================
%  SECTION 3 — SPOILER AREA SOLUTION
% =========================================================================
% For a spoiler on the upper wing surface:
%   A_proj = S_spoiler_total * sin(theta)
%   A_spoiler_eq = Cd_spoiler_normal * A_proj
%
% Therefore:
%   S_spoiler_total = (Delta_CD0_req * S_ref) / (Cd_spoiler_normal * sin(theta))

spoiler_deflection_rad = deg2rad(spoiler_deflection_deg);
projection_factor      = sin(spoiler_deflection_rad);

if projection_factor <= 0
    error('Spoiler deflection angle must produce a positive projected area.');
end

S_spoiler_total = A_spoiler_eq / (Cd_spoiler_normal * projection_factor);  % [m^2]
S_spoiler_each  = S_spoiler_total / n_spoilers;                            % [m^2]

% Back-check the achieved CD0 increment from the solved area.
Delta_CD0_check = (Cd_spoiler_normal * S_spoiler_total * projection_factor) / S_ref;
CD0_landing_check = CD0_landing_baseline + Delta_CD0_check;

% Useful ratios for sanity checking.
spoiler_to_wing_ratio = S_spoiler_total / S_ref;
projected_area_total   = S_spoiler_total * projection_factor;

% =========================================================================
%  SECTION 4 — CONSOLE OUTPUT
% =========================================================================

fprintf('\n');
fprintf('==========================================================================\n');
fprintf('  SPOILER AREA SIZING FOR LANDING CD0 TARGET\n');
fprintf('==========================================================================\n\n');

fprintf('  Inputs\n');
fprintf('    CD0_landing_target   = %.5f\n', CD0_landing_target);
fprintf('    CD0_landing_baseline = %.5f\n', CD0_landing_baseline);
fprintf('    S_ref                = %.3f m^2\n', S_ref);
fprintf('    Spoiler angle        = %.1f deg\n', spoiler_deflection_deg);
fprintf('    Cd_spoiler_normal    = %.3f\n\n', Cd_spoiler_normal);

fprintf('  Required Increment\n');
fprintf('    Delta_CD0_req        = %.5f\n', Delta_CD0_req);
fprintf('    A_spoiler_eq         = %.5f m^2 (equivalent drag area)\n\n', A_spoiler_eq);

fprintf('  Solved Spoiler Area\n');
fprintf('    Projection factor    = sin(%.1f deg) = %.4f\n', spoiler_deflection_deg, projection_factor);
fprintf('    Total spoiler area   = %.5f m^2\n', S_spoiler_total);
fprintf('    Area per spoiler     = %.5f m^2  (%d equal panels)\n', S_spoiler_each, n_spoilers);
fprintf('    Total projected area = %.5f m^2\n\n', projected_area_total);

fprintf('  Back-Check\n');
fprintf('    Achieved Delta_CD0   = %.5f\n', Delta_CD0_check);
fprintf('    Landing CD0 checked  = %.5f\n', CD0_landing_check);
fprintf('    Spoiler/Wing ratio   = %.3f %%\n\n', 100 * spoiler_to_wing_ratio);

fprintf('==========================================================================\n');
fprintf('  RESULT: Use a total spoiler planform area of %.5f m^2\n', S_spoiler_total);
fprintf('==========================================================================\n\n');

% =========================================================================
%  SECTION 5 — OPTIONAL SIMPLE PLOT
% =========================================================================

figure('Name','Spoiler Area Sizing','Units','normalized','Position',[0.18 0.20 0.50 0.42]);
bar([CD0_landing_baseline, CD0_landing_target, CD0_landing_check]);
xticks(1:3);
xticklabels({'Baseline','Target','Check'});
ylabel('Landing CD_0  [-]');
title(sprintf('Landing Drag Target and Spoiler-Sized Check  (S_{spoiler,total} = %.4f m^2)', S_spoiler_total));
grid on;
box on;

