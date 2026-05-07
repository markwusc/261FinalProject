% VN_Diagrams_updated.m
% =========================================================================
% V-n / sustained-turn envelope updated from ConstraintAnalysis_TO_Land_v2.m
%
% This script uses the same shared aircraft parameters, propulsion model,
% weights, and 5% margin design points from the constraint-analysis file.
%
% Design points taken from the 5% margin contour intersection in the
% constraint sweep:
%   Single engine: S = 39.85 m^2, AR = 6.85, b = 16.53 m
%   Twin engine:   S = 47.89 m^2, AR = 5.68, b = 16.49 m
%
% IMPORTANT UPDATE:
% The plotted x-axis is automatically limited to the rightmost valid point
% of the PA = PR sustained-turn curve for each configuration and altitude.
% To make this work for the twin-engine case, the calculation velocity range
% must extend beyond the expected cutoff speed. The plot then crops itself.
%
% ADDED:
% A turn radius vs dynamic pressure graph is included as Figure 2.
% The minimum achievable turn-radius region is highlighted in yellow.
%
% StdAtmos is included as a local function, so this file can run standalone.
% =========================================================================

clear; clc; close all;

%% Shared parameters from ConstraintAnalysis_TO_Land_v2.m

% Physical constants
g       = 9.81;          % gravitational acceleration [m/s^2]
rho_SL  = 1.225;         % sea-level ISA density [kg/m^3]
shp2W   = 745.7;         % shaft horsepower to watts

% Propulsion
P_shp    = 1050;         % rated shaft power per engine [hp]
eta_prop = 0.80;         % propeller efficiency [-]

% Clean aerodynamic values used for cruise / V-n analysis
CD_0_clean   = 0.030;    % clean zero-lift drag coefficient [-]
CL_max_clean = 1.8;      % clean maximum lift coefficient [-]
e            = 0.90;     % Oswald efficiency factor [-]

% Configuration weights and engine counts from constraint analysis
W_To_arr  = [39867.9, 60425.8];     % MTOW for SE and TE [N]
n_eng_arr = [1, 2];                 % engine count [-]

% Propulsive power available at sea level
P_A_arr = eta_prop * P_shp * shp2W .* n_eng_arr;  % [W]

cfg_names = { ...
    'Single Engine  (42 kN, 1 x 1050 shp)', ...
    'Twin Engine    (61 kN, 2 x 1050 shp)' };

% Design points selected from the 5% margin contour intersection in the
% constraint-analysis sweep.
S_design  = [39.849246231155774, 47.889447236180900];  % [m^2]
AR_design = [ 6.854271356783920,  5.678391959798995];  % [-]
b_design  = sqrt(S_design .* AR_design);               % [m]

% Structural limit.
% DHC-6 / Normal Category positive limit load factor.
n_struct_pos = 3.8;     % positive limit load factor [-]

% Sustained-turn power setting.
% Use 1.00 for max power. Use 0.90 if matching the cruise-altitude analysis.
throttle_frac = 1.00;

%% Altitudes and velocity range

h_cases = [0, 6000];                          % altitude cases [m]
[~, rho_cases, ~, ~] = StdAtmos_local(h_cases);

V_min = 1.0;                                  % avoid division by zero [m/s]

% Use a high calculation limit so the twin-engine PA = PR curve actually
% reaches its cutoff. The plot will be cropped to the valid PA curve max.
V_max_calc = 250;                             % calculation max speed [m/s]
V_vn = linspace(V_min, V_max_calc, 1200);     % velocity vector [m/s]

%% Storage arrays for turn-radius figure

turn_data = struct();
case_count = 0;

%% Console summary

fprintf('=== V-n Diagram Inputs Updated from Constraint Analysis ===\n')
for cfg = 1:2
    fprintf('  %s\n', cfg_names{cfg})
    fprintf('    S    = %.2f m^2\n', S_design(cfg))
    fprintf('    AR   = %.2f\n', AR_design(cfg))
    fprintf('    b    = %.2f m\n', b_design(cfg))
    fprintf('    W    = %.1f N  (%.0f kg)\n', W_To_arr(cfg), W_To_arr(cfg)/g)
    fprintf('    P_A  = %.1f kW\n\n', P_A_arr(cfg)/1000)
end

%% Figure 1: Build V-n diagrams

fig1 = figure('Name','Updated V-n Diagrams from Constraint Analysis', ...
              'Units','normalized','Position',[0.03 0.07 0.92 0.78]);

plot_idx = 1;

for cfg = 1:2

    % Aircraft-specific values
    ac.S      = S_design(cfg);
    ac.AR     = AR_design(cfg);
    ac.b      = b_design(cfg);
    ac.W      = W_To_arr(cfg);
    ac.P_A_SL = P_A_arr(cfg);
    ac.CD_0   = CD_0_clean;
    ac.CLmax  = CL_max_clean;
    ac.e      = e;
    ac.k      = 1 / (pi * ac.e * ac.AR);
    ac.n_lim  = n_struct_pos;

    for h_idx = 1:numel(h_cases)

        h   = h_cases(h_idx);
        rho = rho_cases(h_idx);

        q = 0.5 .* rho .* V_vn.^2;

        % Power available with density lapse, matching the turboprop model
        % used in the constraint analysis.
        P_A_h = throttle_frac * ac.P_A_SL * (rho / rho_SL);

        % Lift-limited load factor:
        % n = L/W = q S CLmax / W
        n_CL = q .* ac.S .* ac.CLmax ./ ac.W;

        % Structural limit
        n_struct = ac.n_lim .* ones(size(V_vn));

        % Sustained power limit:
        %
        % Drag in a steady level/turning condition:
        % D = q S CD0 + k (nW)^2 / (qS)
        %
        % Power required:
        % P_R = D V
        %
        % Solving P_A = P_R for n:
        % n^2 = [((P_A/V) - qSCD0)(qS)] / [kW^2]
        n_power_sq = (((P_A_h ./ V_vn) - ac.CD_0 .* q .* ac.S) .* ...
                       (q .* ac.S)) ./ (ac.k .* ac.W.^2);

        n_power = sqrt(n_power_sq);
        n_power(n_power_sq <= 0) = NaN;

        % Find the maximum plotted velocity based on the rightmost valid
        % point of the PA = PR curve.
        valid_power_idx = find(isfinite(n_power));

        if ~isempty(valid_power_idx)
            V_PA_max = V_vn(valid_power_idx(end));
        else
            V_PA_max = V_max_calc;
        end

        q_PA_max = 0.5 * rho * V_PA_max^2;

        % Absolute aerodynamic reference limit based on E_max and equivalent
        % thrust T = P/V. This is included only as a reference line.
        E_max = 1 / (2 * sqrt(ac.CD_0 * ac.k));
        T_equiv = P_A_h ./ V_vn;
        n_aero = E_max .* (T_equiv ./ ac.W);
        n_aero(n_aero < 0) = NaN;

        % Achievable positive envelope.
        % The aircraft is limited by whichever curve is lowest.
        n_mat = [n_CL; n_struct; n_power; n_aero];
        n_env = min(n_mat, [], 1, 'omitnan');
        n_env(isnan(n_env)) = 0;
        n_env = max(n_env, 0);

        % Important speeds
        V_stall_1g = sqrt(2 * ac.W / (rho * ac.S * ac.CLmax));
        V_corner   = sqrt(2 * ac.W * ac.n_lim / ...
                          (rho * ac.S * ac.CLmax));

        %% Turn radius calculations for Figure 2
        %
        % Coordinated level turn radius:
        % r = V^2 / (g*sqrt(n^2 - 1))
        %
        % Any case with n <= 1 cannot produce a steady level turn, so it is
        % removed from the plotted radius curves.

        r_struct = V_vn.^2 ./ (g .* sqrt(n_struct.^2 - 1));
        r_CL     = V_vn.^2 ./ (g .* sqrt(n_CL.^2     - 1));
        r_power  = V_vn.^2 ./ (g .* sqrt(n_power.^2  - 1));
        r_env    = V_vn.^2 ./ (g .* sqrt(n_env.^2    - 1));

        r_CL(n_CL <= 1)         = NaN;
        r_power(n_power <= 1)   = NaN;
        r_env(n_env <= 1)       = NaN;

        r_struct(~isfinite(r_struct)) = NaN;
        r_CL(~isfinite(r_CL))         = NaN;
        r_power(~isfinite(r_power))   = NaN;
        r_env(~isfinite(r_env))       = NaN;

        % Store data for Figure 2
        case_count = case_count + 1;
        turn_data(case_count).cfg_name   = cfg_names{cfg};
        turn_data(case_count).h          = h;
        turn_data(case_count).rho        = rho;
        turn_data(case_count).q          = q;
        turn_data(case_count).q_PA_max   = q_PA_max;
        turn_data(case_count).r_struct   = r_struct;
        turn_data(case_count).r_CL       = r_CL;
        turn_data(case_count).r_power    = r_power;
        turn_data(case_count).r_env      = r_env;

        %% Plot Figure 1: V-n diagram

        figure(fig1)
        ax = subplot(2,2,plot_idx); 
        hold on; grid on; box on;

        % Shaded achievable positive envelope
        fill([V_vn fliplr(V_vn)], [n_env zeros(size(n_env))], ...
             [1.0 0.9 0.1], ...
             'FaceAlpha',0.18, ...
             'EdgeColor','none');

        h1 = plot(V_vn, n_CL,     'b-', 'LineWidth',1.4);
        h2 = plot(V_vn, n_power,  'm-', 'LineWidth',1.4);
        h3 = plot(V_vn, n_struct, 'k-', 'LineWidth',1.4);
        h4 = plot(V_vn, n_aero,   '--', ...
                  'Color',[1.0 0.45 0.0], ...
                  'LineWidth',1.5);
        h5 = plot(V_vn, n_env,    'r-', 'LineWidth',2.2);

        % Speed markers
        xline(V_stall_1g, ':', '$V_{S,1g}$', ...
              'Interpreter','latex', ...
              'LabelVerticalAlignment','bottom');

        xline(V_corner, '--', '$V_A$', ...
              'Interpreter','latex', ...
              'LabelVerticalAlignment','bottom');

        xlabel('Velocity, $V$ [m/s]', 'Interpreter','latex')
        ylabel('Load Factor, $n$', 'Interpreter','latex')

        if h == 0
            h_label = 'Sea Level';
        else
            h_label = sprintf('h = %.0f m', h);
        end

        title(sprintf('%s — %s', cfg_names{cfg}, h_label), ...
              'Interpreter','none', ...
              'FontSize',10)

        legend([h1 h2 h3 h4 h5], ...
            {'$C_{L,max,clean}$', ...
             '$P_A = P_R$', ...
             '$n_{struct}$', ...
             '$E_{max}(T/W)$ reference', ...
             'Achievable envelope'}, ...
            'Interpreter','latex', ...
            'Location','eastoutside', ...
            'FontSize',7)

        % x-axis is limited by the PA = PR curve max for each subplot.
        xlim([0 V_PA_max - 1])
        ylim([0 ac.n_lim + 1.0])

        plot_idx = plot_idx + 1;

    end
end

figure(fig1)
sgtitle(sprintf(['Updated V-n / Sustained-Turn Envelopes from Constraint Analysis  |  ' ...
                 'CL_{max,clean}=%.1f, CD_{0,clean}=%.3f, e=%.2f, throttle=%.0f%%'], ...
                 CL_max_clean, CD_0_clean, e, 100*throttle_frac), ...
        'FontSize',11)

print(fig1, 'VN_Diagrams_updated.png', '-dpng', '-r1200')

%% Figure 2: Turn radius vs dynamic pressure q

fig2 = figure('Name','Turn Radius vs Dynamic Pressure', ...
              'Units','normalized','Position',[0.03 0.07 0.92 0.78]);

for idx = 1:case_count

    q        = turn_data(idx).q;
    q_PA_max = turn_data(idx).q_PA_max;

    r_struct = turn_data(idx).r_struct;
    r_CL     = turn_data(idx).r_CL;
    r_power  = turn_data(idx).r_power;
    r_env    = turn_data(idx).r_env;

    ax = subplot(2,2,idx);
    hold on; grid on; box on;

    % Automatically choose a clean y-limit by ignoring extremely large
    % values near n = 1.
    r_all = [r_CL(:); r_power(:); r_struct(:); r_env(:)];
    r_all = r_all(isfinite(r_all) & r_all > 0);

    if ~isempty(r_all)
        r_upper = prctile(r_all, 95);
    else
        r_upper = 1000;
    end

    % Highlight the feasible/minimum turn radius region in the same yellow
    % color as the V-n diagram. This is the region above the minimum
    % achievable radius curve and below the selected plot limit.
    r_env_fill = r_env;
    r_env_fill(~isfinite(r_env_fill)) = NaN;

    valid_fill = isfinite(r_env_fill) & q <= q_PA_max;

    if any(valid_fill)
        q_fill = q(valid_fill);
        r_fill = r_env_fill(valid_fill);

        fill([q_fill fliplr(q_fill)], ...
             [r_fill 1.15*r_upper*ones(size(r_fill))], ...
             [1.0 0.9 0.1], ...
             'FaceAlpha',0.18, ...
             'EdgeColor','none');
    end

    h1 = plot(q, r_CL,     'b-', 'LineWidth',1.4);
    h2 = plot(q, r_power,  'm-', 'LineWidth',1.4);
    h3 = plot(q, r_struct, 'k-', 'LineWidth',1.4);
    h4 = plot(q, r_env,    'r-', 'LineWidth',2.2);

    xlabel('Dynamic Pressure, $q$ [Pa]', 'Interpreter','latex')
    ylabel('Turn Radius, $r$ [m]', 'Interpreter','latex')

    if turn_data(idx).h == 0
        h_label = 'Sea Level';
    else
        h_label = sprintf('h = %.0f m', turn_data(idx).h);
    end

    title(sprintf('%s — %s', turn_data(idx).cfg_name, h_label), ...
          'Interpreter','none', ...
          'FontSize',10)

    legend([h1 h2 h3 h4], ...
        {'$C_{L,max,clean}$ radius', ...
         '$P_A = P_R$ radius', ...
         '$n_{struct}$ radius', ...
         'Minimum achievable radius'}, ...
        'Interpreter','latex', ...
        'Location','eastoutside', ...
        'FontSize',7)

    % Limit q-axis using the same PA = PR cutoff logic as the V-n plots.
    xlim([0 q_PA_max])

    if ~isempty(r_all)
        ylim([0 1.15*r_upper/3])
    end

end

figure(fig2)
sgtitle('Turn Radius vs Dynamic Pressure  |  Updated Constraint-Analysis Aircraft', ...
        'FontSize',11)

print(fig2, 'TurnRadius_vs_q_updated.png', '-dpng', '-r1200')

%% Local atmosphere function

function [T, rho, p, mu] = StdAtmos_local(h)
% StdAtmos_local  Simple ISA atmosphere through 6000 m.
% Returns temperature [K], density [kg/m^3], pressure [Pa], and dynamic
% viscosity [kg/(m*s)]. Vectorized for h in meters.

    h = max(h, 0);              % guard against negative altitudes

    R     = 287.05;             % gas constant for air [J/(kg*K)]
    g0    = 9.80665;            % standard gravity [m/s^2]
    T0    = 288.15;             % sea-level temperature [K]
    p0    = 101325;             % sea-level pressure [Pa]
    lapse = -0.0065;            % tropospheric lapse rate [K/m]

    T = T0 + lapse .* h;
    p = p0 .* (T ./ T0) .^ (-g0 / (lapse * R));
    rho = p ./ (R .* T);

    % Sutherland's law for dynamic viscosity
    mu0   = 1.716e-5;           % reference viscosity [kg/(m*s)]
    T_ref = 273.15;             % reference temperature [K]
    Suth  = 110.4;              % Sutherland constant [K]

    mu = mu0 .* (T ./ T_ref).^(3/2) .* ...
         (T_ref + Suth) ./ (T + Suth);
end