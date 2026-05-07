% VN_Diagrams_single_engine.m
% =========================================================================
% V-n / sustained-turn envelope for single-engine configuration
%
% Figures produced:
%   Figure 1: V-n / sustained-turn envelopes at sea level and 4572 m
%   Figure 2: Turn radius vs dynamic pressure q at sea level and 4572 m
%
% Figure 2 includes:
%   - CLmax turn-radius limit
%   - T_max / power-available turn-radius limit
%   - n_struct turn-radius limit
%   - minimum achievable turn-radius curve
%   - yellow highlighted feasible turn-radius region
%
% StdAtmos is included as a local function, so this file can run standalone.
% =========================================================================

clear; clc; close all;

%% Shared parameters

% Physical constants
g       = 9.81;          % gravitational acceleration [m/s^2]
rho_SL  = 1.225;         % sea-level ISA density [kg/m^3]
shp2W   = 745.7;         % shaft horsepower to watts

% Propulsion
P_shp    = 1050;         % rated shaft power [hp]
eta_prop = 0.80;         % propeller efficiency [-]

% Clean aerodynamic values
CD_0_clean   = 0.030;    % clean zero-lift drag coefficient [-]
CL_max_clean = 1.8;      % clean maximum lift coefficient [-]
e            = 0.90;     % Oswald efficiency factor [-]

% Single-engine aircraft parameters
W_TO   = 39867.9;        % MTOW [N]
S      = 39.849246231155774;   % wing area [m^2]
AR     = 6.854271356783920;    % aspect ratio [-]
b      = sqrt(S * AR);         % wingspan [m]

% Propulsive power available at sea level
P_A_SL = eta_prop * P_shp * shp2W;  % [W]

cfg_name = 'Single Engine (42 kN, 1 x 1050 shp)';

% Structural limit
% DHC-6 / Normal Category positive limit load factor
n_struct_pos = 3.8;      % positive limit load factor [-]

% Sustained-turn power setting
throttle_frac = 1.00;

%% Altitudes and velocity range

% Sea level and 4572 m = 15,000 ft
h_cases = [0, 4572];                     % [m]
[~, rho_cases, ~, ~] = StdAtmos_local(h_cases);

V_min = 1.0;                             % avoid division by zero [m/s]
V_max_calc = 250;                        % [m/s]
V_vn = linspace(V_min, V_max_calc, 1200);

%% Aircraft structure

ac.S      = S;
ac.AR     = AR;
ac.b      = b;
ac.W      = W_TO;
ac.P_A_SL = P_A_SL;
ac.CD_0   = CD_0_clean;
ac.CLmax  = CL_max_clean;
ac.e      = e;
ac.k      = 1 / (pi * ac.e * ac.AR);
ac.n_lim  = n_struct_pos;

%% Storage arrays for turn-radius figure

turn_data = struct();

%% Console summary

fprintf('=== V-n Diagram Inputs: Single-Engine Configuration ===\n')
fprintf('  %s\n', cfg_name)
fprintf('    S    = %.2f m^2\n', ac.S)
fprintf('    AR   = %.2f\n', ac.AR)
fprintf('    b    = %.2f m\n', ac.b)
fprintf('    W    = %.1f N  (%.0f kg)\n', ac.W, ac.W/g)
fprintf('    P_A  = %.1f kW\n\n', ac.P_A_SL/1000)

%% Figure 1: V-n diagrams

fig1 = figure('Name','Single-Engine V-n Diagrams', ...
              'Units','normalized','Position',[0.05 0.12 0.88 0.52]);

for h_idx = 1:numel(h_cases)

    h   = h_cases(h_idx);
    rho = rho_cases(h_idx);

    q = 0.5 .* rho .* V_vn.^2;

    % Power available with density lapse
    P_A_h = throttle_frac * ac.P_A_SL * (rho / rho_SL);

    % Lift-limited load factor
    n_CL = q .* ac.S .* ac.CLmax ./ ac.W;

    % Structural limit
    n_struct = ac.n_lim .* ones(size(V_vn));

    % Sustained power limit
    % D = q S CD0 + k(nW)^2/(qS)
    % P_R = D V
    % Solve P_A = P_R for n
    n_power_sq = (((P_A_h ./ V_vn) - ac.CD_0 .* q .* ac.S) .* ...
                   (q .* ac.S)) ./ (ac.k .* ac.W.^2);

    n_power = sqrt(n_power_sq);
    n_power(n_power_sq <= 0) = NaN;

    % Find maximum velocity where PA = PR curve exists
    valid_power_idx = find(isfinite(n_power));

    if ~isempty(valid_power_idx)
        V_PA_max = V_vn(valid_power_idx(end));
    else
        V_PA_max = V_max_calc;
    end

    q_PA_max = 0.5 * rho * V_PA_max^2;

    % Aerodynamic reference limit based on E_max and equivalent thrust
    E_max = 1 / (2 * sqrt(ac.CD_0 * ac.k));
    T_equiv = P_A_h ./ V_vn;
    n_aero = E_max .* (T_equiv ./ ac.W);
    n_aero(n_aero < 0) = NaN;

    % Achievable positive envelope
    n_mat = [n_CL; n_struct; n_power; n_aero];
    n_env = min(n_mat, [], 1, 'omitnan');
    n_env(isnan(n_env)) = 0;
    n_env = max(n_env, 0);

    % Important speeds
    V_stall_1g = sqrt(2 * ac.W / (rho * ac.S * ac.CLmax));
    V_corner   = sqrt(2 * ac.W * ac.n_lim / ...
                      (rho * ac.S * ac.CLmax));

    %% Turn-radius calculations for Figure 2

    % Coordinated level turn radius:
    % r = V^2 / (g*sqrt(n^2 - 1))

    r_struct = V_vn.^2 ./ (g .* sqrt(n_struct.^2 - 1));
    r_CL     = V_vn.^2 ./ (g .* sqrt(n_CL.^2     - 1));
    r_power  = V_vn.^2 ./ (g .* sqrt(n_power.^2  - 1));
    r_env    = V_vn.^2 ./ (g .* sqrt(n_env.^2    - 1));

    % Remove invalid points where steady level turn is impossible
    r_CL(n_CL <= 1)         = NaN;
    r_power(n_power <= 1)   = NaN;
    r_env(n_env <= 1)       = NaN;

    r_struct(~isfinite(r_struct)) = NaN;
    r_CL(~isfinite(r_CL))         = NaN;
    r_power(~isfinite(r_power))   = NaN;
    r_env(~isfinite(r_env))       = NaN;

    % Remove points beyond PA = PR cutoff
    r_CL(q > q_PA_max)     = NaN;
    r_power(q > q_PA_max)  = NaN;
    r_struct(q > q_PA_max) = NaN;
    r_env(q > q_PA_max)    = NaN;

    % Store data for Figure 2
    turn_data(h_idx).h          = h;
    turn_data(h_idx).rho        = rho;
    turn_data(h_idx).q          = q;
    turn_data(h_idx).q_PA_max   = q_PA_max;
    turn_data(h_idx).r_struct   = r_struct;
    turn_data(h_idx).r_CL       = r_CL;
    turn_data(h_idx).r_power    = r_power;
    turn_data(h_idx).r_env      = r_env;

    %% Plot Figure 1

    figure(fig1)
    subplot(1,2,h_idx);
    hold on; grid on; box on;

    % Yellow achievable region
    fill([V_vn fliplr(V_vn)], [n_env zeros(size(n_env))], ...
         [1.0 0.9 0.1], ...
         'FaceAlpha',0.18, ...
         'EdgeColor','none', ...
         'HandleVisibility','off');

    h1 = plot(V_vn, n_CL,     'b-', 'LineWidth',1.4);
    h2 = plot(V_vn, n_power,  'm-', 'LineWidth',1.4);
    h3 = plot(V_vn, n_struct, 'k-', 'LineWidth',1.4);
    h4 = plot(V_vn, n_aero,   '--', ...
              'Color',[1.0 0.45 0.0], ...
              'LineWidth',1.5);
    h5 = plot(V_vn, n_env,    'r-', 'LineWidth',2.2);

    xline(V_stall_1g, ':', '$V_{S,1g}$', ...
          'Interpreter','latex', ...
          'LabelVerticalAlignment','bottom');

    xline(V_corner, '--', '$V_A$', ...
          'Interpreter','latex', ...
          'LabelVerticalAlignment','bottom');

    xlabel('Velocity, $V$ [m/s]', 'Interpreter','latex')
    ylabel('Load Factor, $n$', 'Interpreter','latex')

    if h == 0
        h_label = 'a) Sea Level';
    else
        h_label = sprintf('b) h = %.0f m', h);
    end

    title(h_label, ...
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

    xlim([0 V_PA_max - 1])
    ylim([0 ac.n_lim + 1.0])

end

figure(fig1)


print(fig1, 'VN_Diagrams_single_engine.png', '-dpng', '-r1200')

%% Figure 2: Turn radius vs dynamic pressure q
% Shows sea level and 4572 m for the single-engine aircraft.
%
% Colors:
%   Blue    = CLmax turn-radius limit
%   Magenta = T_max / PA = PR turn-radius limit
%   Black   = n_struct turn-radius limit
%   Red     = minimum achievable turn radius
%
% Line style:
%   Solid  = sea level
%   Dashed = 4572 m

fig2 = figure('Name','Single-Engine Turn Radius vs Dynamic Pressure', ...
              'Units','normalized','Position',[0.08 0.12 0.78 0.58]);

hold on; grid on; box on;

% Pull sea-level data
q_sl        = turn_data(1).q;
qmax_sl     = turn_data(1).q_PA_max;
r_CL_sl     = turn_data(1).r_CL;
r_power_sl  = turn_data(1).r_power;
r_struct_sl = turn_data(1).r_struct;
r_env_sl    = turn_data(1).r_env;

% Pull 4572 m data
q_4572        = turn_data(2).q;
qmax_4572     = turn_data(2).q_PA_max;
r_CL_4572     = turn_data(2).r_CL;
r_power_4572  = turn_data(2).r_power;
r_struct_4572 = turn_data(2).r_struct;
r_env_4572    = turn_data(2).r_env;

% Valid minimum-radius points for highlighting
valid_env_sl   = isfinite(r_env_sl)   & q_sl   <= qmax_sl;
valid_env_4572 = isfinite(r_env_4572) & q_4572 <= qmax_4572;

% Determine q and r limits
q_plot_max = max([qmax_sl, qmax_4572]);

r_all = [ ...
    r_CL_sl(:); r_power_sl(:); r_struct_sl(:); r_env_sl(:); ...
    r_CL_4572(:); r_power_4572(:); r_struct_4572(:); r_env_4572(:)];

r_all = r_all(isfinite(r_all) & r_all > 0);

if isempty(r_all)
    r_upper = 1000;
else
    r_upper = 1.15 * prctile(r_all, 95);
end

% Highlight feasible region above minimum achievable turn-radius curve
% Sea level
if any(valid_env_sl)
    q_fill = q_sl(valid_env_sl);
    r_fill = r_env_sl(valid_env_sl);

    fill([q_fill fliplr(q_fill)], ...
         [r_fill r_upper*ones(size(r_fill))], ...
         [1.0 0.9 0.1], ...
         'FaceAlpha',0.14, ...
         'EdgeColor','none', ...
         'HandleVisibility','off');
end

% 4572 m
if any(valid_env_4572)
    q_fill = q_4572(valid_env_4572);
    r_fill = r_env_4572(valid_env_4572);

    fill([q_fill fliplr(q_fill)], ...
         [r_fill r_upper*ones(size(r_fill))], ...
         [1.0 0.9 0.1], ...
         'FaceAlpha',0.07, ...
         'EdgeColor','none', ...
         'HandleVisibility','off');
end

% Sea-level curves: solid
h1 = plot(q_sl, r_CL_sl,     'b-', 'LineWidth',1.5);
h2 = plot(q_sl, r_power_sl,  'm-', 'LineWidth',1.5);
h3 = plot(q_sl, r_struct_sl, 'k-', 'LineWidth',1.5);
h4 = plot(q_sl, r_env_sl,    'r-', 'LineWidth',2.4);

% 4572 m minimum-radius curve: dashed
h5 = plot(q_4572, r_env_4572, 'b--', 'LineWidth',2.4);

xlabel('Dynamic Pressure, $q$ [Pa]', 'Interpreter','latex')
ylabel('Turn Radius, $r$ [m]', 'Interpreter','latex')



xlim([0 q_plot_max])
ylim([0 r_upper])

legend([h1 h2 h3 h4 h5], ...
    '$C_{L,max}$ SL', ...
    '$T_{max}$ / $P_A=P_R$ SL', ...
    '$n_{struct}$ SL', ...
    'Minimum radius SL', ...
    'Minimum radius at 4572 m', ...
    'Interpreter','latex', ...
    'Location','eastoutside', ...
    'FontSize',8)



print(fig2, 'TurnRadius_vs_q_single_engine.png', '-dpng', '-r1200')

%% Local atmosphere function

function [T, rho, p, mu] = StdAtmos_local(h)
% StdAtmos_local  Simple ISA atmosphere through 4572 m.
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