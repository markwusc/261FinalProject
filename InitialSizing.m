% Objective: Obtain all constants needed to size aircraft
% Abishek Iyer, abisheki@usc.edu, 4/4/26

clc; clear; close all

%% GIVEN CONSTANTS (METRIC)

% Engine / power
W_engine = 220;                 % kg
P_sl = 1540e3;                  % W
eta_p = 0.85;
P_available = eta_p * P_sl;     % W

% Wing
S  = 33; %22.3 m^2
b  = 18.6;  %13.7 m
AR = b^2 / S;

% Aero
Cd0    = 0.028;
CL_max = 0.8*3;
e      = 0.9;

% Aircraft
m_tot = 5612;                   % kg

%% STANDARD CONSTANTS

g      = 9.81;                  % m/s^2
rho    = 1.225;                 % kg/m^3
mu_sl  = 1.789e-5;              % Pa*s
gamma  = 1.4;
R      = 287;                   % J/(kg*K)
T_sl   = 288.15;                % K
a_sl   = sqrt(gamma*R*T_sl);    % m/s

%% DERIVED AIRCRAFT CONSTANTS

W = m_tot*g;                    % N
c_bar = S/b;                    % m
WS = W/S;                       % N/m^2

% Induced drag factor
k = 1/(pi*e*AR);

% FINITE WING LIFT CURVE SLOPE

a0 = 2*pi;                      % per rad
a  = a0/(1 + a0/(pi*e*AR));     % per rad

% CHARACTERISTIC CL VALUES

CL_Dmin  = sqrt(pi*e*AR*Cd0);       % same as CL at max L/D
CL_PRmin = sqrt(3*pi*e*AR*Cd0);

% CHARACTERISTIC CD VALUES

CD_Dmin  = 2*Cd0;
CD_PRmin = 4*Cd0;

% CHARACTERISTIC SPEEDS

V_Dmin  = sqrt(2*W/(rho*S*CL_Dmin));       % m/s
V_PRmin = sqrt(2*W/(rho*S*CL_PRmin));      % m/s
V_stall = sqrt(2*W/(rho*S*CL_max));        % m/s

%% L/D MAX AND (CL^(3/2)/CD)_MAX

LD_max = CL_Dmin / CD_Dmin;

CL32_over_CD_max = (CL_PRmin^(3/2)) / CD_PRmin;

%% MINIMUM POWER REQUIRED

P_required_min = sqrt((2*W^3)/(rho*S)) * (1 / CL32_over_CD_max);   % W

%% APPROXIMATE CRUISE SPEED FROM POWER AVAILABLE

V_cr = (2*P_available/(rho*Cd0*S))^(1/3);    % m/s

%% OPTIONAL CONVERSIONS

V_Dmin_kts   = V_Dmin*1.94384;
V_PRmin_kts  = V_PRmin*1.94384;
V_stall_kts  = V_stall*1.94384;
V_cr_kts     = V_cr*1.94384;

%% OTHER USEFUL TERMS

PW = P_available/W;              % W/N
WP = W/P_available;              % N/W

%% Requirements

% Ceiling

sigma_c = ((32*W^3 / (rho*S*(3*pi*e*AR*Cd0)^(3/2))) * (Cd0/(eta_p*P_sl))^2)^(1/3); %density ratio

rho_c = sigma_c * rho; %density at altitude
h_c = (T_sl/0.0065) * (1 - (rho_c/rho)^(1/4.2561)); %ceiling altitude

% Rate of climb

RoC = (P_available - P_required_min)/W;     % m/s
RoC_fpm = RoC * 196.850394;                 % ft/min

% TAKEOFF DISTANCE

mu_r = 0.02;                 % rolling friction coefficient, assumed
CL_TO = CL_max;              % using slide equation as written
CD_phi = CD_Dmin;            % first-pass drag estimate during ground roll

V_LO = 1.2 * V_stall;        % liftoff speed, m/s
V_avg = 0.7 * V_LO;          % average speed used in slide equation

q_avg = 0.5 * rho * V_avg^2; % dynamic pressure at 0.7*V_LO

L_avg = q_avg * S * CL_TO;   % lift during ground roll, N
D_avg = q_avg * S * CD_phi;  % drag during ground roll, N

T_avg = P_available / V_avg; % thrust estimate for prop aircraft, N

d_LO = (1.44 * W^2) / ...
    (rho * g * S * CL_TO * (T_avg - (D_avg + mu_r * (W - L_avg))));   % m

% Takeoff distance over 50 ft obstacle using RoC

h_obs = 50 * 0.3048;          % 50 ft obstacle height in m
V_climb = V_LO;               % first-pass assumption: climb speed ~ liftoff speed

gamma_climb = asin(RoC / V_climb);   % climb angle in rad
d_air = h_obs / tan(gamma_climb);    % airborne distance to clear obstacle, m

d_TO = d_LO + d_air;                 % total takeoff distance over 50 ft obstacle, m
d_TO_ft = d_TO / 0.3048;             % ft

% RANGE (PROP AIRCRAFT, CONSTANT h & V)

% SFC conversion:
SFC_p = 1.7e-7;   % N/W/s

% Fuel fraction assumption
zeta = 0.125;                 % Wfuel/W0 = 6.9/55

% Initial and final weights
W0 = W;
W1 = W0 * (1 - zeta);

% Choose the constant flight speed for the range calculation
V_range = (2*P_available/(0.909*Cd0*S))^(1/3);    % m/s;              % or use V_Dmin if you want

% Max aerodynamic efficiency
E_m = LD_max;

% Weight parameter for constant h,V range equation
W_D = 0.5 * rho * V_range^2 * S * CL_Dmin;

beta_0 = atan(W0 / W_D);
beta_1 = atan(W1 / W_D);

X_hV = (2 * eta_p * E_m / SFC_p) * (beta_0 - beta_1);   % m

X_hV_km  = X_hV / 1000;
X_hV_nmi = X_hV / 1852;

%% LANDING DISTANCE OVER 50 FT OBSTACLE

% Assumptions
mu_brake = 0.40;                  % braking friction coefficient, assumed
CL_land  = CL_max;                % landing lift coefficient, first-pass assumption
CD_land  = Cd0 + k*CL_land^2;     % landing drag coefficient
LD_land  = CL_land / CD_land;     % landing lift-to-drag ratio

% Landing speeds
V_F = 1.35 * V_stall;             % flare speed, m/s
V_T = 1.30 * V_stall;             % touchdown speed, m/s
V_avg_land = 0.70 * V_T;          % average speed during ground roll, m/s

% Reverse thrust
T_avail_land = P_available / V_avg_land;    % thrust available during landing, N
T_rev = 0.25 * T_avail_land;                % reverse thrust, N

% 1) Glide distance from 50 ft obstacle
% d_gl = 15*(L/D)
d_gl_land = 15 * LD_land;                    % m

% 2) Airborne deceleration distance
% d_decel = (L/D)*(V_F^2 - V_T^2)/(2g)
d_decel_land = LD_land * (V_F^2 - V_T^2) / (2*g);   % m

% Total airborne landing distance
d_air_land = d_gl_land + d_decel_land;      % m

% 3) Ground roll
q_land = 0.5 * rho * V_avg_land^2;          % dynamic pressure, Pa
L_land = q_land * S * CL_land;              % lift during rollout, N
D_land = q_land * S * CD_land;              % drag during rollout, N

% Ground-roll equation from slide:
% d_G = 1.69*W^2 / [rho*g*S*CLmax*(T_rev + D + mu_brake*(W-L))]
d_G = (1.69 * W^2) / ...
    (rho * g * S * CL_land * (T_rev + D_land + mu_brake * (W - L_land)));   % m

% Total landing distance over 50 ft obstacle
d_L = d_air_land + d_G;             % m
d_L_ft = d_L / 0.3048;              % ft

%% =========================
%  PRINTED RESULTS SUMMARY
%  =========================

ft_per_m = 3.28084;
kts_per_ms = 1.94384;

fprintf('\n');
fprintf('============================================================\n');
fprintf('               AIRCRAFT SIZING / REQUIREMENTS               \n');
fprintf('============================================================\n');

%% 1) INPUT SUMMARY
fprintf('\n[1] INPUT SUMMARY\n');
fprintf('------------------------------------------------------------\n');
fprintf('Mass, m_tot                 = %8.2f kg\n', m_tot);
fprintf('Weight, W                   = %8.2f N\n', W);
fprintf('Wing area, S                = %8.2f m^2\n', S);
fprintf('Wingspan, b                 = %8.2f m\n', b);
fprintf('Aspect ratio, AR            = %8.3f\n', AR);
fprintf('Cd0                         = %8.4f\n', Cd0);
fprintf('CL_max                      = %8.4f\n', CL_max);
fprintf('Oswald efficiency, e        = %8.4f\n', e);
fprintf('Sea-level power, P_sl       = %8.2f W\n', P_sl);
fprintf('Prop efficiency, eta_p      = %8.3f\n', eta_p);
fprintf('Power available             = %8.2f W\n', P_available);

%% 2) DERIVED AERODYNAMIC CONSTANTS
fprintf('\n[2] DERIVED AERODYNAMIC CONSTANTS\n');
fprintf('------------------------------------------------------------\n');
fprintf('Mean chord, c_bar           = %8.3f m\n', c_bar);
fprintf('Wing loading, W/S           = %8.3f N/m^2\n', WS);
fprintf('Induced drag factor, k      = %8.5f\n', k);
fprintf('Lift-curve slope, a         = %8.4f per rad\n', a);
fprintf('CL at max L/D               = %8.4f\n', CL_Dmin);
fprintf('CL at min power req         = %8.4f\n', CL_PRmin);
fprintf('CD at max L/D               = %8.4f\n', CD_Dmin);
fprintf('CD at min power req         = %8.4f\n', CD_PRmin);
fprintf('Max L/D                     = %8.3f\n', LD_max);
fprintf('(CL^(3/2)/CD)_max           = %8.3f\n', CL32_over_CD_max);

%% 3) CHARACTERISTIC SPEEDS
fprintf('\n[3] CHARACTERISTIC SPEEDS\n');
fprintf('------------------------------------------------------------\n');
fprintf('V_Dmin                      = %8.3f m/s   (%8.3f kt)\n', V_Dmin, V_Dmin_kts);
fprintf('V_PRmin                     = %8.3f m/s   (%8.3f kt)\n', V_PRmin, V_PRmin_kts);
fprintf('V_stall                     = %8.3f m/s   (%8.3f kt)\n', V_stall, V_stall_kts);
fprintf('V_cr (approx)               = %8.3f m/s   (%8.3f kt)\n', V_cr, V_cr_kts);

%% 4) POWER / PERFORMANCE
fprintf('\n[4] POWER / PERFORMANCE\n');
fprintf('------------------------------------------------------------\n');
fprintf('Minimum power required      = %8.2f W\n', P_required_min);
fprintf('Power-to-weight, P/W        = %8.4f W/N\n', PW);
fprintf('Weight-to-power, W/P        = %8.6f N/W\n', WP);
fprintf('Rate of climb               = %8.3f m/s   (%8.3f ft/min)\n', RoC, RoC_fpm);

%% 5) CEILING REQUIREMENT
fprintf('\n[5] CEILING REQUIREMENT\n');
fprintf('------------------------------------------------------------\n');
fprintf('Density ratio at ceiling    = %8.4f\n', sigma_c);
fprintf('Density at ceiling          = %8.4f kg/m^3\n', rho_c);
fprintf('Ceiling altitude, h_c       = %8.2f m    (%8.2f ft)\n', h_c, h_c*ft_per_m);

if h_c*ft_per_m >= 18000
    fprintf('Requirement status          = PASS (h_c >= 18000 ft)\n');
else
    fprintf('Requirement status          = FAIL (h_c < 18000 ft)\n');
end

%% 6) ROC REQUIREMENT
fprintf('\n[6] RATE OF CLIMB REQUIREMENT\n');
fprintf('------------------------------------------------------------\n');
fprintf('Computed ROC                = %8.2f ft/min\n', RoC_fpm);

if RoC_fpm >= 800
    fprintf('Requirement status          = PASS (ROC >= 800 ft/min)\n');
else
    fprintf('Requirement status          = FAIL (ROC < 800 ft/min)\n');
end

%% 7) TAKEOFF PERFORMANCE
fprintf('\n[7] TAKEOFF PERFORMANCE\n');
fprintf('------------------------------------------------------------\n');
fprintf('Rolling friction, mu_r      = %8.3f\n', mu_r);
fprintf('CL_TO                       = %8.3f\n', CL_TO);
fprintf('CD_phi                      = %8.3f\n', CD_phi);
fprintf('Liftoff speed, V_LO         = %8.3f m/s   (%8.3f kt)\n', V_LO, V_LO*kts_per_ms);
fprintf('Avg TO speed                = %8.3f m/s\n', V_avg);
fprintf('Avg dynamic pressure        = %8.3f Pa\n', q_avg);
fprintf('Avg lift                    = %8.3f N\n', L_avg);
fprintf('Avg drag                    = %8.3f N\n', D_avg);
fprintf('Avg thrust available        = %8.3f N\n', T_avg);
fprintf('Ground roll, d_LO           = %8.3f m    (%8.3f ft)\n', d_LO, d_LO*ft_per_m);
fprintf('Airborne distance, d_air    = %8.3f m    (%8.3f ft)\n', d_air, d_air*ft_per_m);
fprintf('Total TO distance, d_TO     = %8.3f m    (%8.3f ft)\n', d_TO, d_TO_ft);

if d_TO_ft <= 500
    fprintf('Requirement status          = PASS (d_TO <= 500 ft)\n');
else
    fprintf('Requirement status          = FAIL (d_TO > 500 ft)\n');
end

%% 8) LANDING PERFORMANCE
fprintf('\n[8] LANDING PERFORMANCE\n');
fprintf('------------------------------------------------------------\n');
fprintf('Braking friction, mu_brake  = %8.3f\n', mu_brake);
fprintf('CL_land                     = %8.3f\n', CL_land);
fprintf('CD_land                     = %8.3f\n', CD_land);
fprintf('L/D_land                    = %8.3f\n', LD_land);
fprintf('Flare speed, V_F            = %8.3f m/s   (%8.3f kt)\n', V_F, V_F*kts_per_ms);
fprintf('Touchdown speed, V_T        = %8.3f m/s   (%8.3f kt)\n', V_T, V_T*kts_per_ms);
fprintf('Avg landing speed           = %8.3f m/s\n', V_avg_land);
fprintf('Landing thrust avail        = %8.3f N\n', T_avail_land);
fprintf('Reverse thrust, T_rev       = %8.3f N\n', T_rev);
fprintf('Glide distance              = %8.3f m    (%8.3f ft)\n', d_gl_land, d_gl_land*ft_per_m);
fprintf('Airborne decel distance     = %8.3f m    (%8.3f ft)\n', d_decel_land, d_decel_land*ft_per_m);
fprintf('Total airborne landing      = %8.3f m    (%8.3f ft)\n', d_air_land, d_air_land*ft_per_m);
fprintf('Ground roll, d_G            = %8.3f m    (%8.3f ft)\n', d_G, d_G*ft_per_m);
fprintf('Total landing distance      = %8.3f m    (%8.3f ft)\n', d_L, d_L_ft);

if d_L_ft <= 500
    fprintf('Requirement status          = PASS (d_L <= 500 ft)\n');
else
    fprintf('Requirement status          = FAIL (d_L > 500 ft)\n');
end

%% 9) RANGE
fprintf('\n[9] RANGE PERFORMANCE\n');
fprintf('------------------------------------------------------------\n');
fprintf('SFC_p                       = %8.4e N/W/s\n', SFC_p);
fprintf('Fuel fraction, zeta         = %8.4f\n', zeta);
fprintf('Initial weight, W0          = %8.3f N\n', W0);
fprintf('Final weight, W1            = %8.3f N\n', W1);
fprintf('Range speed, V_range        = %8.3f m/s   (%8.3f kt)\n', V_range, V_range*kts_per_ms);
fprintf('Weight parameter, W_D       = %8.3f N\n', W_D);
fprintf('Range, X_hV                 = %8.3f km    (%8.3f nmi)\n', X_hV_km, X_hV_nmi);

if X_hV_nmi >= 700
    fprintf('Requirement status          = PASS (X_hV_nmi >= 700 nmi)\n');
else
    fprintf('Requirement status          = FAIL (X_hV_nmi < 700 nmi)\n');
end

%% 10) FINAL REQUIREMENTS SUMMARY
fprintf('\n[10] FINAL REQUIREMENTS SUMMARY\n');
fprintf('------------------------------------------------------------\n');

if h_c*ft_per_m >= 18000
    ceil_status = 'PASS';
else
    ceil_status = 'FAIL';
end

if RoC_fpm >= 800
    roc_status = 'PASS';
else
    roc_status = 'FAIL';
end

if d_TO_ft <= 500
    to_status = 'PASS';
else
    to_status = 'FAIL';
end

if d_L_ft <= 500
    land_status = 'PASS';
else
    land_status = 'FAIL';
end

if X_hV_nmi >= 700
    range_status = 'PASS';
else
    range_status = 'FAIL';
end

fprintf('Ceiling >= 18000 ft         : %s\n', ceil_status);
fprintf('ROC >= 800 ft/min           : %s\n', roc_status);
fprintf('Takeoff <= 500 ft           : %s\n', to_status);
fprintf('Landing <= 500 ft           : %s\n', land_status);
fprintf('Range >= 700 nmi            : %s\n', range_status);

fprintf('============================================================\n');
fprintf('                    END OF SCRIPT OUTPUT                    \n');
fprintf('============================================================\n\n');