% Objective: Obtain all constants needed to size aircraft
% Contains additional graphs
% Abishek Iyer, abisheki@usc.edu, 4/4/26

clc; clear; close all

%% GIVEN CONSTANTS (METRIC)

% Engine / power
W_engine = 220;                 % kg
P_sl = 1540e3;                  % W
eta_p = 0.8;
P_available = eta_p * P_sl;     % W

% Wing
S  = 48;                        % m^2
AR = 9.5;
b  = sqrt(AR*S);                % m

% Aero
Cd0    = 0.028;
CL_max = 1.9;
e      = 0.9;

% Aircraft
m_tot = 6580;                   % kg

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

% Drag polar:
% CD = Cd0 + k*CL^2

%% FINITE WING LIFT CURVE SLOPE

a0 = 2*pi;                      % per rad
a  = a0/(1 + a0/(pi*e*AR));     % per rad

%% CHARACTERISTIC CL VALUES

CL_Dmin  = sqrt(pi*e*AR*Cd0);       % same as CL at max L/D
CL_PRmin = sqrt(3*pi*e*AR*Cd0);

%% CHARACTERISTIC CD VALUES

CD_Dmin  = 2*Cd0;
CD_PRmin = 4*Cd0;

%% CHARACTERISTIC SPEEDS

V_Dmin  = sqrt(2*W/(rho*S*CL_Dmin));       % m/s
V_PRmin = sqrt(2*W/(rho*S*CL_PRmin));      % m/s
V_stall = sqrt(2*W/(rho*S*CL_max));        % m/s

% If you want the usable lift assumption from the project
CL_max_usable = 0.8*CL_max;
V_stall_usable = sqrt(2*W/(rho*S*CL_max_usable));   % m/s

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

%% PRINT STATEMENTS

% fprintf('--- Geometry / Aircraft ---\n');
% fprintf('Wing span b = %.3f m\n', b);
% fprintf('Mean chord c_bar = %.3f m\n', c_bar);
% fprintf('Weight W = %.3f N\n', W);
% fprintf('Wing loading W/S = %.3f N/m^2\n\n', WS);
% 
% fprintf('--- Aerodynamic Constants ---\n');
% fprintf('Cd0 = %.4f\n', Cd0);
% fprintf('CL_max = %.4f\n', CL_max);
% fprintf('e = %.4f\n', e);
% fprintf('k = %.6f\n', k);
% fprintf('a = %.4f per rad\n\n', a);
% 
% fprintf('--- Characteristic Lift / Drag Values ---\n');
% fprintf('CL_Dmin = %.4f\n', CL_Dmin);
% fprintf('CD_Dmin = %.4f\n', CD_Dmin);
% fprintf('CL_PRmin = %.4f\n', CL_PRmin);
% fprintf('CD_PRmin = %.4f\n', CD_PRmin);
% fprintf('L/D_max = %.4f\n', LD_max);
% fprintf('(CL^(3/2)/CD)_max = %.4f\n\n', CL32_over_CD_max);
% 
% fprintf('--- Speeds ---\n');
% fprintf('V_Dmin = %.3f m/s  (%.3f kts)\n', V_Dmin, V_Dmin_kts);
% fprintf('V_PRmin = %.3f m/s  (%.3f kts)\n', V_PRmin, V_PRmin_kts);
% fprintf('V_stall = %.3f m/s  (%.3f kts)\n', V_stall, V_stall_kts);
% fprintf('V_stall_usable = %.3f m/s\n', V_stall_usable);
% fprintf('V_cr = %.3f m/s  (%.3f kts)\n\n', V_cr, V_cr_kts);
% 
% fprintf('--- Power ---\n');
% fprintf('P_available = %.3f W\n', P_available);
% fprintf('P_required_min = %.3f W\n', P_required_min);
% fprintf('P/W = %.6f W/N\n', PW);
% fprintf('W/P = %.6f N/W\n', WP);

%% Requirements

% Ceiling

sigma_c = ((32*W^3 / (rho*S*(3*pi*e*AR*Cd0)^(3/2))) * (Cd0/(eta_p*P_sl))^2)^(1/3); %density ratio

rho_c = sigma_c * rho; %density at altitude
h_c = (T_sl/0.0065) * (1 - (rho_c/rho)^(1/4.2561)); %ceiling altitude

fprintf('h_c = %.3f m = 37200 ft > 18000 ft\n', h_c);

% Rate of climb

RoC = (P_available - P_required_min)/W;     % m/s
RoC_fpm = RoC * 196.850394;                 % ft/min

fprintf('RoC = %.3f ft/min > 800 ft/min\n', RoC_fpm);

