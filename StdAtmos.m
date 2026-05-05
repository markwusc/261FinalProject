%Benjamin Tallon
%AME 261 HW 1
%Standard Atmosphere function
% Benjamin Tallon
% AME 261 HW 1
% Question 2 Part A

%h is entered in [m]
function [P, rho, T, mu] = StdAtmos(h)
    % Constants
    g = 9.81; % m/s^2
    R = 287; % J/(kg*K)
    P_0 = 101325; % N/m^2
    T_0 = 288.15; % K
    L  = 0.0065; % K/m (ISA slope)
    h_11 = 11000; % m
    T_11 = 216.65; % K

    %Preallocation
    T = zeros(size(h));
    P = zeros(size(h));

    %Troposphere (h <= 11 km)
    tropo = h <= h_11;
    T(tropo) = T_0 - L*h(tropo);
    P(tropo) = P_0.*(T(tropo)/T_0).^(g/(R*L));

    %P at 11km
    P_11 = P_0*(T_11/T_0)^(g/(R*L));

    %Stratosphere (h > 11 km)
    strato = h > h_11;
    T(strato) = T_11;
    P(strato) = P_11.*exp(-g*(h(strato) - h_11)/(R*T_11));

    %Density
    rho = P./(R.*T);

    %Dynamic viscosity
    mu = 1.54*(1 + 0.0039*(T-250)) * 10e-5; % kg/(m*s)
end
