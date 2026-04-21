%Discussion 13

clc; clear; close all;

% dash8 masses: [m (kg), x (m), y (m), z (m)]
dash8_masses = [
    6000   16      0       0;        % Fuselage
    1500   14     -4.5    -1;        % Left Wing
    1500   14      4.5    -1;        % Right Wing
    1000   13.5   -4      -1.5;      % Left Engine
    1000   13.5    4      -1.5;      % Right Engine
    1000   31      0       3.5;      % Tail
    5000   15      0.2     0.5;      % Payload
    1500   12      0      -0.5;      % Fwd Fuel Tank (Row 8)
    1500   18      0      -0.5       % Aft Fuel Tank (Row 9)
];

% Function to compute CG
computeCG = @(data) sum(data(:,1).*data(:,2:4)) / sum(data(:,1));

% 1) Baseline
m_tot = sum(dash8_masses(:,1));
r_CG  = computeCG(dash8_masses);

fprintf('Baseline Total Mass: %.2f kg\n', m_tot);
fprintf('Baseline CG: [%.4f, %.4f, %.4f] m\n\n', r_CG);

% 2) Forward tank empty
data_fwd = dash8_masses;
data_fwd(8,1) = 0;   % remove forward tank mass

r_fwd = computeCG(data_fwd);

% 3) Aft tank empty
data_aft = dash8_masses;
data_aft(9,1) = 0;   % remove aft tank mass

r_aft = computeCG(data_aft);

% CG shift
delta_r = r_aft - r_fwd;
delta_mag = norm(delta_r);

fprintf('CG (Forward tank empty): [%.4f, %.4f, %.4f] m\n', r_fwd);
fprintf('CG (Aft tank empty):     [%.4f, %.4f, %.4f] m\n\n', r_aft);

fprintf('CG Shift Vector: [%.4f, %.4f, %.4f] m\n', delta_r);
fprintf('CG Shift Magnitude: %.4f m\n', delta_mag);