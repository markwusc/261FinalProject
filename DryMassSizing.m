%% Conceptual Design Aircraft Weight Estimator
% Based on historical data of aircraft built over several years
% EQ's taken from Appendix I (eye) of:
% AIAA Education Series
% Fundamentals of Aircraft and Airship Design: Volume 1
% Leland M. Nicolai and Grant E. Carichner
% eISBN: 978-1-60086-753-8 print ISBN: 978-1-60086-751-4
% http://arc.aiaa.org/doi/book/10.2514/4.867538
clear; clc; close all;
%% Conversion constant
lb2N = 4.44822162; % Newtons per Pound
%% range of takeoff gross weights
TOGW_lb = logspace(3,7,1000); % values must be in Pounds for EQ's
TOGW_N = lb2N.*logspace(3,7,1000); % Convert for final results/plot
%% Empty Weight Estimates for Various Classes of Aircraft
% EQ's must use TOGW_lb; convert to N after calculation
%
% jet fighter aircraft (Figure I.1)
Ew_N.fighters = lb2N.*( (1.200).*TOGW_lb.^0.947 );
% bombers and transports (Figure I.2)
Ew_N.transports = lb2N.*( (0.911).*TOGW_lb.^0.947 );
% Intelligence, Surveillance, Reconnaissance (Figure I.5)
Ew_N.ISR = lb2N.*( (0.750).*TOGW_lb.^0.947 );
% Unmanned Air Vehicles (Figure I.6)
Ew_N.UAV = lb2N.*( (3.530).*TOGW_lb.^0.815 );
%% plot Empty Weight vs. Takeoff Gross Weight
plot_style = {'b:','r--','m-.','k-'};
fields = fieldnames(Ew_N); % get all fields (aircraft types)
for i = 1:length(fields) % for each field name
    loglog(TOGW_N, Ew_N.(fields{i}), plot_style{i}, 'LineWidth',2); hold on;
end
xlabel('\itTOGW\rm [N]','FontName','Times','FontSize',14)
ylabel('\itW_{empty}\rm [N]','FontName','Times','FontSize',14)
xlim([10^4 10^7]);
h = legend(fields,0); % create figure legen; locate in 'best' location
set(h,'FontName','Times','FontSize',11) % Increase legend font size
set(gcf,'color','w') % set figure color to white
set(gca,'FontSize',11) % increase FontSize for TickLabels