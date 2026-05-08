clc; clear; close all;

filename = 'gaw1polar.txt';

fid = fopen(filename,'r');
lines = {};
tline = fgetl(fid);
while ischar(tline)
    lines{end+1} = tline; 
    tline = fgetl(fid);
end
fclose(fid);

data = [];
for i = 1:length(lines)
    nums = sscanf(lines{i}, '%f');
    if numel(nums) == 7
        data = [data; nums'];
    end
end

%% Extract columns 
alpha = data(:,1);
CL    = data(:,2);

[alpha, idx] = sort(alpha);
CL = CL(idx);

[alpha, idxu] = unique(alpha);
CL = CL(idxu);

%% ---- Find CLmax ----
[clmax, idxmax] = max(CL);
alpha_stall = alpha(idxmax);

alpha_smooth = linspace(min(alpha), max(alpha), 300);
CL_smooth = interp1(alpha, CL, alpha_smooth, 'pchip');

%% Plot 
figure('Color','w');
plot(alpha_smooth, CL_smooth, 'LineWidth', 2); hold on
plot(alpha, CL, 'o', 'MarkerSize', 5)
plot(alpha_stall, clmax, 'ro', 'MarkerSize', 9, 'LineWidth', 2)

xlabel('\alpha (deg)', 'FontSize', 14)
ylabel('C_L', 'FontSize', 14)
title('Lift Curve for GA(W)-1 Airfoil', 'FontSize', 16)

grid on
grid minor
set(gca,'FontSize',12,'LineWidth',1.2)
xlim([min(alpha) max(alpha)])
ylim([min(CL)*0.95 max(CL)*1.05])

text(alpha_stall+0.4, clmax, ...
    sprintf('C_{L,max} = %.3f\n\\alpha = %.1f^\\circ', clmax, alpha_stall), ...
    'FontSize', 12, 'Interpreter', 'tex')

legend('Smooth Curve','Data Points','C_{L,max}','Location','southeast')
box on

fprintf('CL_max = %.4f\n', clmax);
fprintf('Stall angle = %.2f deg\n', alpha_stall);