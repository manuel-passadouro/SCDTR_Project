% Close all figures
close all;

% Define original parameters
m_orig = -0.95;  % Original LDR characteristic slope
b_orig = 6.2;    % Original LDR characteristic intercept
VCC = 3.3;       % Supply voltage

% Example data (replace with actual measurements)
% Duty Cycle Voltage (V)
V_led = [0.00, 0.33, 0.66, 0.99, 1.32, 1.65, 1.98, 2.31, 2.64, 2.97, 3.30];
% LDR Resistance (Ω)
%#1 calibration readings
R_ldr = [1696250.00, 610454.56, 305000.00, 206666.66, 156463.41, 126956.53, 108011.52, 93670.89, 83068.18, 74958.51, 69360.47];
%#2 calibration readings
%R_ldr = [8180000.50, 988780.50, 416562.53, 261192.03, 190735.30, ...
 %        150588.23, 124262.29, 105677.97, 91612.90, 81610.73, 73231.71];

% Compute LUX values using the original parameters
LUX_orig = 10.^((log10(R_ldr) - b_orig) / m_orig);

% Perform linear regression (without log transformation)
p = polyfit(V_led, LUX_orig, 1); % Linear fit

% Extract slope and intercept
m_fit = p(1);
b_fit = p(2);

% Generate fitted values for plotting
LUX_fit = polyval(p, V_led);

% Compute R² (coefficient of determination)
SS_tot = sum((LUX_orig - mean(LUX_orig)).^2);  % Total sum of squares
SS_res = sum((LUX_orig - LUX_fit).^2);        % Residual sum of squares
R2 = 1 - (SS_res / SS_tot);                   % R² calculation

% Display regression results
fprintf('Linear Regression Results (Direct Fit):\n');
fprintf('Slope (m): %.4f\n', m_fit);
fprintf('Intercept (b): %.4f\n', b_fit);
fprintf('R² (coefficient of determination): %.4f\n', R2);

% Plot original data and fitted line
figure;
plot(V_led, LUX_orig, 'bo', 'MarkerSize', 8, 'DisplayName', 'Original Data');
hold on;
plot(V_led, LUX_fit, 'r-', 'LineWidth', 2, 'DisplayName', 'Fitted Line');
xlabel('LED Voltage (V)');
ylabel('LUX');
title('Linear Fit of LUX vs. LED Voltage');
legend('show');
grid on;
