close all;

% Define original parameters
m_orig = -0.8;  % Original LDR characteristic slope
b_min = 5.98; % Original LDR characteristic intercept

b_nom = 6.15; % Original LDR characteristic intercept

b_max = 6.28; % Original LDR characteristic intercept

%Ajust m and b do fit aquired data
m_new = -0.95;
b_new = 6.2;

VCC = 3.3; % Supply voltage

% Example data (replace with actual measurements)
% Duty Cycle Voltage (V)
V_led = [0.00, 0.33, 0.66, 0.99, 1.32, 1.65, 1.98, 2.31, 2.64, 2.97, 3.30];
%#1 calibration readings
R_ldr = [1696250.00, 861276.56, 483373.50, 328429.75, 250828.03, 200000.00, 168043.45, 143370.80, 126046.51, 112972.98, 101580.38];

%R_ldr = [1696250.00, 610454.56, 305000.00, 206666.66, 156463.41, 126956.53, 108011.52, 93670.89, 83068.18, 74958.51, 69360.47];
%#2 calibration readings
%R_ldr = [8180000.50, 988780.50, 416562.53, 261192.03, 190735.30, ...
 %        150588.23, 124262.29, 105677.97, 91612.90, 81610.73, 73231.71];

% Compute LUX values using the original parameters
LUX_b_min = 10.^((log10(R_ldr) - b_min) / m_orig);

LUX_b_nom = 10.^((log10(R_ldr) - b_nom) / m_orig);

LUX_b_max = 10.^((log10(R_ldr) - b_max) / m_orig);

% Compute LUX values using the new parameters
LUX_new = 10.^((log10(R_ldr) - b_new) / m_new);

% Plot results: Original LUX vs. V_LED
figure;
%plot(V_led, LUX_b_min, '-o', 'LineWidth', 2, 'DisplayName', 'Original LUX');
hold on;
plot(V_led, LUX_b_nom, '-o', 'LineWidth', 2, 'DisplayName', 'Nominal m and b');
%plot(V_led, LUX_b_max, '-o', 'LineWidth', 2, 'DisplayName', 'Original LUX');
plot(V_led, LUX_new, '-x', 'LineWidth', 2, 'DisplayName', 'Tuned m and b');
xlabel('LED Voltage [V]');
ylabel('Illuminance [LUX]');
title('Luxmeter Charateristic');
legend('show');
grid on;

% Log-log plot of LUX vs. R_LDR
figure;
loglog(LUX_b_min, R_ldr, '-o', 'LineWidth', 2, 'DisplayName', 'Lower R_{LDR}');
hold on;
loglog(LUX_b_nom, R_ldr, '-o', 'LineWidth', 2, 'DisplayName', 'Nominal R_{LDR}');
loglog(LUX_b_max, R_ldr, '-o', 'LineWidth', 2, 'DisplayName', 'Higher R_{LDR}');
loglog(LUX_new, R_ldr, '-x', 'LineWidth', 2, 'DisplayName', 'Tuned m and b');
xlabel('Illuminance [LUX]');
ylabel('R_{LDR} [Ω]');
title('LDR Characteristic (Illuminance-to-Resistance)');
legend('show');
grid on;
