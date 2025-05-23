% === 1. Indlæs CSV-data ===
data = readmatrix('mag_data.csv');  % To kolonner: X, Y
x = data(:,1);
y = data(:,2);

% === 2. Beregn hard iron offset (bias) ===
x_offset = (max(x) + min(x)) / 2;
y_offset = (max(y) + min(y)) / 2;
bias = [x_offset, y_offset];

% === 3. Print bias til kommandovinduet ===
fprintf('Hard iron bias:\n');
fprintf('  X offset: %.3f\n', x_offset);
fprintf('  Y offset: %.3f\n', y_offset);

% === 4. Fjern offset (kalibreret data) ===
x_corr = x - x_offset;
y_corr = y - y_offset;

% === 5. Beregn plotgrænser (symmetrisk om 0) ===

% Før kalibrering – beregn max afstand fra (0,0)
x_max_raw = max(abs([min(x), max(x)]));
y_max_raw = max(abs([min(y), max(y)]));
r_max_raw = max([x_max_raw, y_max_raw]);
r_plot_raw = r_max_raw * 1.2;  % 20% margen

% Efter kalibrering – brug korrigerede data
r_max_corr = max([max(abs(x_corr)), max(abs(y_corr))]);
r_plot_corr = r_max_corr * 1.2;

% === 6. Plot FØR kalibrering ===
figure;
plot(x, y, '.-');
title('Før hard iron kalibrering');
xlabel('Magnetometer X');
ylabel('Magnetometer Y');
axis equal;
xlim([-r_plot_raw, r_plot_raw]);
ylim([-r_plot_raw, r_plot_raw]);
grid on;
xline(0, '--k');
yline(0, '--k');

% === 7. Plot EFTER kalibrering ===
figure;
plot(x_corr, y_corr, '.-');
title('Efter hard iron kalibrering');
xlabel('Magnetometer X (korrigeret)');
ylabel('Magnetometer Y (korrigeret)');
axis equal;
xlim([-r_plot_raw, r_plot_raw]);
ylim([-r_plot_raw, r_plot_raw]);
grid on;
xline(0, '--k');
yline(0, '--k');
