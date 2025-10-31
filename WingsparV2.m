%% === Bending Capacity vs Wall Fraction (Calibrated Across Materials) ===
clear; clc; close all;

%% === GEOMETRY ===
width_mm = 32;                        % [mm] outer square width
t_frac = linspace(0.005, 0.50, 250);  % wall fraction (0.5%–50%)
t_mm = t_frac * width_mm;

%% === UNIT CONVERSIONS ===
mm2m = 1e-3; N_m_to_lbft = 0.737562;
b = width_mm * mm2m;  t = t_mm * mm2m;  c = b/2;

%% === NOMINAL MATERIAL DATA ===
materials(1).name = 'Al 6061-T6';
materials(1).E = 68.99e9;  materials(1).sigma_nom = 276e6;  materials(1).rho = 2.70e3;

materials(2).name = 'CF (101 GPa / 1260 MPa)';
materials(2).E = 101e9;    materials(2).sigma_nom = 1260e6; materials(2).rho = 1.70e3;

materials(3).name = 'T300/Epoxy (baseline test)';
materials(3).E = 115e9;    materials(3).sigma_nom = 1800e6; materials(3).rho = 1.55e3;

%% === CALIBRATION FACTOR FROM EXPERIMENT ===
sigma_test = 165e6;         % Pa (effective allowable from 3-point bend test)
sigma_nom_T300 = materials(3).sigma_nom;
scale_factor = sigma_test / sigma_nom_T300;   % ≈ 0.092 (9.2%)

% Apply same scaling factor to all materials
for i = 1:length(materials)
    materials(i).sigma_eff = materials(i).sigma_nom * scale_factor;
end

fprintf('Calibrating all materials to T300 experimental ratio (%.1f%% of nominal strength)\n', ...
        scale_factor*100);

%% === MOMENT OF INERTIA ===
I = (b.^4 - (b - 2.*t).^4)/12;   % [m^4]

%% === COMPUTE & PLOT ===
figure('Color','w'); hold on; box on; grid on;
colors = lines(length(materials));

for i = 1:length(materials)
    sigma_allow = materials(i).sigma_eff;         % [Pa]
    M_Nm = (sigma_allow .* I) ./ c;               % [N·m]
    M_lbft = M_Nm * N_m_to_lbft;                  % [lb·ft]

    plot(t_frac*100, M_lbft, 'LineWidth', 2, 'Color', colors(i,:), ...
        'DisplayName', sprintf('%s (σ_eff = %.0f MPa)', ...
        materials(i).name, sigma_allow/1e6));
end

%% === LABELING ===
xlabel('Wall Thickness as % of Width');
ylabel('Maximum Bending Moment [lb·ft]');
title('Maximum Bending Moment vs Wall Thickness (Calibrated to T300 Experiment)');
legend('Location','northwest');
set(gca,'FontSize',12);
xlim([0 50]); ylim padded;
xline(50,'--k','Solid Section Limit','LabelVerticalAlignment','bottom','FontSize',11);

%% === PRINT SAMPLE ===
t_target = 1.2;  % mm
I_target = (b^4 - (b - 2*(t_target*mm2m))^4)/12;
fprintf('\nAt t = %.1f mm (t/b = %.2f%%):\n', t_target, t_target/width_mm*100);
for i = 1:length(materials)
    M_Nm = materials(i).sigma_eff * I_target / c;
    M_lbft = M_Nm * N_m_to_lbft;
    fprintf('  %-30s M_max ≈ %7.1f lb·ft (σ_eff = %.0f MPa)\n', ...
        materials(i).name, M_lbft, materials(i).sigma_eff/1e6);
end
