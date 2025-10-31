function [P, maxT_vel, Pavalible] = propulsion_plus(v, d, Alt, CD0)
%% ============================================================
% Electric Propulsion & Performance Analysis
% Based on Traub (2011) - Battery-Powered Aircraft
%
% Updated: 01/26/2025 - Fixed graph display issues
%% ============================================================
%% Set default figure properties for white background and larger fonts
set(0, 'DefaultFigureColor', 'white');
set(0, 'DefaultAxesColor', 'white');
set(0, 'DefaultAxesFontSize', 14);
set(0, 'DefaultAxesFontWeight', 'bold');
set(0, 'DefaultTextFontSize', 14);
set(0, 'DefaultLegendFontSize', 12);
set(0, 'DefaultLineLineWidth', 2.5);
%% ---- Conversions ----
mph2fps = 1.4666667;
ftlbfps2W = 1.35581795;
fps2mph = 1/mph2fps;
N2lbf = 0.224809;
lbf2N = 1/N2lbf;
m2ft = 3.28084;
ft2m = 1/m2ft;
kgm3_to_slugft3 = 0.00194032;
%% ---- Atmosphere ----
rho0 = 0.002377; % slug/ft³ sea level
T0 = 518.67; L = 0.00356; g = 32.174; Rgas = 1716.59;
Temp = max(T0 - L*Alt, 200);
rho = rho0 * (Temp/T0)^(g/(L*Rgas)-1);
%% ---- Vehicle Parameters ----
W = 32;          % lbf (aircraft weight)
S = 14.7;       % ft² (wing area)
AR = 8;          % aspect ratio
e = 0.85;        % Oswald efficiency
k = 1/(pi*e*AR); % induced drag factor
%% ---- Motor & ESC ----
motor_Kv = 215;      % rpm/V
motor_Rm = 0.045;    % ohm
I0 = 1.2;            % no-load current (A)
eta_motor = 0.85;    % motor efficiency
eta_esc = 0.95;      % ESC efficiency
%% ---- Battery (12S3p LiPo) ----
Vbatt_nom = 44.4;      % V (nominal)
C_Ah = 15.0;          % Ah (capacity)
batt_C_rating = 1; % sustained C-rate
I_max = C_Ah * batt_C_rating * 10; % max current (A)
P_elec_max = Vbatt_nom * I_max;    % max electrical power (W)
%% ---- Propeller (20x10) ----
prop_D_ft = 20/12;   % diameter (ft)
prop_pitch_ft = 10/12; % pitch (ft)
rpm_max = motor_Kv * Vbatt_nom; % max RPM
%% ---- Propeller Model ----
CT_fun = @(J) max(0.01, 0.115 - 0.095*J - 0.025*J.^2);
CP_fun = @(J) max(0.01, 0.055 - 0.025*J - 0.015*J.^2);
eta_prop = @(J) J .* CT_fun(J) ./ CP_fun(J);
%% ---- Peukert Parameters (for Traub analysis) ----
Rt_hr = 1.0;         % rating hour (Traub Eq. 5b)
n_peuk = 1.3;        % Peukert exponent (LiPo)
eta_tot_traub = 0.50; % For Traub calculations (paper uses 0.5)
%% ---- Traub (2011) Relations ----
% Eq. 4: Power required (ft·lbf/s)
P_req = @(U,rho_val) 0.5*rho_val.*U.^3*S*CD0 + (2*W^2*k)./(rho_val.*U*S);
% Eq. 11: Optimal speed for max endurance
U_E = sqrt((2*W/S) * sqrt(k/(3*CD0)) / rho);  % ft/s
% Eq. 12: Optimal speed for max range
U_R = sqrt((2*W/S) * sqrt(k/CD0) / rho);      % ft/s
% Eq. 15: Power required at max endurance condition
P_E_ftlbfps = (2/sqrt(rho*S)) * CD0^0.25 * (2*W*sqrt(k/3))^1.5;
% Eq. 17: Power required at max range condition
P_R_ftlbfps = (1/sqrt(rho*S)) * CD0^0.25 * (2*W*sqrt(k))^1.5;
% Eq. 16: Maximum endurance with Peukert effect
P_E_W = P_E_ftlbfps * ftlbfps2W;
E_max_hr = Rt_hr^(1-n_peuk) * ((eta_tot_traub*Vbatt_nom*C_Ah)/P_E_W)^n_peuk;
% Eq. 18: Maximum range with Peukert effect
P_R_W = P_R_ftlbfps * ftlbfps2W;
R_max_ft = Rt_hr^(1-n_peuk) * ((eta_tot_traub*Vbatt_nom*C_Ah)/P_R_W)^n_peuk * U_R * 3600;
R_max_miles = R_max_ft / 5280;
%% ---- Velocity Grid ----
V_mph = linspace(0, 120, 100);
V_fps = V_mph * mph2fps;
%% ---- Drag Calculation ----
D_lbf = zeros(size(V_fps));
P_req_hp = zeros(size(V_fps));
for i = 1:length(V_fps)
    if V_fps(i) > 0.1
        q = 0.5 * rho * V_fps(i)^2;
        CL = W / (q * S);
        CD = CD0 + k * CL^2;
        D_lbf(i) = q * S * CD;
        P_req_hp(i) = (D_lbf(i) * V_fps(i)) / 550; % hp
    end
end
% Output power required
P = P_req_hp;
%% ---- RPM Range ----
RPM_range = [2000 4000 6000 8000 10000];
num_rpms = length(RPM_range);
%% ---- Thrust Available vs Velocity ----
Thrust_avail = zeros(length(V_fps), num_rpms);
Prop_eff_map = zeros(length(V_fps), num_rpms);
for j = 1:num_rpms
    rpm = RPM_range(j);
    n_rps = rpm / 60;
    
    for i = 1:length(V_fps)
        if n_rps > 0
            J = V_fps(i) / (n_rps * prop_D_ft);
            
            % Limit advance ratio to realistic range
            if J < 1.5
                CT = CT_fun(J);
                T = CT * rho * n_rps^2 * prop_D_ft^4;
                Thrust_avail(i,j) = max(T, 0);
                
                Prop_eff_map(i,j) = max(0, min(1, eta_prop(J)));
            else
                Thrust_avail(i,j) = 0;
                Prop_eff_map(i,j) = 0;
            end
        end
    end
end
maxT_vel = max(Thrust_avail, [], 2);
%% ---- Power Available ----
P_avail_W = zeros(num_rpms, 1);
Current_draw = zeros(num_rpms, 1);
for j = 1:num_rpms
    rpm = RPM_range(j);
    V_emf = rpm / motor_Kv;
    I = max((Vbatt_nom - V_emf) / motor_Rm, I0);
    I = min(I, I_max);
    
    P_elec = Vbatt_nom * I;
    P_shaft = eta_motor * eta_esc * (P_elec - I^2*motor_Rm - Vbatt_nom*I0);
    P_avail_W(j) = max(P_shaft, 0);
    Current_draw(j) = I;
end
Pavalible = max(P_avail_W) / 745.7;

% Calculate power available as function of velocity
P_avail_vs_vel_hp = zeros(size(V_fps));
for i = 1:length(V_fps)
    % Find maximum power available at this velocity across all RPMs
    P_at_vel = zeros(num_rpms, 1);
    for j = 1:num_rpms
        if Thrust_avail(i,j) > 0
            % Power = Thrust * Velocity
            P_at_vel(j) = (Thrust_avail(i,j) * V_fps(i) * ftlbfps2W) / 745.7; % convert to hp
        end
    end
    P_avail_vs_vel_hp(i) = max(P_at_vel);
end

%% ---- Endurance & Range vs Speed ----
Endurance_hrs = zeros(size(V_fps));
Range_miles = zeros(size(V_fps));
Energy_Wh = zeros(size(V_fps));
Current_A = zeros(size(V_fps));
for i = 1:length(V_fps)
    if V_fps(i) > 5 * mph2fps
        P_req_ftlbfps = P_req(V_fps(i), rho);
        P_req_W = P_req_ftlbfps * ftlbfps2W;
        
        t_hrs = Rt_hr^(1-n_peuk) * ((eta_tot_traub*Vbatt_nom*C_Ah)/P_req_W)^n_peuk;
        
        Endurance_hrs(i) = t_hrs;
        Range_miles(i) = V_mph(i) * t_hrs;
        Energy_Wh(i) = P_req_W * t_hrs;
        Current_A(i) = P_req_W / (eta_tot_traub * Vbatt_nom);
    end
end
Spec_energy = Energy_Wh ./ max(Range_miles, 0.1);
%% ---- Load CSV Data ----
csv_valid = false;
csv_power_valid = false;
csv_voltage_valid = false;
csv_torque_valid = false;
try
    if exist('motor_test_data.csv', 'file')
        raw = csvread('motor_test_data.csv', 1, 0);
        
        csv_rpm = raw(:, 6);
        csv_thrust_gf = raw(:, 4);
        valid = csv_rpm > 100 & csv_thrust_gf > 0;
        
        if sum(valid) > 5
            csv_rpm = csv_rpm(valid);
            csv_thrust_gf = csv_thrust_gf(valid);
            csv_thrust_lbf = csv_thrust_gf * 0.00220462;
            csv_thrust_N = csv_thrust_gf / 1000 * 9.81;
            
            csv_n_rps = csv_rpm / 60;
            csv_speed_fps = 0.6 * csv_n_rps * prop_D_ft;
            csv_speed_mph = csv_speed_fps * fps2mph;
            csv_speed_mps = csv_speed_fps * ft2m;
            
            csv_valid = true;
            
            if size(raw, 2) >= 7
                csv_power_W = raw(valid, 7);
                csv_current_A = raw(valid, 3);
                csv_voltage_V = raw(valid, 2);
                
                csv_voltage_valid = any(csv_voltage_V > 0);
                
                csv_power_mech_W = csv_thrust_lbf .* csv_speed_fps * ftlbfps2W;
                csv_power_mech_hp = csv_power_mech_W / 745.7;
                
                if sum(csv_power_W > 0) > 5
                    csv_efficiency = csv_power_mech_W ./ max(csv_power_W, 1);
                    csv_efficiency(csv_efficiency > 1 | csv_efficiency < 0) = NaN;
                    csv_power_valid = true;
                    
                    csv_CP = csv_power_W ./ (rho * csv_n_rps.^3 * prop_D_ft^5) / ftlbfps2W;
                    csv_CP(csv_CP < 0 | csv_CP > 0.3) = NaN;
                end
                
                if size(raw, 2) >= 5
                    csv_torque_Nm = raw(valid, 5) / 1000;
                    csv_torque_valid = any(csv_torque_Nm > 0);
                    
                    csv_power_from_torque_W = 2 * pi * csv_n_rps .* csv_torque_Nm;
                end
            end
            
            csv_CT = csv_thrust_lbf ./ (rho * csv_n_rps.^2 * prop_D_ft^4);
            csv_CT(csv_CT < 0 | csv_CT > 0.3) = NaN;
            
            csv_J = csv_speed_fps ./ (csv_n_rps * prop_D_ft);
            csv_J(csv_J < 0) = NaN;
            
            if csv_power_valid
                csv_prop_efficiency = (csv_thrust_lbf .* csv_speed_fps * ftlbfps2W) ./ csv_power_W;
                csv_prop_efficiency(csv_prop_efficiency > 1 | csv_prop_efficiency < 0) = NaN;
            end
        end
    end
catch ME
    csv_valid = false;
    csv_power_valid = false;
    fprintf('Warning: CSV loading failed - %s\n', ME.message);
end
%% ---- Air Density vs Altitude ----
Alt_range = 0:1000:20000;
rho_alt = rho0 * ((T0 - L*Alt_range)/T0).^(g/(L*Rgas)-1);
%% ===================== PLOTS =====================
fig = figure('Name', 'Main Propulsion Analysis', 'Position', [50, 50, 1920, 1080]);
fig.Position = get(0, 'ScreenSize');
tlo = tiledlayout(2,3,'Padding','compact','TileSpacing','compact');

% Plot 1: Power Available, Required, and Excess
ax1 = nexttile;
excess_hp = P_avail_vs_vel_hp - P_req_hp;
h1 = plot(V_mph, P_avail_vs_vel_hp, 'b-', 'LineWidth', 2.5);
hold on;
h2 = plot(V_mph, P_req_hp, 'r--', 'LineWidth', 2);
h4 = plot(V_mph, excess_hp, 'g-.', 'LineWidth', 2);

% Add vertical lines for target speed range (60-70 mph)
y_lims = get(gca, 'YLim');
plot([60 60], y_lims, 'm--', 'LineWidth', 2.5);
plot([70 70], y_lims, 'm--', 'LineWidth', 2.5);
% Add shaded region
patch([60 70 70 60], [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], ...
      'm', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

if csv_power_valid
    h3 = scatter(csv_speed_mph, csv_power_mech_hp, 60, 'filled', 'MarkerFaceColor', [0.8 0.3 0.3], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    legend([h1 h2 h4 h3], 'Power Available', 'Power Required', 'Excess Power', 'CSV Power Data', 'Location', 'best');
else
    legend([h1 h2 h4], 'Power Available', 'Power Required', 'Excess Power', 'Location', 'best');
end
ylabel('Power (hp)', 'FontWeight', 'bold');
xlabel('Airspeed (mph)', 'FontWeight', 'bold');
%title('Power Available, Required, and Excess', 'FontWeight', 'bold');
% Fixed ylim calculation
valid_data = [P_avail_vs_vel_hp(isfinite(P_avail_vs_vel_hp)); P_req_hp(isfinite(P_req_hp)); excess_hp(isfinite(excess_hp))];
if ~isempty(valid_data)
    y_min = min([min(valid_data(:)) 0]);
    y_max = max(valid_data(:));
    if isscalar(y_max) && isscalar(y_min) && y_max > y_min && isfinite(y_min) && isfinite(y_max)
        ylim([y_min*1.1 y_max*1.2]);
    end
end
xlim([0 130]);
grid on; box on;

% Plot 2: Thrust vs Velocity
ax2 = nexttile;
hold on; grid on; box on;
colors = jet(num_rpms);
leg_handles = [];
leg_labels = {};
for j = 1:num_rpms
    h = plot(V_mph, Thrust_avail(:,j), '-', 'LineWidth', 2.5, 'Color', colors(j,:));
    leg_handles(end+1) = h;
    leg_labels{end+1} = sprintf('%d RPM', round(RPM_range(j)));
end

% Add vertical lines for target speed range (60-70 mph)
y_lims_thrust = get(gca, 'YLim');
plot([60 60], y_lims_thrust, 'm--', 'LineWidth', 2.5);
plot([70 70], y_lims_thrust, 'm--', 'LineWidth', 2.5);
% Add shaded region
patch([60 70 70 60], [y_lims_thrust(1) y_lims_thrust(1) y_lims_thrust(2) y_lims_thrust(2)], ...
      'm', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

if csv_valid
    % Color-code test data points based on closest RPM
    [~, idx_rpm] = min(abs(RPM_range(:)' - csv_rpm(:)), [], 2);
    cmap = jet(num_rpms);
    
    for i = 1:length(csv_rpm)
        scatter(csv_speed_mph(i), csv_thrust_lbf(i), 90, ...
            'filled', 'MarkerFaceColor', cmap(idx_rpm(i), :), ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
    end
    
    leg_labels{end+1} = 'Test Data (color-coded by RPM)';
end
h_drag = plot(V_mph, D_lbf, 'k--', 'LineWidth', 3);
leg_handles(end+1) = h_drag;
leg_labels{end+1} = 'Drag';
xlabel('Airspeed (mph)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Thrust (lbf)', 'FontWeight', 'bold', 'FontSize', 11);
%title('Thrust vs Velocity - Multiple RPM Settings', 'FontWeight', 'bold', 'FontSize', 12);
legend(leg_handles, leg_labels, 'Location', 'northeast', 'FontSize', 9);
thrust_valid = maxT_vel(isfinite(maxT_vel) & maxT_vel > 0);
if ~isempty(thrust_valid)
    ylim([0 max(thrust_valid)*1.2]);
end
xlim([0 120]);

% Plot 3: Traub Endurance/Range
ax3 = nexttile;
yyaxis left
h1 = plot(V_mph, Endurance_hrs, 'b-', 'LineWidth', 2.5);
hold on;
h2 = plot(U_E*fps2mph, E_max_hr, 'bo', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', 'b');

% Add vertical lines for target speed range (60-70 mph)
y_lims_endur = get(gca, 'YLim');
plot([60 60], y_lims_endur, 'm--', 'LineWidth', 2);
plot([70 70], y_lims_endur, 'm--', 'LineWidth', 2);

if csv_power_valid && length(csv_speed_mph) > 10
    csv_endurance_hrs = zeros(size(csv_speed_mph));
    for i = 1:length(csv_speed_mph)
        if csv_power_W(i) > 0
            csv_endurance_hrs(i) = Rt_hr^(1-n_peuk) * ((eta_tot_traub*Vbatt_nom*C_Ah)/csv_power_W(i))^n_peuk;
        end
    end
    h3 = scatter(csv_speed_mph, csv_endurance_hrs, 50, 'b', 'filled', 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.6);
end
ylabel('Endurance (hours)', 'FontWeight', 'bold');
endur_valid = Endurance_hrs(isfinite(Endurance_hrs) & Endurance_hrs > 0);
if ~isempty(endur_valid)
    ylim([0 max(endur_valid)*1.2]);
else
    ylim([0 1]);
end
ax3.YColor = 'b';

yyaxis right
h4 = plot(V_mph, Range_miles, 'r-', 'LineWidth', 2.5);
h5 = plot(U_R*fps2mph, R_max_miles, 'rs', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', 'r');

% Add vertical lines for target speed range (60-70 mph)
y_lims_range = get(gca, 'YLim');
plot([60 60], y_lims_range, 'm--', 'LineWidth', 2);
plot([70 70], y_lims_range, 'm--', 'LineWidth', 2);
% Add shaded region
patch([60 70 70 60], [0 0 max(y_lims_range) max(y_lims_range)], ...
      'm', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

if csv_power_valid && length(csv_speed_mph) > 10
    csv_range_miles = csv_speed_mph .* csv_endurance_hrs;
    h6 = scatter(csv_speed_mph, csv_range_miles, 50, 'r', 'filled', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.6);
end
ylabel('Range (miles)', 'FontWeight', 'bold');
range_valid = Range_miles(isfinite(Range_miles) & Range_miles > 0);
if ~isempty(range_valid)
    ylim([0 max(range_valid)*1.2]);
else
    ylim([0 10]);
end
ax3.YColor = 'r';

xlabel('Airspeed (mph)', 'FontWeight', 'bold');
%title('Traub Endurance/Range Analysis', 'FontWeight', 'bold');
xlim([0 120]);
if csv_power_valid && exist('h6', 'var')
    legend([h1 h2 h3 h4 h5 h6], 'Endurance', 'Max Endurance', 'CSV Endurance', ...
           'Range', 'Max Range', 'CSV Range', 'Location', 'best', 'FontSize', 8);
else
    legend([h1 h2 h4 h5], 'Endurance', 'Max Endurance', 'Range', 'Max Range', ...
           'Location', 'best', 'FontSize', 9);
end
grid on; box on;

% Plot 4: Energy Consumption
ax4 = nexttile;
yyaxis left
h1 = plot(V_mph, Energy_Wh, 'b-', 'LineWidth', 2.5);
hold on;

% Add vertical lines for target speed range (60-70 mph)
y_lims_energy = get(gca, 'YLim');
plot([60 60], y_lims_energy, 'm--', 'LineWidth', 2);
plot([70 70], y_lims_energy, 'm--', 'LineWidth', 2);

if csv_power_valid && exist('csv_endurance_hrs', 'var')
    csv_energy_Wh = csv_power_W .* csv_endurance_hrs;
    h2 = scatter(csv_speed_mph, csv_energy_Wh, 60, 'b', 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.6);
end
ylabel('Energy per Flight (Wh)', 'FontWeight', 'bold');
energy_valid = Energy_Wh(isfinite(Energy_Wh) & Energy_Wh > 0);
if ~isempty(energy_valid)
    ylim([0 max(energy_valid)*1.2]);
end
ax4.YColor = 'b';

yyaxis right
h3 = plot(V_mph, Spec_energy, 'r-', 'LineWidth', 2.5);

% Add vertical lines for target speed range (60-70 mph)
y_lims_spec = get(gca, 'YLim');
plot([60 60], y_lims_spec, 'm--', 'LineWidth', 2);
plot([70 70], y_lims_spec, 'm--', 'LineWidth', 2);
% Add shaded region
patch([60 70 70 60], [0 0 max(y_lims_spec) max(y_lims_spec)], ...
      'm', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

if csv_power_valid && exist('csv_range_miles', 'var')
    csv_spec_energy = csv_energy_Wh ./ max(csv_range_miles, 0.1);
    h4 = scatter(csv_speed_mph, csv_spec_energy, 60, 'r', 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.6);
end
ylabel('Specific Energy (Wh/mile)', 'FontWeight', 'bold');
spec_valid = Spec_energy(isfinite(Spec_energy) & Spec_energy > 0);
if ~isempty(spec_valid)
    ylim([0 max(spec_valid)*1.2]);
end
ax4.YColor = 'r';

xlabel('Airspeed (mph)', 'FontWeight', 'bold');
%title('Energy Consumption Analysis', 'FontWeight', 'bold');
xlim([0 120]);
if csv_power_valid && exist('h4', 'var')
    legend([h1 h2 h3 h4], 'Energy/Flight', 'CSV Energy', 'Specific Energy', 'CSV Spec Energy', ...
           'Location', 'best', 'FontSize', 9);
else
    legend([h1 h3], 'Energy/Flight', 'Specific Energy', 'Location', 'best', 'FontSize', 9);
end
grid on; box on;

% Plot 5: Power Required per Thrust
ax5 = nexttile;
% Calculate power per thrust (hp per lbf of thrust required)
P_per_thrust = zeros(size(V_fps));
for i = 1:length(V_fps)
    if D_lbf(i) > 0.1
        P_per_thrust(i) = P_req_hp(i) / D_lbf(i);
    end
end

h1 = plot(V_mph, P_per_thrust, 'b-', 'LineWidth', 2.5);
hold on;

% Add vertical lines for target speed range (60-70 mph)
y_lims_pt = get(gca, 'YLim');
plot([60 60], y_lims_pt, 'm--', 'LineWidth', 2.5);
plot([70 70], y_lims_pt, 'm--', 'LineWidth', 2.5);
% Add shaded region
patch([60 70 70 60], [y_lims_pt(1) y_lims_pt(1) y_lims_pt(2) y_lims_pt(2)], ...
      'm', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

ylabel('Power Required per Thrust (hp/lbf)', 'FontWeight', 'bold');
xlabel('Airspeed (mph)', 'FontWeight', 'bold');
%title('Specific Power: Power Required per Pound of Thrust', 'FontWeight', 'bold');

% Mark optimal points
valid_pts = P_per_thrust > 0 & isfinite(P_per_thrust);
if any(valid_pts)
    [min_val, min_idx] = min(P_per_thrust(valid_pts));
    V_mph_valid = V_mph(valid_pts);
    h2 = plot(V_mph_valid(min_idx), min_val, 'ro', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', 'r');
    legend([h1 h2], 'Power/Thrust', sprintf('Most Efficient (%.1f mph)', V_mph_valid(min_idx)), 'Location', 'best');
    
    ylim([0 max(P_per_thrust(valid_pts))*1.2]);
end

xlim([0 120]);
grid on; box on;

% Plot 6: Propeller Efficiency
ax6 = nexttile;
hold on;
if size(Prop_eff_map,1) >= 2 && size(Prop_eff_map,2) >= 2
    % Clean up efficiency map - replace invalid values with NaN for better visualization
    Prop_eff_clean = Prop_eff_map;
    Prop_eff_clean(Prop_eff_clean < 0 | Prop_eff_clean > 1) = NaN;
    
    [C, h_contour] = contourf(RPM_range, V_mph, Prop_eff_clean*100, 20);
    
    colorbar;
    colormap(ax6, jet);
    caxis([0 100]);
end

% Add horizontal lines for target speed range (60-70 mph)
x_lims_prop = get(gca, 'XLim');
plot(x_lims_prop, [60 60], 'm--', 'LineWidth', 2.5);
plot(x_lims_prop, [70 70], 'm--', 'LineWidth', 2.5);
% Add shaded region
patch([x_lims_prop(1) x_lims_prop(2) x_lims_prop(2) x_lims_prop(1)], [60 60 70 70], ...
      'm', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

xlabel('RPM', 'FontWeight', 'bold');
ylabel('Airspeed (mph)', 'FontWeight', 'bold');
%title('Propeller Efficiency Map (%)', 'FontWeight', 'bold');
xlim([min(RPM_range) max(RPM_range)]);
ylim([0 120]);
grid on; box on;
drawnow;
