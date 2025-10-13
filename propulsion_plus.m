function [P] = propulsion_plus(v, d, Alt)
%% ============================================================
% Electric Propulsion & Performance Analysis 
% Updated on 01/10/2025
% Written by: Francisco Javier Laso Iglesias 
% refined by: 
% - Drag-polar power required
% - Prop map via simple CT(J), CP(J) placeholders 
% - Back-EMF & electrical power caps (ESC/battery)
% - Traub (2011) endurance & range with Peukert effect
% - Operating-point tables + plots
%
% USAGE
%   P = propulsion_analysis_complete(v, d, Alt)
%     v   : flight speed at the operating point [ft/s]
%     d   : required thrust/drag at that point [lbf]
%     Alt : pressure altitude [ft]
%
% RETURNS
%   P : Electrical input power at (v, d) [W], limited by system caps
%
% Notes:
%   * All internal calcs are IMPERIAL Units as asked by Chief Engineer
%   * Traub (2011) relations used for U_E, U_R, and endurance with Peukert.
% ============================================================
%% ---- Vehicle & System numbers ----
% Motor / ESC / Battery (3S2P, 22 Ah cells -> 132 Ah total)
motor_Kv       = 380;        % rpm/V (spec)
motor_Rm       = 0.0317;       % ohm 
esc_I_cont     = 150;         % A (controller limit)
eta_motor_cru  = 0.85;       % -
eta_esc        = 0.93;       % -
Vbatt_nom        = 11.1;     % V (3S nominal)
batt_capacity_Ah = 132;     % Ah (2P × 22 Ah)
batt_C_rating    = 80/120;      % sustained C for ops 
I_batt_max       = batt_capacity_Ah * batt_C_rating; % A
I_lim            = min(esc_I_cont, I_batt_max);       % A (electrical current cap)
P_elec_max       = Vbatt_nom * I_lim;                 % W (electrical power cap)
% Propeller geometry (22×12 in, 2 blades)
prop_D_ft     = 22/12;       % ft (diameter)
prop_pitch_ft = 18/12;       % ft (pitch)
prop_blades   = 2;
rpm_max_hw    = motor_Kv * Vbatt_nom;  % rpm (no-load back-EMF cap)
% Airframe / Aerodynamics
W   = 40.00;   % lbf  (18.14 kg ≈ 40 lbf)
S   = 8.09;    % ft^2 (75.25 dm^2)
CD0 = 0.03;    % parasite drag
b_ft= 1.27/0.3048;          % wingspan [ft]
AR  = b_ft^2 / S;           % ≈ 21.4
e   = 0.75;                 % Oswald
k   = 1/(pi*e*AR);          % induced-drag factor
%% ---- Atmosphere ----
rho0 = 0.002377; T0 = 518.67; L = 0.00356; g = 32.174; Rgas = 1716.59; % imperial
atmo = @(hft) deal( max(T0 - L*hft, 200), rho0 * (max(T0 - L*hft,200)/T0)^(g/(L*Rgas)-1) );
%% ---- Conversions ----
mph2fps   = 1.4666667;
ftlbfps2W = 1.35581795;
%% ---- Simple prop map  ----
% PConst / TConst
PConst = 1.20;   % power fudge factor
TConst = 1.00;   % thrust fudge factor
% Advance ratio J = V / (n D); helper with divide-by-zero guard
J_of = @(V_fps, rpm) V_fps ./ max((rpm/60) * prop_D_ft, eps);
% Crude shapes vs J (keep within 0..~1.2); replace with data when available
CT_fun_raw = @(J) (0.12 - 0.08.*J);         % falls with J
CP_fun_raw = @(J) (0.06 + 0.04.*J);         % rises slightly with J
CT_fun     = @(J) max(0.02, CT_fun_raw(J) * TConst);
CP_fun     = @(J) max(0.02, CP_fun_raw(J) * PConst);
etap_fun   = @(J) J .* CT_fun(J) ./ CP_fun(J);  % propulsive efficiency
J_clamp    = @(J) min(max(J,0), 1.2);
%% ---- Total efficiency used in endurance (tunable) ----
eta_prop_guess = 0.75;                                   % typical well-matched cruise
eta_tot_guess  = eta_esc * eta_motor_cru * eta_prop_guess;
%% ---- Traub (2011) relations (Peukert) ----
% Power required for level flight (ft·lbf/s) via drag polar:
P_req = @(U,rho) 0.5*rho*U.^3*S*CD0 + (2*W^2*k)./(rho.*U.*S);
% Endurance vs speed with Peukert (hours):
Rt_hr  = 1.0;     % rating hour
C_Ah   = batt_capacity_Ah;
n_peuk = 1.30;    % 1.1–1.3 typical for LiPo;
E_hours = @(U,rho,eta_tot) (Rt_hr^(1-n_peuk) .* (eta_tot*Vbatt_nom*C_Ah ./ (P_req(U,rho)*ftlbfps2W))).^n_peuk;
% Analytic speeds for max endurance & range (from drag polar):
U_E = @(rho) sqrt(2*W./(rho*S)) .* (k/(3*CD0)).^(1/4);
U_R = @(rho) sqrt(2*W./(rho*S)) .* (k/(CD0)).^(1/4);
%% ---- Analysis grids ----
alts = [0 2500 5000 7500 10000 12500];
[maxT_alt, rho_alt] = deal(zeros(size(alts)));
[~, rho_in] = atmo(Alt);   % density at requested analysis altitude
% 1) Max static thrust vs altitude (J≈0, rpm capped by hardware)
for i = 1:numel(alts)
    [~, rho]   = atmo(alts(i));
    rho_alt(i) = rho;
    rpm        = rpm_max_hw;         % conservative back-EMF cap
    n          = rpm/60;             % rev/s
    J          = 0;                  % static
    CT         = CT_fun(J);
    % Dimensional consistency in imperial prop theory:
    maxT_alt(i) = CT * rho * (n^2) * (prop_D_ft^4);   % lbf
end
% 2) Max thrust vs airspeed at Alt (respect back-EMF & electrical caps)
V_mph = linspace(0,120,60); V_fps = V_mph * mph2fps;
maxT_vel = zeros(size(V_fps));
for i = 1:numel(V_fps)
    V = V_fps(i); Tmax = 0;
    for rpm = linspace(2000, rpm_max_hw, 30)
        n = rpm/60;
        J = J_clamp((V>0) * V/(n*prop_D_ft));
        CT = CT_fun(J); CP = CP_fun(J);
        T  = CT * rho_in * n^2 * prop_D_ft^4;                     % lbf
        Pshaft_W = (CP * rho_in * n^3 * prop_D_ft^5) * ftlbfps2W; % W
        eta_prop_use = max(etap_fun(J), 0.05);
        Pelec_W  = Pshaft_W / (eta_motor_cru*eta_esc*eta_prop_use);
        % Back-EMF screen (rpm ≈ Kv*V); margin for sag/losses:
        Vbemf = rpm / motor_Kv;  % V
        if (Vbemf <= 0.85*Vbatt_nom) && (Pelec_W <= P_elec_max)
            Tmax = max(Tmax, T);
        end
    end
    maxT_vel(i) = Tmax;
end
% 3) Cruise power vs thrust 
T_levels = linspace(5,20,20);  % lbf
P_for_T  = zeros(size(T_levels));
V_for_T  = zeros(size(T_levels));
for i = 1:numel(T_levels)
    Ugrid = linspace(20, 100, 200)*mph2fps;
    q     = 0.5*rho_in*(Ugrid.^2);
    CL    = W./(q*S);
    CD    = CD0 + k.*CL.^2;
    Dvals = q.*S.*CD;
    [~,ix] = min(abs(Dvals - T_levels(i)));
    Umatch = Ugrid(ix);
    V_for_T(i) = Umatch;
    Preq_W = P_req(Umatch, rho_in)*ftlbfps2W;
    P_for_T(i) = Preq_W / (eta_prop_guess*eta_motor_cru*eta_esc);
end
% 4) Thrust vs velocity for throttle fractions (power-limited)
throttle_levels = [0.3 0.5 0.7 0.85 1.0];
V_th_mph = 0:100;
V_th_fps = V_th_mph*mph2fps;
T_th_mat = zeros(numel(throttle_levels), numel(V_th_fps));
for ii = 1:numel(throttle_levels)
    Pavail = P_elec_max * throttle_levels(ii);
    rpm_top = rpm_max_hw * throttle_levels(ii);
    for jj = 1:numel(V_th_fps)
        V = V_th_fps(jj);
        Tmax = 0;
        for rpm = linspace(1000, rpm_top, 24)
            n = rpm/60;
            J = J_clamp((V>0) * V/(n*prop_D_ft));
            CT = CT_fun(J); CP = CP_fun(J);
            T  = CT * rho_in * n^2 * prop_D_ft^4;
            Pshaft_W = (CP * rho_in * n^3 * prop_D_ft^5) * ftlbfps2W;
            eta_prop_use = max(etap_fun(J), 0.05);
            Pelec_W  = Pshaft_W / (eta_motor_cru*eta_esc*eta_prop_use);
            if Pelec_W <= Pavail; Tmax = max(Tmax, T); end
        end
        T_th_mat(ii,jj) = Tmax;
    end
end
% 5) Traub endurance/range at Alt (numeric + analytic)
[~, rho_input] = atmo(Alt);
U_grid_mph = linspace(20, 100, 200);
U_grid_fps = U_grid_mph*mph2fps;
E_grid_hr  = E_hours(U_grid_fps, rho_input, eta_tot_guess);
[Emx_hr, ixE] = max(E_grid_hr);
UmxE = U_grid_fps(ixE);
Ue_ana = U_E(rho_input);
Ur_ana = U_R(rho_input);
Emax_hr = E_hours(Ue_ana, rho_input, eta_tot_guess);
Rmax_ft = Ur_ana * E_hours(Ur_ana, rho_input, eta_tot_guess) * 3600;  % ft
Rmax_mi = Rmax_ft / 5280;
%% ---- Output P for the input point (v, d) ----
if d > 0 && v > 0
    Preq_input_W = (d * v) * ftlbfps2W;                % aerodynamic power
    P = min(Preq_input_W / eta_tot_guess, P_elec_max); % electrical input
else
    P = 1000; % guard value
end
%% ======================= PLOTS =======================
% 1) Maximum Thrust & Cruise Power
figure('Name','1. Maximum Thrust Performance','Position',[50,50,1400,500]);
subplot(1,3,1);
plot(alts/1000, maxT_alt, 'bo-','LineWidth',2.2,'MarkerFaceColor','b');
grid on; box on; xlabel('Altitude (kft)'); ylabel('Max Static Thrust (lbf)');
title('Max Thrust vs Altitude');
subplot(1,3,2);
plot(V_mph, maxT_vel, 'ro-','LineWidth',2.2,'MarkerFaceColor','r');
grid on; box on; xlabel('Velocity (mph)'); ylabel('Max Thrust (lbf)');
title('Max Thrust vs Velocity');
subplot(1,3,3);
plot(T_levels, P_for_T/1000, 'go-','LineWidth',2.2,'MarkerFaceColor','g');
grid on; box on; xlabel('Required Thrust (lbf)'); ylabel('Electrical Power (kW)');
title('Cruise Power vs Thrust');
% 2) Thrust vs Velocity – Multiple Throttle Settings
figure('Name','2. Thrust vs Velocity by Throttle','Position',[100,100,1000,700]); hold on; grid on; box on;
for ii = 1:numel(throttle_levels)
    plot(V_th_mph, T_th_mat(ii,:), 'LineWidth',2.2, ...
         'DisplayName',sprintf('%.0f%% Throttle',100*throttle_levels(ii)));
end
xlabel('Velocity (mph)'); ylabel('Available Thrust (lbf)');
title('Available Thrust vs Velocity (Throttle)'); legend('Location','northeast');
% 3) Traub Endurance/Range 
figure('Name','3. Traub Endurance & Range','Position',[120,120,1200,500]);
subplot(1,2,1);
plot(U_grid_mph, E_grid_hr,'k-','LineWidth',2.2); hold on;
xline(UmxE/mph2fps,'--','E_{max} (numeric)');
xline(Ue_ana/mph2fps,':','U_E (analytic)');
grid on; box on; xlabel('Speed (mph)'); ylabel('Endurance (hr)');
title(sprintf('Endurance vs Speed (Peukert n=%.2f)', n_peuk));
legend('E(U)','Numeric Peak','Analytic U_E','Location','best');
subplot(1,2,2);
bar([Emax_hr, Rmax_mi]); grid on; box on;
set(gca,'XTickLabel',{'E_{max} (hr)','R_{max} (mi)'}); title('Max Endurance & Range');
% 4) Energy ("Fuel") Consumption Variations
V_const    = 60 * mph2fps;               % fixed speed for (power vs thrust)
T_vec      = linspace(8,18,15);          % lbf
P_vs_T_W   = (T_vec * V_const) * ftlbfps2W / eta_tot_guess;
T_const    = 12;                         % lbf
V_vec_mph  = linspace(30,80,15);
V_vec_fps  = V_vec_mph * mph2fps;
P_vs_V_W   = (T_const * V_vec_fps) * ftlbfps2W / eta_tot_guess;
P_vs_alt_W = zeros(size(alts));          % fixed speed for (power vs altitude)
for i = 1:numel(alts)
    [~, rho_a] = atmo(alts(i));
    P_req_alt_ftlbfps = P_req(V_const, rho_a);
    P_vs_alt_W(i)     = P_req_alt_ftlbfps * ftlbfps2W / eta_tot_guess;
end
figure('Name','4. Energy Consumption Variations','Position',[160,160,1400,900]);
subplot(2,2,1);
plot(T_vec, P_vs_T_W/1000, 'bo-','LineWidth',2.2,'MarkerFaceColor','b');
grid on; box on; xlabel('Required Thrust (lbf)'); ylabel('Electrical Power (kW)');
title('Power vs Thrust (at 60 mph)');
subplot(2,2,2);
plot(V_vec_mph, P_vs_V_W/1000, 'ro-','LineWidth',2.2,'MarkerFaceColor','r');
grid on; box on; xlabel('Velocity (mph)'); ylabel('Electrical Power (kW)');
title('Power vs Velocity (at 12 lbf)');
subplot(2,2,3);
plot(alts/1000, P_vs_alt_W/1000, 'go-','LineWidth',2.2,'MarkerFaceColor','g');
grid on; box on; xlabel('Altitude (kft)'); ylabel('Electrical Power (kW)');
title('Power vs Altitude (level flight at 60 mph)');
subplot(2,2,4);
plot(V_vec_mph, (P_vs_V_W)/1000, 'mo-','LineWidth',2.2,'MarkerFaceColor','m');
grid on; box on; xlabel('Velocity (mph)'); ylabel('Energy Rate (kW·h/h)');
title('Energy Consumption Rate vs Velocity');
% 5) Air density vs altitude
figure('Name','5. Air Density vs Altitude','Position',[180,180,600,400]);
plot(alts/1000, rho_alt, 'mo-','LineWidth',2.2,'MarkerFaceColor','m');
grid on; box on; xlabel('Altitude (kft)'); ylabel('Air Density (slug/ft^3)');
title('Air Density vs Altitude');
%% ====== OPERATIONAL METRICS (Power, Energy, Cost, Battery, Endurance) ======
% Costs & pack-level assumptions
electricity_cost_per_kWh = 0.15;  % $/kWh
usable_DoD               = 0.85;  % usable depth of discharge (fraction)
pack_spec_energy_Whkg    = 180;   % pack-level specific energy (Wh/kg)
% Helpers
Wh_per_hr = 1;          % W·hr per hour = Wh
kg2lb     = 2.20462262;
% Battery energy on board (nominal) & usable
E_pack_Wh_nom    = Vbatt_nom * batt_capacity_Ah;
E_pack_Wh_usable = E_pack_Wh_nom * usable_DoD;
% Pack mass/weight estimate (context)
pack_mass_est_kg   = E_pack_Wh_nom / pack_spec_energy_Whkg;
pack_weight_est_lb = pack_mass_est_kg * kg2lb;
% Define three operating points: Input point, U_E (endurance), U_R (range)
ops(1).name = 'Input (v,d)';
ops(2).name = 'Max Endurance U_E';
ops(3).name = 'Max Range U_R';
% 1) Input point: if invalid, estimate from drag at v
if (d > 0 && v > 0)
    U_in = v;
    Preq_input_W = (d * v) * ftlbfps2W;    % aerodynamic power
else
    U_in = max(UmxE, 30*mph2fps);          % fallback speed
    Preq_input_W = P_req(U_in, rho_input) * ftlbfps2W;
end
P_elec_in_W = Preq_input_W / eta_tot_guess;
ops(1).U_fps   = U_in;
ops(1).Pelec_W = min(P_elec_in_W, P_elec_max);
% 2) Max endurance speed U_E
Ue = U_E(rho_input);
Preq_E_W = P_req(Ue, rho_input)*ftlbfps2W;
ops(2).U_fps   = Ue;
ops(2).Pelec_W = min(Preq_E_W/eta_tot_guess, P_elec_max);
% 3) Max range speed U_R
Ur = U_R(rho_input);
Preq_R_W = P_req(Ur, rho_input)*ftlbfps2W;
ops(3).U_fps   = Ur;
ops(3).Pelec_W = min(Preq_R_W/eta_tot_guess, P_elec_max);
% Compute metrics for each operating point (no fixed mission time)
for ii = 1:numel(ops)
    U = ops(ii).U_fps;  P_elec_W = ops(ii).Pelec_W;
    % 1) Power & energy rate
    ops(ii).Power_kW = P_elec_W/1000;
    ops(ii).EnergyRate_kW   = P_elec_W/1000;        % kWh/h = kW
    ops(ii).EnergyRate_Whph = P_elec_W * Wh_per_hr; % Wh/h
    % 2) Cost per hour
    ops(ii).CostPerHour_USD = ops(ii).EnergyRate_kW * electricity_cost_per_kWh;
    % 3) Endurance (two ways)
    Preq_ftlbfps = P_req(U, rho_input);
    E_peuk_hr  = (Rt_hr^(1-n_peuk) * ...
                 (eta_tot_guess * Vbatt_nom * (C_Ah*usable_DoD) / ...
                 (Preq_ftlbfps*ftlbfps2W))^n_peuk);         % Traub + Peukert
    E_ideal_hr = E_pack_Wh_usable / max(P_elec_W,1);         % simple Wh / P
    ops(ii).Endurance_hr      = E_peuk_hr;
    ops(ii).EnduranceIdeal_hr = E_ideal_hr;
    % 4) Conservative mission time: limiting endurance
    ops(ii).MissionTime_hr = min(E_peuk_hr, E_ideal_hr);
    % 5) Energy used over that mission time (cap at usable Wh)
    ops(ii).EnergyUsed_Wh = min(E_pack_Wh_usable, P_elec_W * ops(ii).MissionTime_hr);
    % 6) Battery mass needed to support that mission time (with DoD)
    batt_mass_needed_kg         = ops(ii).EnergyUsed_Wh / max(pack_spec_energy_Whkg * usable_DoD, eps);
    ops(ii).BattMassNeeded_kg   = batt_mass_needed_kg;
    ops(ii).BattWeightNeeded_lb = batt_mass_needed_kg * kg2lb;
end
%% ---- Console summary ----
fprintf('\n================= PROPULSION ANALYSIS =================\n');
fprintf('Power cap: P_elec_max = %.0f W | I_lim = %.0f A | Battery = 3S4P %.1fAh\n', P_elec_max, I_lim, batt_capacity_Ah);
fprintf('Alt: %.0f ft | rho = %.5f slug/ft^3\n', Alt, rho_input);
fprintf('U_E (analytic): %.1f mph | U_R (analytic): %.1f mph\n', Ue_ana/mph2fps, Ur_ana/mph2fps);
fprintf('E_max (Peukert n=%.2f): %.2f hr | R_max: %.1f miles\n', n_peuk, Emax_hr, Rmax_mi);
fprintf('Input: v=%.1f ft/s, d=%.1f lbf -> P_elec ~= %.0f W (cap %.0f W)\n\n', v, d, P, P_elec_max);
fprintf('--------------------- OPERATIONAL METRICS ---------------------\n');
fprintf('Battery nominal energy: %.0f Wh | Usable (DoD %.0f%%): %.0f Wh\n', ...
        E_pack_Wh_nom, 100*usable_DoD, E_pack_Wh_usable);
fprintf('Pack est. mass: %.2f kg (%.2f lb) @ %.0f Wh/kg\n', ...
        pack_mass_est_kg, pack_weight_est_lb, pack_spec_energy_Whkg);
fprintf('Electricity price: $%.2f/kWh\n\n', electricity_cost_per_kWh);
fprintf('%-20s | %-10s | %-10s | %-12s | %-15s | %-15s | %-12s\n', ...
    'Operating Point', 'Speed(mph)', 'Power(kW)', 'Mission(hr)', 'Energy Used(Wh)', 'Battery Mass(kg)', 'Endur(hr)');
fprintf(repmat('-',1,140)); fprintf('\n');
for ii = 1:numel(ops)
    mph = ops(ii).U_fps / mph2fps;
    fprintf('%-20s | %10.1f | %10.2f | %12.2f | %15.0f | %15.2f | %12.2f\n', ...
        ops(ii).name, mph, ops(ii).Power_kW, ops(ii).MissionTime_hr, ...
        ops(ii).EnergyUsed_Wh, ops(ii).BattMassNeeded_kg, ops(ii).Endurance_hr);
end
% Aggregate summary
avg_power_consumption = mean([ops.Power_kW]);
max_thrust_available  = max(maxT_vel);
fprintf('\n--------------------- SUMMARY METRICS ---------------------\n');
fprintf('Max Thrust Available: %.1f lbf\n', max_thrust_available);
fprintf('Average Power Consumption: %.2f kW\n', avg_power_consumption);
fprintf('Max Endurance (Theoretical, Peukert): %.2f hours\n', Emax_hr);
fprintf('Max Range (Theoretical): %.1f miles\n', Rmax_mi);
fprintf('System Efficiency (assumed): %.1f%% (motor: %.1f%%, ESC: %.1f%%, prop: %.1f%%)\n', ...
    eta_tot_guess*100, eta_motor_cru*100, eta_esc*100, eta_prop_guess*100);
end
