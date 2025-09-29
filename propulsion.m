function [P] = propulsion(V, D, Alt)
%% ============================================================
% Propulsion Function  - Updated on28/09/2025
% Corrected Propulsion Function for AE 420 Framework
%  Francisco Javier Laso Iglesias
%
%  Inputs:
%    V   - flight velocity (ft/s)
%    D   - drag force = thrust required (lbf) 
%    Alt - altitude (ft)
%
%  Outputs:
%    P   - power consumption (watts)
% ============================================================


%% System Parameters (Your T-Motor U3-700 Configuration)
motor_Kv = 700;              % RPM/V
motor_Rm = 0.05;             % ohms (motor resistance)
motor_Imax = 25;             % A (maximum current)

% Battery System 100/6S LiPo
Vbatt_nom = 22.2;            % V (6S nominal)
batt_capacity_Ah = 84;       % Ah (converted from 84000 mAh)
batt_C_rating = 65;          % C rating
I_batt_max = batt_capacity_Ah * batt_C_rating; % max battery current
I_motor_max = min(motor_Imax, I_batt_max);     % actual motor current limit

% Efficiencies
eta_esc = 0.93;              % ESC efficiency
eta_motor = 0.88;            % realistic motor efficiency under load
eta_prop = 0.75;             % propeller efficiency (typical cruise)

% Propeller
prop_diam = 20/12;           % ft (20 inches)

%% Constants
rho0 = 0.002377;             % slug/ft^3
T0 = 518.67;                 % °R
L = 0.00356;                 % °R/ft
g = 32.174;                  % ft/s^2
R = 1716.59;                 % ft·lbf/(slug·°R)

%% Standard Atmosphere
T_alt = T0 - L*Alt;
if T_alt <= 0
    T_alt = 200; % prevent negative temperature
end
sigma = (T_alt/T0)^((g/(L*R)) - 1);
rho = rho0 * sigma;

%% Power Calculation 

% Step 1: Ideal propulsive power
P_ideal_ftlbfps = D * V;     % ft·lbf/s

% Step 2: Shaft power (account for propeller efficiency)
P_shaft_ftlbfps = P_ideal_ftlbfps / eta_prop; % ft·lbf/s

% Step 3: Convert to watts for motor analysis
P_shaft_watts = P_shaft_ftlbfps * 550 / 745.7; % convert ft·lbf/s to hp to watts
P_shaft_watts = P_shaft_ftlbfps * 1.356; % direct conversion: ft·lbf/s to watts

% Step 4: Motor electrical power
P_motor_elec = P_shaft_watts / eta_motor; % watts

% Step 5: Total electrical power (include ESC losses)
P_elec_total = P_motor_elec / eta_esc; % watts

% Step 6: Check system limits
P_batt_max = Vbatt_nom * I_motor_max; % maximum available power

% Motor voltage check (simplified)
% Estimate RPM needed for this power level
rpm_est = 4000; % reasonable estimate for cruise
omega_rad_s = rpm_est * (2*pi/60);
V_back_emf = omega_rad_s / (motor_Kv * 2*pi/60);
V_motor_est = V_back_emf + (P_motor_elec/Vbatt_nom) * motor_Rm;

% Check if motor can operate
if V_motor_est > Vbatt_nom || P_elec_total > P_batt_max
    % Scale back to feasible power
    P_feasible = min(P_elec_total, P_batt_max * 0.9);
    P = P_feasible;
else
    P = P_elec_total;
end
%end of code 
end

