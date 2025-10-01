function [P] = propulsion(V, D, Alt)
%% ============================================================
% Propulsion Function
% Updated: September 30, 2025
% Francisco Javier Laso Iglesias
%
% Inputs:
%   V   - flight velocity (ft/s)
%   D   - drag force = thrust required (lbf)
%   Alt - altitude (ft)
%
% Outputs:
%   P   - electrical power consumption (watts)
%
% Reference: Traub, L.W. (2011). "Range and Endurance Estimates
%            for Battery-Powered Aircraft." Journal of Aircraft.
% ============================================================
 
%% System Parameters
% Motor MAD Components CM 4530 Lifter
motor_Kv = 380;              % RPM/V
motor_Rm = 0.0317;             % ohms (motor resistance)
motor_Imax = 122.3;             % A
 
% Battery System: 3S4P LiPo
% 3S = 22.2V nominal, 2P = 2 cells in parallel
Vbatt_nom = 11.1;            % V (3S nominal: 3.7V × 3)
batt_capacity_Ah = 132;    % Ah (4P × 14.12 Ah per cell)
batt_C_rating = 80/120;         % Continuous discharge C-rating
I_batt_max = batt_capacity_Ah * batt_C_rating;
I_motor_max = min(motor_Imax, I_batt_max);
 
% System Efficiencies
eta_esc = 0.93;              % ESC efficiency
eta_motor = 0.85;            % Motor efficiency (cruise)
eta_prop = 0.75;             % Propeller efficiency (cruise)
eta_tot = eta_esc * eta_motor * eta_prop;  % Total efficiency
 
% Propeller
prop_diam = 22/12;           % ft (20 inches diameter)
 
%% Physical Constants
rho0 = 0.002377;             % slug/ft^3 (sea level)
T0 = 518.67;                 % °R (sea level temperature)
L = 0.00356;                 % °R/ft (lapse rate)
g = 32.174;                  % ft/s^2
R = 1716.59;                 % ft·lbf/(slug·°R)
 
%% Conversion Factor
ftlbfps_to_W = 1.35582;      % ft·lbf/s to Watts
 
%% Standard Atmosphere Model
T_alt = T0 - L*Alt;
if T_alt <= 0
    T_alt = 200;
end
sigma = (T_alt/T0)^((g/(L*R)) - 1);
rho = rho0 * sigma;
 
%% Power Calculation
 
% Step 1: Aerodynamic power required
% P_req = D × U
P_aero_ftlbfps = D * V;      % ft·lbf/s
 
% Step 2: Convert to Watts
P_aero_W = P_aero_ftlbfps * ftlbfps_to_W;
 
% Step 3: Electrical input power
% Per Traub: P_battery = P_required / η_tot
P_elec_W = P_aero_W / eta_tot;
 
%% System Limits Check
 
% Maximum power available from battery
P_batt_max = Vbatt_nom * I_motor_max;  % Watts
 
% Motor back-EMF check
% Typical cruise advance ratio J ≈ 0.8 for efficient props
J_cruise = 0.8;
n_rps = V / (J_cruise * prop_diam);  % rev/s
rpm_est = n_rps * 60;                 % RPM
 
% Back-EMF voltage
V_back_emf = rpm_est / motor_Kv;
 
% Required motor voltage (simplified)
I_motor_est = P_elec_W / Vbatt_nom;
V_motor_req = V_back_emf + (I_motor_est * motor_Rm);
 
% Check operating limits
if V_motor_req > Vbatt_nom || P_elec_W > P_batt_max
    % System is at or beyond limits - cap to maximum available
    P = min(P_elec_W, P_batt_max * 0.95);  % 95% for safety margin
    warning('Power demand (%.0f W) exceeds system limits. Capping to %.0f W.', P_elec_W, P);
else
    P = P_elec_W;
end
 
end
 
