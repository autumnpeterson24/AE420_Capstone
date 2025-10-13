function [SM, I] = stability(CG, AR, t, Vh, ARh, alpha, cmac, hn, config)
% REVISON 2.6 (9/24/25)
%
% This function calculates the static margin (simplified) and the required
% horizontal tail incidence angle for a given aircraft configuration.
%
% --- Method ---
% 1. Select a design configuration (1-4) to load its unique parameters.
% 2. Calculate the aerodynamic center (AC) of the wing.
% 3. Calculate the static margin as the non-dimensional distance between the
%    AC and the Center of Gravity (CG).
% 4. Estimate the wing's pitching moment coefficient about the AC.
% 5. Determine the required tail incidence angle 'it' to trim the aircraft
%    (achieve Cm = 0) at zero lift.
%
% --- Inputs ---
%   AR     - Wing Aspect Ratio
%   Vh     - Horizontal tail volume coefficient
%   ARh    - Horizontal tail aspect ratio
%   cmac   - Wing mean aerodynamic chord
%   config - Configuration number to use for the analysis (1, 2, 3, or 4)
%
% --- Outputs ---
%   SM     - Static Margin (non-dimensional, based on AC location)
%   I      - Required tail incidence angle (degrees)
%

%% 1. AIRCRAFT CONFIGURATION DATABASE
% --- Each column represents a different aircraft configuration ---
% --- Please replace these placeholder values with your actual data ---

% Longitudinal locations are measured from the aircraft nose
Lw_vec     = [5.0, 4.0, 2.8, 1.583 0.250]; % x-location of the wing's leading edge
% Wing-specific geometric properties REPLACE WITH REAL VALUES
Lambda_vec = [10,  10,  10,  2.5,  0.000]; % Wing sweep angle (degrees)

%% 2. CONSTANTS & ASSUMPTIONS
% --- These values are assumed to be constant across all configurations ---
a0t = 0.113575; % 2D lift curve slope for tail airfoil (per radian, for NACA 0015)
Cm0 = -0.09;   % 2D airfoil moment coefficient at AoA=0 (BOE103)

%% 3. SELECT ACTIVE CONFIGURATION
% --- Extracts the data for the chosen 'config' number ---
Lw     = Lw_vec(config);
Lambda = Lambda_vec(config); % Wing sweep in degrees

%% 4. CALCULATIONS
% --- Static Margin ---
SM  = (hn - CG)/cmac;   % static margin taken in x-distance from the nose 

% --- Tail Incidence Angle ---
% Convert 2D tail lift curve slope to 3D using lifting-line theory
at = a0t / (1 + (a0t / (pi * ARh)));

% Convert wing sweep from degrees to radians for calculations
Lambda_rad = deg2rad(Lambda);

% Estimate the wing's pitching moment about its aerodynamic center (C_M_ac)
% This formula adjusts the 2D airfoil data for a 3D swept wing
CMac_wing = (Cm0 * AR * cos(Lambda_rad)^2) / (AR + 2 * cos(Lambda_rad));

% Calculate the required tail incidence to counteract the wing's moment
% This ensures the aircraft has a neutral or slightly positive pitching moment
I = -CMac_wing / (Vh * at);
% Convert incidence angle to degrees for the final output
% I = rad2deg(it_rad);

end
