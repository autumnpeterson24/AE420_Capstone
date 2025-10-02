function [Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S, cmac, t, config)
% REVISON 2.4 (9/24/25)
%
% This function calculates the size, geometry, and key stability parameters for
% an aircraft's horizontal and vertical tails based on a selected configuration.
%
% --- Approach ---
% 1. Select a design configuration (1-4).
% 2. Define geometric parameters (AR, taper ratio) for tails.
% 3. Use typical volume coefficients (Vh, Vv) from literature.
% 4. Estimate tail moment arms (Lh, Lv) based on wing span.
% 5. Calculate required tail areas (Sh, Sv) from volume coefficient definitions.
% 6. Estimate control surface spans (aileron, elevator, rudder).
% 7. Calculate the aircraft's neutral point (hn).
%
% --- Inputs ---
%   b      - Wing span
%   S      - Wing reference area
%   cmac   - Wing mean aerodynamic chord
%   t      - Wing taper ratio
%   CG     - Center of Gravity location (longitudinal)
%   config - Configuration number to use for the analysis (1, 2, 3, or 4)
%
% --- Outputs ---
%   Sh     - Area of horizontal tail
%   ARh    - Aspect ratio of horizontal tail
%   th     - Taper ratio of horizontal tail
%   Lh     - Moment arm of horizontal tail
%   Vh     - Horizontal tail volume coefficient
%   Sv     - Area of vertical tail
%   ARv    - Aspect ratio of vertical tail
%   tv     - Taper ratio of vertical tail
%   Lv     - Moment arm of vertical tail
%   hn     - Neutral point location (non-dimensional, fraction of cmac)
%

%% 1. AIRCRAFT CONFIGURATION DATABASE
% --- Each column represents a different aircraft configuration ---
% --- Please replace these placeholder values with your actual data ---

% Horizontal Stabilizer Data
Lh_vec = [-2.800, 4.033, 2.853, 2.964]; % Moment arm (c/4 wing to c/4 tail)
bh_vec = [ 4.000, 2.742, 5.615, 2.972]; % Span
th_vec = [ 0.400, 1.000, 1.000, 0.702]; % Taper ratio
% Vh_vec = [ 0.620, 0.875, 1.916, 1.073]; % Volume Coefficient of Horizontal Tail

% Vertical Stabilizer Data
Lv_vec = [0.201 , 4.033, 0.0100, 4.89]; % Moment arm (c/4 wing to c/4 tail)
bv_vec = [2.828 , 0.792, 1.7930, 0.886]; % Span
tv_vec = [0.364 , 1.000, 0.5880, 0.560]; % Taper ratio
% Vv_vec = [0.005 , 0.030, 0.0003, 0.027]; % Volume Coefficient of Vertical Tail

% Wing Location Data
Lw_vec = [4.958, 2.746, 2.685, 1.1];     % x-location of the wing's leading edge

%% 2. CONSTANTS & ASSUMPTIONS
% --- These values are assumed to be constant across all configurations ---
a0  = 0.0972;   % 2D lift curve slope for wing airfoil (per radian)(BOE 103)
a0t = 0.113575; % 2D lift curve slope for tail airfoil (per radian)(NACA 0015)

%% 3. SELECT ACTIVE CONFIGURATION
% --- Extracts the data for the chosen 'config' number ---
Lh = Lh_vec(config);
bh = bh_vec(config);
th = th_vec(config);
Vh  = 0.4;

Lv = Lv_vec(config);
bv = bv_vec(config);
tv = tv_vec(config);
Vv  = 0.04;

Lw = Lw_vec(config);

%% 4. CALCULATIONS
% --- Main Wing Aspect Ratio ---
AR = b^2 / S;

% --- Tail Area Sizing (from Volume Coefficients) ---
% Vh = (Sh * Lh) / (S * cmac) -> Rearranged for Sh
Sh = (Vh * S * cmac) / abs(Lh);
disp(Sh)
% Vv = (Sv * Lv) / (S * b) -> Rearranged for Sv
Sv = (Vv * S * b) / abs(Lv);
disp(Sv)

% --- Tail Aspect Ratios ---
ARh = bh^2 / Sh;
ARv = bv^2 / Sv;

% --- Control Surface Sizing (based on Raymer's guidelines) ---
aileron_span  = 0.4 * b;  % Ailerons span ~40% of the wing (e.g., from 0.5b to 0.9b)
elevator_span = 0.9 * bh; % Elevator span is ~90% of horizontal tail span
rudder_span   = 0.9 * bv; % Rudder span is ~90% of vertical tail span

% --- Neutral Point Calculation ---
% Convert 2D lift curve slopes to 3D using lifting-line theory
a  = a0 / (1 + (a0 / (pi * AR)));
at = a0t / (1 + (a0t / (pi * ARh)));

% Downwash Calculation (d_epsilon / d_alpha)
% Note: This requires a separate function 'Downwash_on_Tail' to be defined
% and available in your MATLAB path.
deda = Downwash_on_Tail(AR, b, t, Lh);

% Aerodynamic center of the wing/body (assumed at quarter-chord of MAC)
%corrected to be taken from nose of the aircraft
xAC = (0.25 * cmac) + Lw;

% Non-dimensionalize key longitudinal distances by the MAC
% h       = (Lw - CG) / cmac;   % Non-dimensional CG location
h_ac_wb = (Lw - xAC) / cmac;  % Non-dimensional Aerodynamic Center of wing/body

% Calculate the neutral point
hn = h_ac_wb + Vh * (at / a) * (1 - deda);

end
