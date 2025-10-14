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

%% 1. AIRCRAFT CONFIGURATION DATABASE
% --- Each column represents a different aircraft configuration ---
% --- Please replace these placeholder values with your actual data ---



% Horizontal Stabilizer Data
Lh_vec = [4.033, 2.478, 2.964, 4.000, 2.300]; % Moment arm (c/4 wing to c/4 tail)
bh_vec = [2.742, 0.4*b, 2.972, 0.3*b 3.300]; % Span
th_vec = [1.000, 1.000, 0.702, 1.000, 0.600]; % Taper ratio
% Vh_vec = [ 0.620, 0.875, 1.916, 1.073]; % Volume Coefficient of Horizontal Tail

% Vertical Stabilizer Data
Lv_vec = [4.033, 2.478, 4.890, 4.000, 2.300]; % Moment arm (c/4 wing to c/4 tail)
bv_vec = [0.792, 0.3*b, 0.886, 0.15*b, 1.967]; % Span
tv_vec = [1.000, 1.000, 0.560, 1.000, 0.500]; % Taper ratio
% Vv_vec = [0.005 , 0.030, 0.0003, 0.027]; % Volume Coefficient of Vertical Tail

% Wing Location Data
Lw_vec = [4.000, 2.889, 1.100, 1.275 2.5]; % x-location of the wing's leading edge

%% 2. SELECT ACTIVE CONFIGURATION
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

%% 3. SIZING CALCULATIONS
% --- Main Wing Sizing ---
AR = b^2 / S;
c_root = 2*S/(b*(1+t)); % wing root chord
c_tip = t*c_root;       % wing tip chord
Lambda_w = atan((c_root-c_tip)/b); % Leading edge sweep angle (radians)

% --- Tail Area Sizing (from Volume Coefficients) ---
% Vh = (Sh * Lh) / (S * cmac) -> Rearranged for Sh
Sh = (Vh * S * cmac) / abs(Lh);
c_root_h = 2*Sh/(bh*(1+th)); % wing root chord
c_tip_h = th*c_root_h;       % wing tip chord
cmach = (2/3)*c_root_h*(1+th+th^2)/(1+th);  % wing mean aerodynamic chord
Lambda_h = atan((c_root_h-c_tip_h)/bh); % Leading edge sweep angle (radians)

% Vv = (Sv * Lv) / (S * b) -> Rearranged for Sv
Sv = (Vv * S * b) / abs(Lv);

% --- Tail Aspect Ratios ---
ARh = bh^2 / Sh;
ARv = bv^2 / Sv;

% --- Control Surface Sizing (based on Raymer's guidelines) ---
aileron_span  = 0.4 * b;  % Ailerons span ~40% of the wing (e.g., from 0.5b to 0.9b)
elevator_span = 0.9 * bh; % Elevator span is ~90% of horizontal tail span
rudder_span   = 0.9 * bv; % Rudder span is ~90% of vertical tail span

%% 4. NEUTRAL POINT ESTIMATION
[~, ~, ~, ~, ~, ~, M, ~, ~, ~] = atmosphere(5500, 100);
eta_h = 0.9; % approximate dynamic pressure ratio from freestream to tail

% 3D Lift-Curve slope estimation using polhamus equation
kw = 1+((8.2-2.3*Lambda_w)-AR*(0.22-0.153*Lambda_w))/100;
kh = 1+(AR*(1.87-0.000233*Lambda_h))/100;

a_w = (2*pi*AR)/(2+sqrt(((AR^2*(1-M^2))/kw^2*(1+tan(Lambda_w)^2/(1-M^2)))+4));
a_h = (2*pi*ARh)/(2+sqrt(((ARh^2*(1-M^2))/kh^2*(1+tan(Lambda_h)^2/(1-M^2)))+4));

% Downwash Calculation (d_epsilon / d_alpha)
% Note: This requires a separate function 'Downwash_on_Tail' to be defined
% and available in your MATLAB path.
deda = Downwash_on_Tail(AR, b, t, Lh);

% Aerodynamic center of the wing/body (assumed at quarter-chord of MAC)
% taken from nose of the aircraft
XAC = (0.25 * cmac) + Lw;
xAC = XAC/cmac; % non-dimensionalized value w.r.t. cmac

% Aerodynamic center of the horizontal tail (assumed at quarter-chord of MAC)
% taken from nose of the aircraft

XACh = (0.25 * cmach) + Lh;
xACh = XACh/cmac; % non-dimensionalized value w.r.t. cmac

% Calculating the neutral point of the aircraft
h_num = xAC+a_h/a_w*eta_h*Sh/S*(1-deda)*xACh;
h_den = 1+a_h/a_w*eta_h*Sh/S*(1-deda);
hn = h_num/h_den*cmac; % aircraft neutral point w.r.t. the nose (ft)
end
