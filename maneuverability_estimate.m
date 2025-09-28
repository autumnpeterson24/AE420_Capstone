function [Q] = maneuverability_estimate(x_cg,L,W,alpha,V,alt,AR,S,t,Lambda,Lw,ARh,Sh,Lambda_h,Lh,delta_e)
% Author: Trey Talko
% Date: 9/22/2025
% Purpose: This model should provide an estimate for the achievable pitch
% and roll rates of an aircraft.
%
% All units are in base units of feet (ft), slugs, pounds (lbf), and
% seconds (s).
%
% Nomenclature

%% Inputs
% overall configuration inputs
% x_cg - x location of the aircraft cg relative to the nose (ft)
% L - overall length of the aircraft (ft)
% W - overall weight of the aircraft (lb)
% alpha - Angle of attack at condition (degrees)
% V - airspeed (ft/s)
% alt - altitude (ft)

% main wing inputs
% AR - aspect ratio of the main wing
% S - reference area of main wing (ft^2)
% t - main wing taper ratio
% Lambda - sweep of main wing (degrees)
% Lw - Wing moment arm (ft)

% Tail Inputs
% ARh - aspect ratio of the horizontal tail
% Sh - Reference area of the tail (ft^2)
% Lambda_h - sweep of horizontal tail (degrees)
% Lh - Tail moment arm

% elevator inputs
% delta_e - elevator deflection angle (degrees)

%% Givens
eta = 0.9; % Approximate tail efficiency factor
g = 32.2; % acceleration of gravity (ft/s^2)
Ry = 0.38; % nondimensionalized radius of gyration for single-prop aircraft
h_flap_percentage = 30; % standard flap percentage for fighter aircraft
alpha = deg2rad(alpha);

%% Calculated Geometries
% Main Wing
b = sqrt(AR*S); % wingspan (ft)
c_root = 2*S/(b*(1+t)); % wing root chord (ft)
cmac = (2/3)*c_root*(1+t+t^2)/(1+t);  % wing mean aerodynamic chord (ft)

% tail
S_fh = h_flap_percentage/100*Sh; % elevator flap area (ft^2)

%% Pulling atmospheric data from atmospheric data function
[windspeed, P, rho, T, mu, nu, mach, Re, q, a] = ...
    atmosphere(alt, V);

%% Approximating Downwash Effects
deda = Downwash_on_Tail(AR,b,t,Lh);

%% Approximating the Lift Curve Slope and Effects of Horizontal Tail Flap
% Deflection
Beta = sqrt(1-mach^2); % EQ 1
a = ((2*pi*AR)/ ...
    (2+sqrt(4+AR^2*Beta^2*(1+tand(Lambda)^2/Beta^2)))); % EQ 2
a_h = ((2*pi*ARh)/ ...
    (2+sqrt(4+ARh^2*Beta^2*(1+tand(Lambda_h)^2/Beta^2)))); % EQ 2
Delta_Clmaxh = 0.9; % Airfoil Lift Increment Due to Flap Deflection 
% Approximated as 0.9 as detailed in table 12.2 of Ref 1 
Delta_CLmaxh = Delta_Clmaxh*(S_fh/Sh)*cosd(Lambda_h)*.8; % EQ 3
Delta_a0Lh = -Delta_CLmaxh/a_h; % EQ 4
% Effective result reduced by 20% from assuming that the elevator gap is
% not sealed as detailed on pg 418 of Ref 1.

%% Calculating C_mw
C_mw = C_m0_foil*((AR*cosd(Lambda)^2)/(AR+2*cos(Lambda)))-a*alpha*Lw; % EQ 5

%% Calculating C_malpha
C_malpha_wing = -a*(x_ac-x_cg)/cmac;
C_malpha_h = -eta*a_h*(1-deda*alpha)*(Sh*Lh)/(S*cmac);
C_malpha = C_malpha_wing+C_malpha_h;

%% Calculating Pitch Damping Derivative
C_mq = -2*eta*a_h*((Sh*Lh^2)/(S*cmac^2))*(1/(2*V/cmac));

%% Calculating Pitch Moment Derivative due to Elevator Deflection
delta_e_rad = deg2rad(delta_e);
tau_e = Delta_a0Lh/delta_e_rad;
C_mdelta = -eta*a_h*tau_e*((Sh*Lh)/(S*cmac));

%% Calculating Total Aerodynamic Moment
C_m = C_mw+C_malpha+C_mq;
M_aero = q*S*cmac*C_m;

%% Approximating Pitch Mass Moment of Inertia
Iyy = (L^2*W*Ry^2)/(4*g);

%% Estimating Instantaneous Pitch Rate
% assume Q_dot = Q for small t
M_delta = q*S*cmac*C_mdelta*delta_e;
Q = M_delta/Iyy; % approximate pitch rate in degrees/s
fprintf('Predicted pitch rate is %.2f degrees per second',Q)

%% References
% 1 - Author, Aircraft Design - A Conceptual Approach
    % EQ 1 = (12.7)
    % EQ 2 = (12.6)
    % EQ 3 = (12.21)
    % EQ 4 = (16.13)
    % EQ 5 = (16.19)