% Author: Brian Roth
% Date: Sept 22, 2022
% Course: AE 420 - Aircraft Preliminary Design
% Purpose: Main code that calls disciplinary functions
%
% Instructions:
%   This code is intended to provide the basic framework for communication
%   between disciplinary functions. In specific, it models how to send and
%   receive information. The contents of each function should be replaced
%   with your own analysis code.
%
%   Feel free to expand the information shared between functions. Any
%   changes should be coordinated with the project manager.
%
% Caution: Units are chosen for ease of interpretation. Some conversions
%   may be necessary when computing properties such as RE, L, and D.
%
% Code structure:
%   Call Controls function
%       Inputs: Wing geometry (b, S, cmac, taper)
%       Outputs: Tail sizes (S_h, AR_h, taper_h, Lt_h, Vh) for both tails
%                Neutral point
%                Control surface sizing (rudder, elevator)
%
%   Call Structures function
%       Inputs: Aircraft geometry (wing and tail sizes; fuselage length)
%       Outputs: Total weight and c.g. location
%
%   Call Aerodynamics function
%       Inputs: Wing geometry (S, b, taper)
%               Tail geometry (S, b, taper) for both tails
%               Flight velocity and altitude
%       Outputs: Drag coefficients, total drag, angle of attack (alpha)
%
%   Call Stability function
%       Inputs: CG, AR, taper, Vh, ARh, alpha (wing's angle of attack)
%       Outputs: Static margin and tail incidence angle
%
%   Call Propulsion function
%       Inputs: Flight velocity, drag, and altitude
%       Outputs: Power consumption
%
%   Call Performance function
%       Inputs: Weight, wing area, drag, power consumption
%       Outputs: Performance (TOFL, climb rate, max altitude, flight time)
%
%   Methods:
%       A variety of options exist for organizing your design choices
%       and communicating them between disciplinary design codes. Here are
%       a few options:
%           1. Make all of your variables global
%           2. Pass variables to/from subroutines, as needed
%           3. Create "objects" to organize info and simplify communication
%           4. Use a .mat file to store and recall design choices
%           5. Use an Excel file to store and recall design choices
%       For sake of simplicity, this code models option (2). However, feel 
%       free to adapt it to incorporate another option.

close all
clear variables
clc

% Design variable choices
V = 15;      % flight velocity (ft/s)
Alt = 12000; % flight altitude (ft)
b = 44;      % wing span (inches)
cavg = 6.5;  % average wing chord length (inches)
t = 0.7;     % wing taper ratio

% Calculated properties
S = b*cavg;     % wing area (in^2)
AR = b^2/S;     % wing aspect ratio
c_root = 2*S/(b*(1+t)); % wing root chord
c_tip = t*c_root;       % wing tip chord
cmac = (2/3)*c_root*(1+t+t^2)/(1+t);  % wing mean aerodynamic chord

% Call Controls function
[Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S, cmac, t, config);

% Call Structures function
[W, CG] = structures(S, AR, t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv);
%   Note: Structures and Controls will need to agree on how CG is defined.

% Call Aerodynamics function
[D, CDp, CDi, alpha] = aerodynamics(W, S, AR, t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);

% Call Stability function
[SM, I] = stability(CG, AR, t, Vh, ARh, alpha, cmac, config);

% Call Propulsion function
[P] = propulsion(V, D, Alt);

% Call Performance function
[TOFL, Climb, MaxAlt, Time] = performance(W, S, D, P);


