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

% Author: Brian Roth
% Date: Sept 22, 2022
% Course: AE 420 - Aircraft Preliminary Design
% Purpose: Main code that calls disciplinary functions
%
% Modified to keep S constant and vary AR

close all
clear variables
clc

% Design variable choices
Alt = 5000; % flight altitude (ft)
S = 10;      % wing area (ft^2) - CONSTANT
t = 0.7;     % wing taper ratio
Cfig = 2;    % Config Selection

% Loop setup
tic
T = 8.75; % Thrust (lbf) variable
AR_values = [5, 8, 10, 12, 20]; % Different aspect ratios to test
colors = ['y','b', 'g', 'r', 'k']; % Colors for different AR curves

figure;
hold on;
grid on;

for ar_idx = 1:length(AR_values)
    AR = AR_values(ar_idx);
    
    % Vary velocity
    V_sweep = linspace(50.6, 118.1, 100); % Sweep velocity from 50.6 to 118.1 ft/s
    
   
    E_array = zeros(1, 100);
    
    for jj = 1:100
        % Calculate wing geometry based on fixed S and AR
        V = V_sweep(jj);        % Current velocity
        b = sqrt(AR * S);       % Calculate span from AR and S
        cavg = S / b;           % average chord
        c_root = 2*S/(b*(1+t)); % wing root chord
        c_tip = t*c_root;       % wing tip chord
        cmac = (2/3)*c_root*(1+t+t^2)/(1+t);  % wing mean aerodynamic chord
        

            % Call Controls function
            [Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S, cmac, t, Cfig);
            
            % Call Structures function
            [W, CG] = structures(S, AR, t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, Cfig);
            
            % Call Aerodynamics function
            [D, CDp, CDi, alpha] = aerodynamics(W, S, AR, t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);
            
            % Call Stability function
            [SM, I] = stability(CG, AR, t, Vh, ARh, alpha, cmac, Cfig);
            
            % Call Propulsion function
            [P] = propulsion(V, D, Alt);

            % Call Performance function
            [TOFL, Climb, MaxAlt, Time] = performance(W, S, T, V, Alt, AR);
           
        
           % T_end = T(jj);
            Time_end = Time;

            % Store results
            E_array(jj) = Time_end;
            
            fprintf("AR = %4.2f, V = %4.2f ft/s,  E = %4.2f h\n", ...
                    AR, V, Time_end);
    end
    
    % Plot for this AR value (remove zeros if any)
    valid_idx = E_array > 0;
    plot(V_sweep(valid_idx), E_array(valid_idx), [colors(ar_idx) '-o'], ...
         'LineWidth', 2, 'DisplayName', sprintf('AR = %d', AR_values(ar_idx)));
end

% Finalize plot
xlabel('V (ft/s)', 'FontSize', 12);
ylabel('E (hours)', 'FontSize', 12);
title('Endurance vs  Velocity ', 'FontSize', 14);
legend('show', 'Location', 'best');
hold off;

toc
