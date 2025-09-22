function [SM, I] = stability(CG, AR, t, Vh, ARh, alpha);

% Method of computing tail incidence
%   The first stability criteria states that CM_0 must be positive. To
%   determine its actual value, one must know two quantities:
%       1. Wing's angle of attack in steady level flight (alpha_equil)
%       2. d_CM / d_alpha --- This is a function of tail geometry and CG. 

% Please replace these "place holder" values with your own calculations.
SM = 0.10;  % static margin
I = 4;      % tail incidence angle (positive downward, in degrees)