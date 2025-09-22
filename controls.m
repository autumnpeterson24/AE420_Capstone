function [Sh,ARh,th,Lh,Vh,Sv,ARv,tv,Lv,hn] = controls(b, S, cmac, t)

% Approach:
% 1. Define AR and taper for the horizontal and vertical tails
% 2. Choose VH from guidelines in literature (i.e., 0.35 < Vh < 0.50)
%       Similarly, 0.02 < Vv < 0.04
% 3. Choose L_HT and L_VT as percentage of wing span (based on competitors)
% 4. Calculate Sh and Sv (from definitions of Vh and Vv)
% 5. Calculate rudder and elevator sizes
% 6. Calculate neutral point
%       Note: d_epsilon/d_alpha is needed. de/da is the variation in 
%           downwash angle with change in the wing's angle of attack. 
%       Source: Curve fit to data from Roskam and DATCOM
%       Limitation: Assumes that bh = 0.4*b. (Otherwise, an approximation.)

% Please replace these "place holder" values with your own calculations.
Sh = 30;    % area of horizontal tail (in^2)
ARh = 2.5;  % aspect ratio of horizontal tail
th = 0.8;   % taper ratio of horizontal tail
Lh = 24;    % moment arm of horizontal tail (inches)
Vh = 0.45;  % horizontal tail voiume coefficient

Sv = 18;    % area of vertical tail (in^2)
ARv = 1.3;  % aspect ratio of vertical tail
tv = 0.8;   % taper ratio of vertical tail
Lv = 24;    % moment arm of vertical tail (inches)

hn = 0.8;   % neutral point

% Calculations (please retain for your code)
AR = b^2/S; % aspect ratio of wing
%   Use Lh as an approx of the distance from c/4 of wing to c/4 of tail
deda = Downwash_on_Tail(AR,b,t,Lh);
