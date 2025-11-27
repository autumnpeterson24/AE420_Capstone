function [T, W, WingArea] = DesignPoint()
% Design Point 
% F.I.S.H Flying Intercept SUAS Hook
%--------------------------------------------------------------------------
% The code inputs a few parameters about the aircraft of the minimum wing
% tip thickness, the taper ratio of the wing and the desired wing area from
% Carpet Plot code. The outputs are performance metrics of the aircraft
% along with some wing sizing data. 
% This tool sould be used as a way to size a wing quickly and verify that
% the design can meet stability and performance criteria.
%--------------------------------------------------------------------------
% INPUTS
% Wing Tip Thickness (in)
% Desired Wing Area (ft^2)
% Taper Ratio
%--------------------------------------------------------------------------
% OUTPUTS
% Wing loading
% Wing span
% Aspect Ratio
% Weight of aircraft
% TOFL (ft)
% Climb rate (ft/s)
% Endurance (hr)
% CG (ft)
%--------------------------------------------------------------------------
% CONSTANTS
%--------------------------------------------------------------------------
Cfig = 2;
Alt = 5000; %ft
AirfoilMaxThickness = .12; %Fraction of chord
V = 101; %ft/s
%--------------------------------------------------------------------------
% INPUTS
%--------------------------------------------------------------------------
WingTipThickness = 1.5; %inches
WingArea = 11; %ft^2
TaperRatio = .67;
%--------------------------------------------------------------------------
% Wing Sizing Calculations
%--------------------------------------------------------------------------
TipChord = (WingTipThickness/12)/AirfoilMaxThickness; %ft
RootChord = TipChord/TaperRatio; %ft
AverageWingChord = (TipChord + RootChord) / 2;
SingleWingSpan = (WingArea/2)/(AverageWingChord); %ft
TotalWingSpan = SingleWingSpan * 2; %ft
AR = (TotalWingSpan^2)/WingArea;
cmac = (2/3)*RootChord*(1+TaperRatio+TaperRatio^2)/(1+TaperRatio);
%--------------------------------------------------------------------------
% Aircraft Design Code
%--------------------------------------------------------------------------
b = TotalWingSpan;
S = WingArea;
t = TaperRatio;
% Call Controls function
[Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S , cmac , t, Cfig);
% Call Structures function
[W, CG] = structures_modified(S , AR , t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, Cfig,0,0);
% Call Aerodynamics function
[D, CDp, CDi, alpha] = aerodynamics(W, S , AR , t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);
% Call Stability function
[SM, I] = stability(CG, AR , t, Vh, ARh, alpha, cmac , hn, Cfig);
% Call Propulsion function
%[P] = propulsion(V, D, Alt);
% Call Performance function
T = .3*W;
[TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, D,CDp);