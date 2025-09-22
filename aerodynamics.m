function [D, CDp, CDi, alpha] = aerodynamics(S,AR,t,Sh,ARh,th,Sv,ARv,tv,V,Alt);

% Please replace these "place holder" values with your own calculations.
CDp = 0.0120;   % parasite drag coefficient
CDi = 0.0140;   % induced drag coefficient
D = 15;         % total drag (in grams)
alpha = 7;      % angle of attack at the CL needed for steady, level flight