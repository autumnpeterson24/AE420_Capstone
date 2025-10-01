function [TOFL, Climb, MaxAlt, Time] = performance(W, S, P,V, h);
% Input variables:
%		W		        Weight of aircraft (lb)
%		S		        Wing area (ft^2)
%       P               Thrust of aircraft (lbf)
%       V    Cruise speed of aircraft (kts)
%       h               Aircraft altitude (ft)
% Output variables: 
%		TOFL            Takeoff feild length (ft)
%		Climb           Climb rate at cruise altitude (ft/min)
%       MaxAlt          Maximum altitude of aircraft (ft)
%       Time            Endurance of aircraft at cruise (h)
%--------------------------------------------------------------------------
%Constants
R = 1716.0;	 % specific gas constant for air [(ft*lb) / (slug*Rankine)]
g = 32.174;	 % acceleration due to gravity (ft/s^2)

%Baseline Constants
Cl_max = 1.4;
Cdo = [0.0313 0.0322 0.0321]; % TO 5200 6000 ft
n_total = .61; %Estimate
k_maxAlt = 3*Cdo(3)/(Cl_max^2);
%atmosphere calc
T = 518.69 - (0.00356 * h);
Pres = 2116 .* (T./ 518.6).^5.256;    % NASA
rho = Pres ./ (R.*T);

%Preliminary Calculations
V = V*1.68781; %ft/s
T_W = .3;
P = [T_W*W .86*T_W*W .84*T_W*W]; %lbf Roths T/W estimate
D = (1/2).*rho.*V^2*S.*Cdo; %lbf
Vavg = 0.7.*1.2.*sqrt(2.*W./(rho(1).*S.*Cl_max)); % average velocity
Lavg=1.*0.5.*rho(1).*Vavg.^2.*S; % average lift

% takeoff field length (ft) (Bennett)
TOFL = (1.44*W^2)/(rho(1)*S*Cl_max*g*(P(1)-D(1)-.03*(W-Lavg)));

% rate of climb (ft/min) (Bennett)
Climb_TO = V*((P(1)-D(1))/W);
Climb_Cruise = V*((P(2)-D(2))/W);
Climb_MaxAlt = V*((P(3)-D(3))/W);
Climb = Climb_Cruise;

% max altitude (ft) Needs to find altitude where 100ft/min climb is acheived (Bennett)
MaxAlt = 15000;
P_MaxAlt = 100*W/V/60 + D(3)

alts = linspace(0,40000,80);
prop_D_ft     = 22/12;
CT_fun_raw = @(J) (0.12 - 0.08.*J); 
CT_fun     = @(J) max(0.02, CT_fun_raw(J) * 1);
rho0 = 0.002377; T0 = 518.67; L = 0.00356; g = 32.174; Rgas = 1716.59; % imperial
atmo = @(hft) deal( max(T0 - L.*hft, 200), rho0 .* (max(T0 - L.*hft,200)./T0).^(g./(L.*Rgas)-1) );
[maxT_alt, rho_alt] = deal(zeros(size(alts)));
% 1) Max static thrust vs altitude (J≈0, rpm capped by hardware)
for i = 1:numel(alts)
    [~, rho]   = atmo(alts(i));
    rho_alt(i) = rho;
    rpm        = 4218;         % conservative back-EMF cap
    n          = rpm/60;             % rev/s
    J          = 0;                  % static
    CT         = CT_fun(J);
    % Dimensional consistency in imperial prop theory:
    maxT_alt(i) = CT * rho * (n^2) * (prop_D_ft^4);   % lbf
    if maxT_alt >= P_MaxAlt
        MaxAlt = alts(i);
        break;
    end
end

%needs work with prop team to find the altitude

% flight time (hours) https://www.researchgate.net/publication/269567470_Range_and_Endurance_Estimates_for_Battery-Powered_Aircraft
Rt = 1; %Battery hour rating (typically 1 according to Traub for small rechargable battery packs)
n = 1.3; %Battery discharge parameter (taken from Traub's paper saying 1.3 is common for lithium polymer batteries, li ion is closer to 1.05 for reference according to AI)
V = 11.11; %Battery Voltage (Payloads team's value)
C = 56; %Battery capacity in amp hours (Guess based off of similar applications and prop teams estimate of 5000mAh)
k = 3*Cdo(2)/(Cl_max^2); % Sovled from Cdo = (1/3)k*CL^2 

Time = (Rt^(1-n)) * ((n_total*V*C)/((2/sqrt(rho*S))*(Cdo(2)^.25)*(2*W*sqrt(k/3))^1.5))^n; %hours
Time2 = (Rt^(1-n)) * ((n_total*V*C)/(.5*rho*V^3*S*Cdo(2) + 2*W^2*k/(rho*V*S)))^n; %hours

%Code Outputs
fprintf("Takeoff field length              = %4.2f (ft)\n",TOFL);
fprintf("Climb rate at takeoff             = %4.2f (ft/s) or %4.2f (ft/min)\n", Climb_TO,Climb_TO*60);
fprintf("Climb rate at cruise              = %4.2f (ft/s) or %4.2f (ft/min)\n", Climb_Cruise,Climb_Cruise*60);
fprintf("Climb rate at max altitude        = %4.2f (ft/s) or %4.2f (ft/min)\n", Climb_MaxAlt,Climb_MaxAlt*60);
fprintf("Thrust required for max altitude  = %4.2f (lbf)\n",P_MaxAlt);
fprintf("Maximum Altitude                  = %4.4f (ft)\n",MaxAlt);
fprintf("Max Endurance                     = %4.2f (h)\n",Time);
fprintf("Cruise Endurance                  = %4.2f (h)\n",Time2);

%--------------------------------------------------------------------------
%NOTE

%With current aircraft configuration, the takeoff distance was prioritized
%to be under 350ft which drives a thrust requirement. That thrust needed to
%get the takeoff distance is then used in the climb rate calculation which
%provides the 900-1100 ft/min climb rates at various altitudes. GA
%requirement's state a climb rate of 400-500 ft/min was our target which
%seems impossible to achieve both requirements so the takeoff distance was
%prioritized as we can out perform climb rates if needed