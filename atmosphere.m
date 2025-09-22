% Author: Brian Roth
% Date: Sept 22, 2022
% Course: AE 420 - Aircraft Preliminary Design
% Purpose: Provide atmospheric properties

function [windspeed, P, rho, T, mu, nu, mach, Re, q, a] = atmosphere(altitude, velocity)

% Input variables:
%		altitude		Geometric altitude (ft)
%		velocity		Flight velocity (airspeed) (ft/s)
%
% Output variables: 
%		windspeed       Windspeed (ft/s)
%		P				Pressure(lb/ft^2)
%		rho             Density (slugs/ft^2)
%		T				Temperature (deg Rankine)
%		mu				Viscosity (lb*s / ft^2)
%		nu				Kinematic viscosity (ft^2/s)
%		mach			Mach number
%		Re				Reynolds number per ft (1/ft)
%		q				Dynamic pressure (lb/ft^2)
%       a               Speed of sound (ft/s)
%--------------------------------------------------------------------------

R = 1716.0;	 % specific gas constant for air [(ft*lb) / (slug*Rankine)]
g = 32.174;	 % acceleration due to gravity (ft/s^2)

% ATMOSPHERIC CONDITIONS  (www.grc.nasa.gov/WWW/K-12/airplane/atmos.html)

% Troposphere
if (altitude <= 36152)
   T = 518.69 - (0.00356 * altitude);
   P = 2116 * [T/ 518.6]^5.256;    % NASA
   
% Lower Stratosphere
elseif ( (altitude > 36152) & (altitude <= 82345) )
   T = 390.0;
   P = 473.1 * exp(1.73 - 0.000048 * altitude) ;
   
% Upper Stratosphere
else
   T = -205.05 + 0.00164 * altitude + 459.7;    % deg F
   P = 51.97 * (T/389.98)^(-11.388);    % lb/ft^2
end

rho = P / (R*T);
mu = 0.3170E-10 * T^1.5 * (734.7 / (T + 216.0));
nu = mu / rho;

% FLIGHT SPEED PROPERTIES
a = sqrt(1.4 * R * T);
mach = velocity / a;
q = 0.5 * rho * velocity^2;
Re = velocity / nu;  % Reynolds number per foot

% WINDSPEED CALCULATION
location = 1;   % 1 - Arizona in December
                % 2 - Oakland in July
if location == 1
% Windspeed data for Fort Huachuca, Arizona in December
%	Units: (m/s, km)
WIND = [0.00 2.93 3.50 3.87 4.39 4.94 5.73 6.37 7.17 7.83 8.60 9.29 10.05 10.77 11.49 12.22 12.85 13.58 14.22 14.86 15.55 16.20 16.89 17.56 18.17 18.77 19.35 19.98 20.60 21.15 21.80 22.44 23.12 23.74 24.28 24.82 25.37 25.86 26.26 26.70 27.16 27.64 27.97 28.10 28.25 28.38 28.48 28.57 28.58 28.24 27.96 27.67 27.23 26.65 26.04 25.46 24.82 24.10 23.40 22.71 21.95 21.07 20.20 19.35 18.50 17.63 16.73 15.77 14.83 13.90 13.01 12.14 11.30 10.54  9.80  9.14  8.56  8.06  7.61  7.15  6.79  6.52  6.37  6.24  6.15  6.12  6.18  6.33  6.49  6.53  6.62  6.75  6.93  7.16  7.43  7.65  7.81  8.06  8.36  8.67  8.91  9.13  9.44  9.79 10.21 10.57 11.09 11.43 11.87 12.34 12.83 13.37 13.81 14.21 14.48 14.78 15.12 15.45 15.86 16.43 17.01 17.60 18.20;
   		0.00 0.25 0.50 0.75 1.00 1.25 1.50 1.75 2.00 2.25 2.50 2.75 3.00  3.25  3.50  3.75  4.00  4.25  4.50  4.75  5.00  5.25  5.50  5.75  6.00  6.25  6.50  6.75  7.00  7.25  7.50  7.75  8.00  8.25  8.50  8.75  9.00  9.25  9.50  9.75  10.00 10.25 10.50 10.75 11.00 11.25 11.50 11.75 12.00 12.25 12.50 12.75 13.00 13.25 13.50 13.75 14.00 14.25 14.50 14.75 15.00 15.25 15.50 15.75 16.00 16.25 16.50 16.75 17.00 17.25 17.50 17.75 18.00 18.25 18.50 18.75 19.00 19.25 19.50 19.75 20.00 20.25 20.50 20.75 21.00 21.25 21.50 21.75 22.00 22.25 22.50 22.75 23.00 23.25 23.50 23.75 24.00 24.25 24.50 24.75 25.00 25.25 25.50 25.75 26.00 26.25 26.50 26.75 27.00 27.25 27.50 27.75 28.00 28.25 28.50 28.75 29.00 29.25 29.50 29.75 30.00 30.25 30.50];

% Convert altitude from (ft) to (km)
SI_altitude = altitude*0.0003048;

% Interpolate to find the windspeed (in m/s)
temp1 = floor(SI_altitude*4 + 1);
temp2 = ( SI_altitude - WIND(2,temp1) ) / ( WIND(2,temp1+1) - WIND(2,temp1) );
SI_windspeed = WIND(1,temp1) + temp2*(WIND(1,temp1+1) - WIND(1,temp1));

% Convert windspeed from (m/s) to (ft/s)
windspeed = SI_windspeed * 3.28084;

elseif location == 2
% Windspeed data for Oakland, CA in July --	Units: (kts, ft)
WIND = [0.0 3.7 6.0  7.4  8.5  9.3 10.0 10.6 11.3 11.9 12.3  12.7  12.9  13.4  14.2  15.5  17.2  19.7  21.4  27.0  33.0  36.0  40.0  43.1  33.4  24.8  12.8  11.5  12.9  17.5  23.2  27.8  29.0  32.3  35.0;
        0.0 10 1000 2000 3000 4000 5000 6000 7000 8000 9000 10500 12000 14000 16000 18000 20000 23000 25000 30000 35000 37500 40500 44000 47000 50000 55000 58000 62000 70000 80000 88000 90000 96000 100001];

% Interpolate to find the windspeed
i = 1;
while (altitude - WIND(2,i)) >= 0.0,
    i = i+1;
end
temp1 = i-1;  % the altitude in the table that's just below the current altitude
temp2 = (altitude - WIND(2,temp1) ) / ( WIND(2,temp1+1) - WIND(2,temp1) );
windspeed = WIND(1,temp1) + temp2*(WIND(1,temp1+1) - WIND(1,temp1));
windspeed = 1.68781 * windspeed;    % convert from knots to ft/s

else
    'See Atmosphere subroutine to update wind profiles'
    return
end

scale_factor = 1.0;
windspeed = scale_factor * windspeed;

return