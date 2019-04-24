% -*- mode: matlab -*-
%
% compute the temperature, density and pressure ratios for
% altitudes up to 20 km for the International Standard Atmosphere
%
% Usage:    [theta, delta, sigma] = int_std_atm(h)
%
% Input:    h      pressure altitude, m
%
% Outputs:  T       temperature at altitude
%           rho     density     "     "
%           p       pressure    "     "
%           a       speed of sound at altitude
%           mu      kinematic viscosity
%          
%
%           delta  ratio of pressure    to sea level pressure
%           sigma    "    " density     "   "    "   density
%           theta    "    " temperature "   "    "   temperature

%
% Notes:  formulas taken from Appendix 2 of W. Austin Mair and
%         David L. Birdsall, _Aircraft_Performance_, Cambridge
%         Aerospace Series No. 5.  Cambridge: Cambridge Univ.
%         Press, 1992.

function [T, p, rho, a, mu, delta, sigma, theta] = int_std_atm(h_in, flight_condition)

h   = h_in ./ 1000;  %Convert m input to km

T0  = 288.15;   % K, sea level standard temperature
g   = 9.80665;  % m/s^2, gravitational acceleration
R   = 287.053;  % J/kg K, gas constant
p0  = 1.0132e5; %Pa, sea level standard pressure
rho0= 1.225;    %kg/m^3, sea level standard density
mu0 = 1.78e-5; %sea level

gam = 1.4;      %specific hear raio of air

T1 = 216.65;  % K, constant temperature above 11 km
Tsuth = 110.0;

%Correct for hot or cold days
switch flight_condition
  case 'std day'
    dT = 0;
  case 'hot day'
    dT = 15;
    %airplane.environment.dT = 29.9;  % 95 deg F at 5,000 ft MSL
  case 'cold day'
    dT = -15;
end

% compute the temperture, pressure, and density ratios

theta_o   = max(1 - 6.5*h/T0, T1/T0);
theta = theta_o + dT/T0;
delta   = (theta_o).^5.256.*exp(-1000*g*(max(h,11)-11)/(R*T1));
sigma   = delta./theta;
a       = (gam .* delta.*p0./(rho0.*sigma)).^.5;
mu  = mu0 * sqrt(theta_o)^3.0 * (T0 + Tsuth) / (theta_o*T0 + Tsuth);
rho = sigma*rho0;
T   = theta*T0;
p   = delta*p0;
end
