%%%%%%%%%%%%%%%%%%%%
%AeroModelSSTOL Test
%%%%%%%%%%%%%%%%%%%%

hft         =   3000;   % Altitude above Sea Level, ft
VKIAS       =   50;     % Indicated Airspeed, kt

hm          =   hft * 0.3048;    % Altitude above Sea Level, m
VmsIAS      =   VKIAS * 0.5154;  % Indicated Airspeed, m/s

disp('  ')
disp('Initial Conditions')
disp('==================')
disp(['Altitude           = ',num2str(hft),' ft,   = ',num2str(hm),' m'])
disp(['Indicated Airspeed = ',num2str(VKIAS),' kt, = ',num2str(VmsIAS),' m/s'])

    
%   US Standard Atmosphere, 1976, Table Lookup for I.C.    
[temp,airPres,airDens,soundSpeed] = int_std_atm(hm);
disp('  ')
disp(['Air Density     = ',num2str(airDens),' kg/m^3, Air Pressure = ',num2str(airPres),' N/m^2'])
disp(['Air Temperature = ',num2str(temp),' K,         Sound Speed  = ',num2str(soundSpeed),' m/s'])

qBarSL  =   0.5*1.225*VmsIAS^2;  % Dynamic Pressure at sea level, N/m^2
V       =   sqrt(2*qBarSL/airDens);	% True Airspeed, TAS, m/s
TASms   =   V;
disp('  ')
disp(['Dynamic Pressure = ',num2str(qBarSL),' N/m^2, True Airspeed = ',num2str(V),' m/s'])


alpha   =	20;      % Angle of attack, deg (relative to air mass)
beta    =	0;      % Sideslip angle, deg (relative to air mass)
dA      =	0;      % Aileron angle, deg
dE      =	5;   % Elevator angle, deg
dR      =	0;      % Rudder angle, deg
dF_L    =   40;     % Flap angle, deg
dF_R    =   40;     % Flap angle, deg
dB_L    = 	.1;    % Left Blower throttle setting, % / 100
dB_R    = 	.1;    % Right Blower throttle setting, % / 100
dT_L    = 	0.0;    % Left Cruiser throttle setting, % / 100
dT_R    = 	0.0;    % Right Cruiser throttle setting, % / 100
hdot    =	0;      % Altitude rate, m/s
p       =	0;      % Body-axis roll rate, deg/s
phi     =	0;      % Body roll angle wrt earth, deg
psi     =	0;      % Body yaw angle wrt earth, deg
q       =	0;      % Body-axis pitch rate, deg/sec
r       =	0;      % Body-axis yaw rate, deg/s
theta   =	10;  % Body pitch angle wrt earth, deg [theta = alpha if hdot = 0]
xe      =	0;      % Initial longitudinal position, m
ye      = 	0;      % Initial lateral position, m
ze      = 	-hm;    % Initial vertical position, m [h: + up, z: + down]

%construct the state vector
[x0,u0]=constructStateandControlVector(alpha,beta,dA, dE, dR,dF_L,dF_R, dB_L,dB_R, dT_L,dT_R, hdot, p,q,r, phi,theta, psi, xe,ye,ze,V,soundSpeed);

%% find trim state
if 1
    inputs0 = [alpha, dE, dB_L]; %% assumes we want equal left and right blowing 
    [x,u, trimmed_inputs] = calculate_trimmed_state(inputs0,x0, u0, airplane);
end

disp('Initial inputs')
inputs0'
disp('Trimed Conditions')
trimmed_inputs'
   