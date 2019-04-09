function [norm_body_force, x, u] = calculate_body_force(inputs,airplane)

%%
CL          = 3;

hft         =  1000;   % Altitude above Sea Level, ft
VKIAS       =  15;     % Indicated Airspeed, kt

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

%VmsIAS=sqrt(2*airplane.weights.MTOW/(airDens*airplane.geometry.Wing.S*CL))

qBarSL  =   0.5*1.225*VmsIAS^2;  % Dynamic Pressure at sea level, N/m^2
V       =   sqrt(2*qBarSL/airDens);	% True Airspeed, TAS, m/s
TASms   =   V;
disp('  ')
disp(['Dynamic Pressure = ',num2str(qBarSL),' N/m^2, True Airspeed = ',num2str(V),' m/s'])


alpha   =	inputs(1);      % Angle of attack, deg (relative to air mass)
beta    =	0;      % Sideslip angle, deg (relative to air mass)
dA      =	0;      % Aileron angle, deg
dE      =	inputs(2);      % Elevator angle, deg
dR      =	0;      % Rudder angle, deg
dF_L    =   40;     % Flap angle, deg
dF_R    =   40;     % Flap angle, deg
dB_L    = 	inputs(3);    % Left Blower throttle setting, % / 100
dB_R    = 	inputs(3);    % Right Blower throttle setting, % / 100
dT_L    = 	inputs(4);    % Left Cruiser throttle setting, % / 100
dT_R    = 	inputs(4);    % Right Cruiser throttle setting, % / 100
hdot    =	0;      % Altitude rate, m/s
p       =	0;      % Body-axis roll rate, deg/s
phi     =	0;      % Body roll angle wrt earth, deg
psi     =	0;      % Body yaw angle wrt earth, deg
q       =	0;      % Body-axis pitch rate, deg/sec
r       =	0;      % Body-axis yaw rate, deg/s
theta   =	alpha+atan(hdot/sqrt(V^2-hdot^2))/0.01745329;  % Body pitch angle wrt earth, deg [theta = alpha if hdot = 0]
xe      =	0;      % Initial longitudinal position, m
ye      = 	0;      % Initial lateral position, m
ze      = 	-hm;    % Initial vertical position, m [h: + up, z: + down]

Mach	= 	V / soundSpeed;	
gamma	=	57.2957795 * atan(hdot / sqrt(V^2 - hdot^2));
disp(['Mach number      = ',num2str(Mach),', Flight Path Angle = ',num2str(gamma),' deg'])										
disp('  ')

phir	=	phi * 0.01745329;
thetar	=	theta * 0.01745329;
psir	=	psi * 0.01745329;

alphar	=	alpha * 0.01745329;
betar	=	beta * 0.01745329;

%construct the state vector
[x,u]=constructStateandControlVector(alpha,beta,dA, dE, dR,dF_L,dF_R, dB_L,dB_R, dT_L,dT_R, hdot, p,q,r, phi,theta, psi, xe,ye,ze,V,soundSpeed);

    
[CX,CL,CY,Cl,Cm,Cn]	=	AeroModelSSTOL(x,u,Mach,alphar,betar,V);

% this is the coefficients in stability axes. 
%Compute the gravitational force contribution

mg = angle2dcm(-psir, -thetar+alphar, -phir) * [0;0;airplane.weights.MTOW];

body_force = [CX; CY; -CL]*(0.5*airDens*V^2*airplane.geometry.Wing.S) + mg;

body_moments = [Cl; Cm; Cn]*(0.5*airDens*V^2*airplane.geometry.Wing.S*airplane.geometry.Wing.cbar);

disp('body force:')
disp(body_force)
disp('body_moments:')
disp(body_moments)

norm_body_force = norm([body_force;body_moments]);
