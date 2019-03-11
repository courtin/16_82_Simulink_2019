	function xdot = EoM3(t,x)
%   Modified from: 
%	FLIGHT Equations of Motion

%	June 12, 2015  
%	===============================================================
%	Copyright 2006-2015 by ROBERT F. STENGEL.  All rights reserved.

	global m u Ixx Iyy Izz Ixz S b cBar 
    
%   Select Aerodynamic Model

 
    AeroModel   =   @AeroModelSSTOL;
    
    %[value,isterminal,di0.04214011rection] = event(t,x);
    
    uTotal = u;
    
%	Earth-to-Body-Axis Transformation Matrix
	HEB		=	DCM(x(8),x(4),x(9));
%	Atmospheric State
    x(12)    =   min(x(12),0);    % Limit x(6) to <= 0 m
	[temp,airPres,airDens,soundSpeed]	=	int_std_atm(-x(12));
%	Body-Axis Wind Field
	%windb	=	WindField(-x(6),x(10),x(11),x(12));
%	Body-Axis Gravity Components
	gb		=	HEB * [0;0;9.80665];

%	Air-Relative Velocity Vector
    x(1)    =   max(x(1),0);        %   Limit axial velocity to >= 0 m/s
	Va		=	[x(1);x(5);x(2)];%+ windb;
	V		=	sqrt(Va' * Va);
	alphar	=	atan(Va(3) / abs(Va(1)));
%    alphar  =   min(alphar, (pi/2 - 1e-6));  %   Limit angle of attack to <= 90 deg
    
	alpha 	=	57.2957795 * alphar;
    
	betar	= 	asin(Va(2) / V);
	beta	= 	57.2957795 * betar;
	Mach	= 	V / soundSpeed;
	qbar	=	0.5 * airDens * V^2;

%	Incremental Flight Control Effects

%  	if CONHIS >=1 && RUNNING == 1
% 		[uInc]	=	interp1(tuHis,deluHis,t);
% 		uInc	=	(uInc)';
% 		uTotal	=	u + uInc;
% 	else
% 		uTotal	=	u;
%     end
    
%	Force and Moment Coefficients; Thrust	
	[CXs,CL,CY,Cl,Cm,Cn]	=	AeroModel(x,uTotal,Mach,alphar,betar,V);

	qbarS	=	qbar * S;

	CX	=	-CXs * cos(alphar) + CL * sin(alphar);	% Body-axis X coefficient
	CZ	= 	-CXs * sin(alphar) - CL * cos(alphar);	% Body-axis Z coefficient

%	State Accelerations
   
	Xb =	CX * qbarS / m;
	Yb =	CY * qbarS / m;
	Zb =	CZ * qbarS / m;
	Lb =	Cl * qbarS * b;
	Mb =	Cm * qbarS * cBar;
	Nb =	Cn * qbarS * b;
	nz	=	-Zb / 9.80665;							% Normal load factor

%	Dynamic Equations
	xd1 = Xb + gb(1) + x(7) * x(5) - x(3) * x(2);
	xd5 = Yb + gb(2) - x(7) * x(1) + x(6) * x(2);
	xd2 = Zb + gb(3) + x(3) * x(1) - x(6) * x(5);
	
	y	=	HEB' * [x(1);x(5);x(2)];
	xd10	=	y(1);
	xd11	=	y(2);
	xd12	=	y(3);
	
	xd6	= 	(Izz * Lb + Ixz * Nb - (Ixz * (Iyy - Ixx - Izz) * x(6) + ...
				(Ixz^2 + Izz * (Izz - Iyy)) * x(7)) * x(3)) / (Ixx * Izz - Ixz^2);
	xd3 = 	(Mb - (Ixx - Izz) * x(6) * x(7) - Ixz * (x(6)^2 - x(7)^2)) / Iyy;
	xd7 =	(Ixz * Lb + Ixx * Nb + (Ixz * (Iyy - Ixx - Izz) * x(7) + ...
				(Ixz^2 + Ixx * (Ixx - Iyy)) * x(6)) * x(3)) / (Ixx * Izz - Ixz^2);

	cosPitch	=	cos(x(4));
	if abs(cosPitch)	<=	0.00001
		cosPitch	=	0.00001 * sign(cosPitch);
	end
	tanPitch	=	sin(x(4)) / cosPitch;
		
	xd8	=	x(6) + (sin(x(8)) * x(3) + cos(x(8)) * x(7)) * tanPitch;
	xd4	=	cos(x(8)) * x(3) - sin(x(8)) * x(7);
	xd9	=	(sin(x(8)) * x(3) + cos(x(8)) * x(7)) / cosPitch;
	
	xdot	=	[xd1;xd2;xd3;xd4;xd5;xd6;xd7;xd8;xd9;xd10;xd11;xd12];
