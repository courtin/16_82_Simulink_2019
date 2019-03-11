	function [CX,CL,CY,Cl,Cm,Cn]	=	AeroModelBizJet(x,u,Mach,alphar,betar,V)
%	Bizjet Aero Model
%   x(1) = u_b          Body axis u velocity, m/s
%   x(2) = w_b          Body axis w velocity, m/s
%   x(3) = q            Pitch rate, rad/s
%   x(4) = theta        Pitch euler angle, rad
%   x(5) = v_b          Body axis v velocity, m/s
%   x(6) = p            Roll rate, rad/s
%   x(7) = r            Yaw rate, rad/s
%   x(8) = phi          Roll euler angle, rad
%   x(9) = psi          Yaw euler angle, rad
%   x(10) = xe          North position, Earth frame, m
%   x(11) = ye          East position, Earth frame, m
%   x(12) = ze =-h      Negative of altitude, Earth frame, m

%   u(1) = elevator deflection, rad, positive trailing edge down
%   u(2) = rudder deflection, rad, positive trailing edge down
%   u(3) = aileron deflection, rad, positive left trailing edge down
%   u(4) = flap deflection, rad, positive trailing edge down
%   u(5:end) = u_t, thrust control vector, 0-1

%   u_thrust(1) = throttle, 0-1

%	Inertial, Geometric, and Aerodynamic Properties
%   *** GeoMassAero.m must first be run to save InerGeo.mat, DataTable.mat, 
%   and RotCont.mat ***
	load InerGeo.mat
    load DataTable.mat
    load RotCont.mat
    
    global SMI

	alphadeg	=	57.2957795 * alphar;
    
    %Thrust Control vector - will need to be more complex for blown wing
    u_t = u(5:end);
    
%	Thrust Properties
	StaticThrust	=	2*6.49*10^3;	% Static Thrust @ Sea Level, N	
%	Current Thrust
	[~,~,airDens] = int_std_atm(-x(6));
	Thrust			=	u_t(1) * StaticThrust * (airDens / 1.225)^0.7 ...
						* (1 - exp((-x(12) - 17000) / 2000));
									% Thrust at Altitude, N
	CT = Thrust/(.5*airDens*V^2*S);
%	Current Longitudinal Characteristics
%	====================================

%	Lift Coefficient
	CLStatic    =	interp1(AlphaTable,CLTable,alphadeg);
									% Static Lift Coefficient
	CLqr        =	CLqHat * cBar/(2*V);
									% Pitch-Rate Effect, per rad/s
	CLdEr   	=	interp1(AlphaTable,CLdETable,alphadeg);
                                    % Elevator Effect, per rad
	CLdSr       =	CLdEr;			% Stabilator Effect, per rad
    CL          =	CLStatic + CLqr*x(3) + CLdEr*u(1); %+ CLdSr*u(7);
									% Total Lift Coefficient	
%	Drag Coefficient
	CDStatic	=	interp1(AlphaTable,CDTable,alphadeg);
									% Static Drag Coefficient
	CD          =	CDStatic;		% Total Drag Coefficient
    
    CX = CD - CT;
	
%	Pitching Moment Coefficient
	CmStatic	=	interp1(AlphaTable,CmTable,alphadeg);
									% Static Pitching Moment Coefficient
	CmdEr		=	interp1(AlphaTable,CmdETable,alphadeg);
									% Elevator Effect, per rad
	Cmqr        =	-CLqHat*(lHT/cBar) * cBar/(2*V);
									% Pitch-Rate + Alpha-Rate Effect, per rad/s
    CmdSr       =	CmdEr;          % Stabilator Effect, per rad
	Cm          =	CmStatic - CL*SMI + Cmqr*x(8) + CmdEr*u(1);% + CmdSr*u(7);
									% Total Pitching Moment Coefficient
	
%	Current Lateral-Directional Characteristics
%	===========================================

%	Rolling Moment Coefficient
	ClBr	=	interp1(AlphaTable,ClBetaTable,alphadeg);
									% Dihedral Effect, per rad
	ClpHat	=	interp1(AlphaTable,ClpHatTable,alphadeg);
    Clpr    =   ClpHat * (b / (2 * V));				
	ClrHat	=	interp1(AlphaTable,ClrHatTable,alphadeg);
									% Roll-Rate Effect, per rad/s	
	Clrr	=	ClrHat * (b / (2 * V));				
									% Yaw-Rate Effect, per rad/s
	CldAr	=	interp1(AlphaTable,CldATable,alphadeg);
									% Aileron Effect, per rad	
	CldRr	=	interp1(AlphaTable,CldRTable,alphadeg);
									% Rudder Effect, per rad
	CldASr	=	0;                  % Asymmetric Spoiler Effect, per rad
	Cl      =	(ClBr*betar + CldRr*u(2)) + Clrr * x(7) + Clpr * x(6) ...
                + (CldAr*u(3));% + CldASr*u(5));
									% Total Rolling-Moment Coefficient
                                    
%	Side-Force Coefficient
	CYBr	=	interp1(AlphaTable,CYBetaTable,alphadeg);
									% Side-Force Slope, per rad
	CYdAr	=	CYdAo;              % Aileron Effect, per rad	
	CYdRr	=	0.1574;				% Rudder Effect, per rad	
	CYdASr	=	0;                  % Asymmetric Spoiler Effect, per rad
	CY	=	(CYBr*betar + CYdRr*u(2)) + (CYdAr*u(3));% + CYdASr*u(5));
									% Total Side-Force Coefficient
                                    
%	Yawing Moment Coefficient
	CnBr	=	interp1(AlphaTable,CnBetaTable,alphadeg);
									% Directional Stability, per rad
	Cnpr	=	CL * (1 + 3 * taperw)/(12 * (1 + taperw)) * (b / (2 * V));				
									% Roll-Rate Effect, per rad/s	
	Cnrr	=	(-2 * (lVT / b) * CnBr - 0.1 * CL^2) * (b / (2 * V));
    CnpHat	=	interp1(AlphaTable,CnpHatTable,alphadeg);
    Cnpr    =   CnpHat * (b / (2 * V));				
	CnrHat	=	interp1(AlphaTable,CnrHatTable,alphadeg);
									% Roll-Rate Effect, per rad/s	
	Cnrr	=	CnrHat * (b / (2 * V));				
									% Yaw-Rate Effect, per rad/s
									% Yaw-Rate Effect, per rad/s
	CndAr	=	interp1(AlphaTable,CndATable,alphadeg);
									% Aileron Effect, per rad	
	CndRr	=	interp1(AlphaTable,CndRTable,alphadeg);
									% Rudder Effect, per rad	
	CndASr	=	0;			% Asymmetric Spoiler Effect, per rad
	Cn	=	(CnBr*betar + CndRr*u(2)) + Cnrr * x(7) + Cnpr * x(6) ...
			+ (CndAr*u(3));% + CndASr*u(5));
									% Total Yawing-Moment Coefficient


