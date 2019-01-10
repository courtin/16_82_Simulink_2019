	function [CX,CL,CY,Cl,Cm,Cn]	=	AeroModelSSTOL(x,u,Mach,alphar,betar,V)
%	SSTOL Aero Model
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
%   u(4) = left flap deflection, rad, positive trailing edge down
%   u(5) = left flap deflection, rad, positive trailing edge down
%   u(6:end) = u_t, thrust control vector, 0-1

%   u_thrust(1) = left blowing throttle, 0-1
%   u_thrust(2) = right blowing throttle, 0-1
%   u_thrust(3) = left thruster throttle, 0-1
%   u_thrust(4) = right thruster throttle, 0-1

%	Inertial, Geometric, and Aerodynamic Properties
%   *** STOL_Input.m must first be run to save Airplane.mat***
	load Airplane.mat
    load 40df_fits.mat

    %Handy variable shorthands
    i_w = airplane.geometry.Wing.i;
    i_t = airplane.geometry.Htail.i;
    cbar = airplane.geometry.Wing.cbar;
    e = airplane.aero.Wing.e; %TODO: Make this a function of delta_f, etc.
    AR = airplane.geometry.Wing.AR;
    clat = airplane.aero.Htail.cla;
    Sh = airplane.geometry.Htail.S;
    Sv = airplane.geometry.Vtail.S;
    Sw = airplane.geometry.Wing.S;
    lh = airplane.geometry.Htail.l;
    lv = airplane.geometry.Vtail.l;
    b  = airplane.geometry.Wing.b;
    Vv = Sv/Sw*lv/b;
    Vh = Sh/Sw*lh/cbar;
    tau_h = airplane.aero.Htail.tau;
    x_cg = airplane.weights.xcg;
    x_acw = airplane.aero.Wing.x_ac;
    
    alt = -x(12);
    
    rad2deg = 180/pi;
    
    alpha_w     = alphar + i_w;
	a_w_deg	=	(alpha_w)*rad2deg;
    
    %Thrust Control vector
    u_t = u(6:end);
    d_BL = u_t(1);
    d_BR = u_t(2);
    d_TL = u_t(3);
    d_TR = u_t(4);

%   Blowing and Thrust Calculations
%   ====================================
    [dCJ_BL, ~, CT_BL] = propulsor_perf(d_BL, airplane.propulsion.left_blower, cbar, alt, V);
    [dCJ_BR, ~, CT_BR] = propulsor_perf(d_BR, airplane.propulsion.right_blower, cbar, alt, V);
    [~, T_L, CT_CL] = propulsor_perf(d_TL, airplane.propulsion.left_cruiser, cbar, alt, V);
    [~, T_R, CT_CR] = propulsor_perf(d_TR, airplane.propulsion.right_cruiser, cbar, alt, V);
    
%	CL Calculations 
%	====================================

    %Wing Lift Coefficient
    %C_L_wing
    %Currently this interpolator is only for 40deg flaps; any
    %flap changes are ignored.
    cl_left = cl_fit(a_w_deg, dCJ_BL);
    cl_right = cl_fit(a_w_deg, dCJ_BR);
    
    CLw = .9*(cl_left + cl_right)/2;
    
    %Tail Lift Coefficient
    %C_L_ht
    eta_h = 1; %Update with better method of estimating tail dynamic pressure ratio
    eps = 2*CLw/(pi*e*AR); %May want to use M&S method here instead
    a_h = (alphar+i_t-eps);
    
    eta = clat/(2*pi);
    sweep_h = airplane.geometry.Htail.sweep;
    CLah = (2*pi*AR)/(2+sqrt(4*(AR/eta)^2*(1+tan(sweep_h)^2)));
    CLt = CLah*a_h*Sh/Sw*eta_h;
    %Elevator and Pitch Rate effects
    CLde = airplane.stability.CLde;
    CLde_check = Sh/Sw*CLah*tau_h;
    CLde = CLde_check; %Very different drom JVL; need to debug
    CLq = airplane.stability.CLq;
    CLq_check = -2*Vh*CLah;
    CLq = CLq_check; %Very different drom JVL; need to debug
    CL = CLw + CLah*a_h + CLq*x(3) + CLde*u(1);

%	CX Calculations 
%	====================================
    cx_left = cx_fit(a_w_deg, dCJ_BL);
    cx_right = cx_fit(a_w_deg, dCJ_BR);
    
    CXw = (cx_left+cx_right)/2;
    CDi = CLw^2/(pi*AR*e+(CT_BL+CT_BR));
    CDp = .03;  %Placeholder, this has little effect on the high-lift cases
    
    CX = CXw + CDi + CDp - CT_CL - CT_CR;

%	Cm Calculations 
%	====================================    
    cm_left     = cm_fit(a_w_deg, dCJ_BL);
    cm_right    = cm_fit(a_w_deg, dCJ_BR); 
    
    Cmw = (cm_left + cm_right)/2;
    Cmh = -Vh*eta_h*CLah*a_h;
    
    Cmde = airplane.stability.Cmde;
    Cmq = airplane.stability.Cmq;
    
    Cmaf = 0; %Negelect fuselage contributions for now
    Cmf = Cmaf*alphar;
    
    Cm = CLw*(x_cg-x_acw)/cbar + Cmw + Cmh + Cmf + Cmde*u(1) + Cmq*x(3);
    
    	
%	Current Lateral-Directional Characteristics
%	===========================================
%	Cl Calculations 
%	==================================== 
%	Rolling Moment Coefficient

	%Cl      =	(ClBr*betar + CldRr*u(2)) + Clrr * x(7) + Clpr * x(6) ...
    %            + (CldAr*u(3));% + CldASr*u(5));
									% Total Rolling-Moment Coefficient
    Cl = 0;

%	CY Calculations 
%	==================================== 
%	Side-Force Coefficient
	%CY	=	(CYBr*betar + CYdRr*u(2)) + (CYdAr*u(3));% + CYdASr*u(5));
									% Total Side-Force Coefficient
    CY = 0;
%	Cn Calculations 
%	====================================                                     
%	Yawing Moment Coefficient
	%Cn	=	(CnBr*betar + CndRr*u(2)) + Cnrr * x(7) + Cnpr * x(6) ...
	%		+ (CndAr*u(3));% + CndASr*u(5));
									% Total Yawing-Moment Coefficient
    Cn = 0;


