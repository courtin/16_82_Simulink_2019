	function [CX,CL,CY,Cl,Cm,Cn, T, vis_data]	=	AeroModelSSTOL(x,u,Mach,alphar,betar,V,airplane)
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
%   u(5) = right flap deflection, rad, positive trailing edge down
%   u(6:end) = u_t, thrust control vector, 0-1

%   u_thrust(1) = left blowing throttle, 0-1
%   u_thrust(2) = right blowing throttle, 0-1
%   u_thrust(3) = left thruster throttle, 0-1
%   u_thrust(4) = right thruster throttle, 0-1

%	Inertial, Geometric, and Aerodynamic Properties
%   *** STOL_Input.m must first be run to save Airplane.mat***

d2r = deg2rad(1);

% 	A=load('Airplane.mat');
%     airplane = A.airplane;
    

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
    
    
    alpha_w     = alphar + i_w;
	a_w_deg	=	rad2deg(alpha_w);
    
    %Thrust Control vector
    u_t = u(6:end);
    d_BL = u_t(1);
    d_BR = u_t(2);
    d_TL = u_t(3);
    d_TR = u_t(4);

%   Blowing and Thrust Calculations
%   ====================================
    [dCJ_BL, ~, ~, CT_BL] = propulsor_perf_qprop(d_BL, airplane.propulsion.left_blower, cbar,Sw, alt, V);
    [dCJ_BR, ~,~, CT_BR] = propulsor_perf_qprop(d_BR, airplane.propulsion.right_blower, cbar,Sw, alt, V);
    [dCJ_TL,~, T_L, CT_CL] = propulsor_perf_qprop(d_TL, airplane.propulsion.left_cruiser, cbar,Sw, alt, V);
    [dCJ_TR,~, T_R, CT_CR] = propulsor_perf_qprop(d_TR, airplane.propulsion.right_cruiser, cbar,Sw, alt, V);

    %Outboard most motor contributes to half of the blowing
    %dCJ_BL=(3*dCJ_BL+0.5*dCJ_TL)/3.5;
    %dCJ_BR=(3*dCJ_BR+0.5*dCJ_TR)/3.5;
    
    %CT_BL=CT_BL+0.5*CT_CL;
    %CT_BR=CT_BR+0.5*CT_CR;
    
    %Half of outboard motor is pure thrust
%     CT_CL=0.5*CT_CL;
%     CT_CR=0.5*CT_CR;
   
    
    %T_L=0.5*T_L;
    %T_R=0.5*T_R;
    
    T=T_L+T_R;
    
%   Stability Derivates from JVL
%   ====================================
%   Currently these are just keyed off of the left flap deflection; no
%   provision yet for these as a result of asymmetric flap deflections

    [CLde, CLq, ...
    Cmde, Cmq, ...
    CYb, CYdA, CYdR, CYr, CYp,...
    Clb, CldA, CldR, Clr, Clp,...
    Cnb, CndA, CndR, Cnr, Cnp] = get_JVL_derivatives(u(4), dCJ_BL, airplane);


%	CL Calculations 
%	====================================

    %Wing Lift Coefficient
    %C_L_wing
    %Currently this interpolator is only for 40deg flaps; any
    %flap changes are ignored except the zero deg flap case.
    
    flap_L_deg = round(rad2deg(u(4)));
    flap_R_deg = round(rad2deg(u(5)));
    
    [cl_left,cx_left,cm_left]=get_coeffs_wing(a_w_deg,flap_L_deg,V,airplane,d_BL,d_TL);
    [cl_right,cx_right,cm_right]=get_coeffs_wing(a_w_deg,flap_R_deg,V,airplane,d_BR,d_TR);
    
%     if a_w_deg>25
%         [~,~,cm_right]=get_coeffs_wing(25,dCJ_BR,flap_R_deg,airplane);
%         cm_right=cm_right-0.0067*(a_w_deg-25);
%         [~,~,cm_left]=get_coeffs_wing(25,dCJ_BL,flap_L_deg,airplane);
%         cm_left=cm_left-0.0067*(a_w_deg-25);
%     end
%     if a_w_deg<-25
%         [~,~,cm_right]=get_coeffs_wing(25,dCJ_BR,flap_R_deg,airplane);
%         cm_right=cm_right-0.0067*(a_w_deg+25);
%         [~,~,cm_left]=get_coeffs_wing(25,dCJ_BL,flap_L_deg,airplane);
%         cm_left=cm_left-0.0067*(a_w_deg+25);
%     end
    
    
%     cl_left = getCLwing(a_w_deg,dCJ_BL,flap_L_deg,airplane);
%     cl_right = getCLwing(a_w_deg,dCJ_BR,flap_R_deg,airplane);
    
    CLw = .9*(cl_left + cl_right)/2;
    
    %Tail Lift Coefficient
    %C_L_ht
    eta_h = 1; %Update with better method of estimating tail dynamic pressure ratio
    eps = 2*CLw/(pi*e*AR); %May want to use M&S method here instead
    if eps>0
        eps=min([eps pi/2]);
    elseif eps<0
        eps=max([eps -pi/2]);
    end
    camber=0.05;
    a_h = rad2deg(alphar+i_t-eps+camber*1.5);
    
    eta = clat/(2*pi);
    sweep_h = airplane.geometry.Htail.sweep;

    cl_t=cl_airfoil(a_h);    
%     CLt = (cl_t*AR)/(2+sqrt(4*(AR/eta)^2*(1+tan(sweep_h)^2)));
    CLh=0.9*cl_t;
    
    CLt = CLh*Sh/Sw*eta_h;
    %Elevator and Pitch Rate effects
    %CLde = airplane.stability.CLde;
    %CLde_check = Sh/Sw*CLah*tau_h;
    %CLde = CLde_check; %Very different drom JVL; need to debug
    %CLq = airplane.stability.CLq;
    %CLq_check = -2*Vh*CLah*eta_h;
    %CLq = CLq_check; %Very different from JVL; need to debug
    
    
    CL = CLw + CLt + CLq*x(3)*(cbar/(2*V)) + CLde*u(1);
    if length(CL) ~= 1
        error('CL is not size 1')
    end
    
%     CL=min(CL,100);
%     CL=max(CL,-100);
    
%	CX Calculations 
%	====================================
%     cx_left = getCXwing(a_w_deg,dCJ_BL,CT_BL,flap_L_deg,airplane);
%     cx_right = getCXwing(a_w_deg,dCJ_BR,CT_BR,flap_R_deg,airplane);
    
    CXw = (cx_left+cx_right)/2;
    CDi = CLw^2/(pi*AR*e+2*(CT_BL+CT_BR));
    CDp = .02;  %Placeholder, this has little effect on the high-lift cases
    
    CX = CXw + CDi + CDp;% - CT_CL - CT_CR;
    CX
    CXw
    CDi
    CDp
%     CX=min(CX,100);
%     CX=max(CX,-100);
    
%	Cm Calculations 
%	====================================    
%     cm_left = getCMwing(a_w_deg,dCJ_BL,flap_L_deg,airplane);
%     cm_right = getCMwing(a_w_deg,dCJ_BR,flap_R_deg,airplane);
%     
    
    Cmw = (cm_left + cm_right)/2;
%     Cmh = -Vh*eta_h*CLah*a_h;
    Cmh = -Vh*eta_h*CLh;
    
    %Cmde = airplane.stability.Cmde;
    %Cmq = airplane.stability.Cmq;
    
    Cmaf = 0; %Negelect fuselage contributions for now
    Cmf = Cmaf*alphar;
    
    Cm = CLw*(x_cg-x_acw)/cbar + Cmw + Cmh + Cmf + Cmde*u(1) + Cmq*x(3)*(cbar/(2*V));
    
%     Cm=max(Cm,-100);
%     Cm=min(Cm,100);
    
%	Current Lateral-Directional Characteristics
%	===========================================
%	Cl Calculations 
%	==================================== 
%	Rolling Moment Coefficient

	Cl      =	Clb*betar + CldR*u(2) + Clr * x(7)*(b/(2*V)) + Clp * x(6)*(b/(2*V)) ...
                + CldA*u(3);%
									% Total Rolling-Moment Coefficient
    %Cl = 0;

%	CY Calculations 
%	==================================== 
%	Side-Force Coefficient
	CY	=(	CYb*betar + CYdR*u(2) + CYdA*u(3)+CYp*x(6)*(b/(2*V))+CYr*x(7)*(b/(2*V)));% + CYdASr*u(5));
								% Total Side-Force Coefficient
    %CY = 0;
%	Cn Calculations 
%	====================================                                     
%	Yawing Moment Coefficient
	Cn	=	Cnb*betar + CndR*u(2) + Cnr * x(7)*(b/(2*V)) + Cnp * x(6)*(b/(2*V)) ...
			+ CndA*u(3);% + CndASr*u(5));
									% Total Yawing-Moment Coefficient
    %Cn = 0;

% vis_data.dCJ_BL = dCJ_BL;
% vis_data.dCJ_BR = dCJ_BR;
% vis_data.CLw = CLw;
CL_tail = (CLt + CLde*u(1))*Sw/Sh*1/eta_h; %Display CL referenced to tail area;
% vis_data.CL = CL;
% vis_data.Cmw = Cmw;
spiral_stability=Clb*Cnr/(Clr*Cnb)
vis_data = [dCJ_BL dCJ_BR CLw CL_tail CL Cmw CX eps];
CX
    end