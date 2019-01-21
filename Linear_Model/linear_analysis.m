%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AeroModelSSTOL Linearized Flight Analysis%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
STOL_Input;
global V m Ixx Iyy Izz Ixz S b cBar TrimHist x u

g = airplane.environment.g;
Ixx = airplane.weights.Ixx;
Iyy = airplane.weights.Iyy;
Izz = airplane.weights.Izz;
Ixz = airplane.weights.Ixz;
m = airplane.weights.MTOW/g;
S = airplane.geometry.Wing.S;
b = airplane.geometry.Wing.b;
cBar = airplane.geometry.Wing.cbar;
hft         =   1000;   % Altitude above Sea Level, ft
VKIAS       =   30;     % Indicated Airspeed, kt

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
disp(['Air Temperature = ',num2str(temp),   ' K,      Sound Speed  = ',num2str(soundSpeed),' m/s'])

qBarSL  =   0.5*1.225*VmsIAS^2;  % Dynamic Pressure at sea level, N/m^2
V       =   sqrt(2*qBarSL/airDens);	% True Airspeed, TAS, m/s
TASms   =   V;
disp('  ')
disp(['Dynamic Pressure = ',num2str(qBarSL),' N/m^2, True Airspeed = ',num2str(V),' m/s'])


SIM     =   0;  %1 to run simulation, 0 otherwise
alpha   =	15;      % Angle of attack, deg (relative to air mass)
beta    =	0;      % Sideslip angle, deg (relative to air mass)
dA      =	0;      % Aileron angle, deg
dE      =	-2.935;      % Elevator angle, deg
dR      =	0;      % Rudder angle, deg
dF_L    =   40;     % Flap angle, deg
dF_R    =   40;     % Flap angle, deg
dB_L    = 	.5;    % Left Blower throttle setting, % / 100
dB_R    = 	.5;    % Right Blower throttle setting, % / 100
dT_L    = 	0;    % Left Cruiser throttle setting, % / 100
dT_R    = 	0;    % Right Cruiser throttle setting, % / 100
hdot    =	0;      % Altitude rate, m/s
p       =	0;      % Body-axis roll rate, deg/s
phi     =	0;      % Body roll angle wrt earth, deg
psi     =	0;      % Body yaw angle wrt earth, deg
q       =	0;      % Body-axis pitch rate, deg/sec
r       =	0;      % Body-axis yaw rate, deg/s
theta   =	alpha;  % Body pitch angle wrt earth, deg [theta = alpha if hdot = 0]
xe      =	0;      % Initial longitudinal position, m
ye      = 	0;      % Initial lateral position, m
ze      = 	-hm;    % Initial vertical position, m [h: + up, z: + down]

Mach	= 	V / soundSpeed;	
gamma	=	57.2957795 * atan(hdot / sqrt(V^2 - hdot^2));
disp(['Mach number      = ',num2str(Mach),', Flight Path Angle = ',num2str(gamma),' deg'])										
disp('  ')
rad2deg = 180/pi;
phir	=	phi * 0.01745329;
thetar	=	theta * 0.01745329;
psir	=	psi * 0.01745329;

alphar	=	alpha * 0.01745329;
betar	=	beta * 0.01745329;

x	=	[V * cos(alphar) * cos(betar)
        V * sin(alphar) * cos(betar)
        q * 0.01745329
        thetar
        V * sin(betar)
        p * 0.01745329
        r * 0.01745329
        phir
        psir
        xe
        ye
        ze];

u	=	[dE * 0.01745329
        dA * 0.01745329
        dR * 0.01745329
        dF_L * 0.01745329
        dF_R * 0.01745329
        dB_L
        dB_R
        dT_L
        dT_R];


%Determine trim state
OptParam        =   [];
TrimHist        =   [];
InitParam		=	[u(1);u(6);x(4)];
gammar          =   0; %Trim flight path angle
        
options =   optimset('TolFun',1e-10);
CostFun = @(optparam) trim_cost(optparam);
[OptParam,J,ExitFlag,Output]  =	fminsearch(CostFun,InitParam,options);

u(1) = OptParam(1);
u(6) = OptParam(2);
u(7) = OptParam(2);

x(1) = V*cos(OptParam(3));
x(2) = V*sin(OptParam(3));
x(4) = OptParam(3);

subplot(3,2,[2,4,6])
plot(TrimHist(4,:))
title('Trim Cost')
subplot(3,2,1)
plot(TrimHist(1,:))
ylabel('\delta_E')
subplot(3,2,3)
plot(TrimHist(2,:))
ylabel('\delta_{T,B}')
subplot(3,2,5)
plot(TrimHist(3,:))
ylabel('\theta')

%Initial State
disp('Trimmed Initial State')
disp('==================')
disp('Control Vector')
disp('--------------')
disp(['Elevator   = ',num2str(u(1)*rad2deg),' deg, Aileron = ',num2str(u(3)*rad2deg),' deg, Rudder = ',num2str(u(2)*rad2deg),' deg'])
disp(['Left Flap  = ',num2str(u(4)*rad2deg),' deg, Right Flap = ',num2str(u(5)*rad2deg)])
disp(['Throttle (Left Blowing)   = ',num2str(u(6)),' x 100%, Throttle (Right Blowing) = ',num2str(u(7)),' x 100%'])
disp(['Throttle (Left Cruise)    = ',num2str(u(8)),' x 100%, Throttle (Right Cruise)  = ',num2str(u(9)),' x 100%'])

disp('  ')
disp('State Vector')
disp('------------')
disp(['u   = ',num2str(x(1)),' m/s, v = ',num2str(x(5)),' m/s, w = ',num2str(x(2)),' m/s'])
disp(['x   = ',num2str(x(10)),' m, y = ',num2str(x(11)),' m, z = ',num2str(x(12)),' m'])
disp(['p   = ',num2str(x(6)),' rad/s, q = ',num2str(x(3)),' rad/s, r = ',num2str(x(7)),' rad/s'])
disp(['Phi = ',num2str(x(8)*rad2deg),' deg, Theta = ',num2str(x(4)*rad2deg),' deg, Psi = ',num2str(x(9)*rad2deg),' deg'])
disp('  ')
format short
[CX,CL,CY,Cl,Cm,Cn]	=	AeroModelSSTOL(x,u,Mach,x(4),betar,V);
disp('Stability-Axis Force Coefficients')
disp('---------------------------------')
disp(['CX = ',num2str(CX),' CL = ',num2str(CL),' CY = ',num2str(CY)])
disp(['Cm = ',num2str(Cm),' Cl = ',num2str(Cl),' Cn = ',num2str(Cn)])


thresh	=	[.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1];
ti = 0;
xi = [x;u];
xdott = LinMod(ti,xi);
[dFdX, fac] = numjac('LinMod', ti, xi, xdott, thresh, [], 0);

%Plot roots
F = dFdX(1:12, 1:12);
G = dFdX(1:12, 13:end);

e = eig(F);
figure()
ax = axes();
scatter(real(e), imag(e))
title('Root Locus')
grid
ax.YAxisLocation = 'origin';
ax.XAxisLocation = 'origin';

%save('eig_high.mat', 'e')

e_u = unique(e);
for i = 1:length(e_u)
    [damp, freq] = get_damping_freq(e_u(i));
    disp(['Eigenvalue: ', num2str(e_u(i))]);
    disp(['Damping ratio: ', num2str(damp), ', Natural Frequency: ', num2str(freq) ' rad/s, Period: ', num2str(2*pi/freq), ' s']);
end

% disp('  ')
% disp('Control Vector Time History Table')
% disp('=================================')
% disp('Time, sec: t0, t1, t2, ...')
% tuHis	=	[0 9 10 11 12 100]
% 
% disp('Columns:  Elements of the Control Vector')
% disp('Rows:     Value at time, t0, t1, ...')
% deluHis	=  [0 0 0 0 0 0 0
%             0 0 0 0 0 0 0
%             -.05 0 0 0 0 0 0
%             .05 0 0 0 0 0 0
%             0 0 0 0 0 0 0
%             0 0 0 0 0 0 0]
% 
% %Analysis of F and G matrix
% 
% C = [1 0 0 0 0 0 0 0 0 0 0 0;
%     0 1 0 0 0 0 0 0 0 0 0 0
%     0 0 1 0 0 0 0 0 0 0 0 0
%     0 0 0 1 0 0 0 0 0 0 0 0];
% 
% sys = ss(F,G,C,0);
% %t = repmat([0:.1:60], length(C), 1);
% t = [0:.1:10];
% ut = zeros(length(u), length(t));
% for i = 1:length(t)
%     ut(:,i) = u;
% end
% y =lsim(sys, ut, t,x);
% figure()
% plot(t,y)
% legend('ub', 'wb', 'q', '\theta')
% 
% %Simulation initial conditions - start from trim state, plus any deltas
% del_x = [0      %u_b
%          0      %w_b
%          0 / rad2deg;     %q
%          0 / rad2deg;      %theta
%          0      %v_b
%          0      %p
%          0      %r
%          0      %phi
%          0      %psi
%          0      %x_e
%          0      %y_e
%          0];    %z_e
% del_u = [0 / rad2deg      %d_E
%          0      %d_R
%          0      %d_A
%          0      %d_FL
%          0      %d_FR
%          0      %d_TBL
%          0      %d_TBR
%          0      %d_TCL
%          0];    %d_TCR
% 
% u = u+del_u;
% x_i = x+del_x;
% 
% %Simulation
% if SIM
%     tf      =	100;    % Final time for simulation, sec
%     ti      = 	0;      % Initial time for simulation, sec
%     mps2kts = 1.94384;
%     mps2fpm = 196.85;
%     [t_his, x_his] = ode45('EoM3', [ti, tf], x_i);
%     %Speed History
%     figure()
% 
%     subplot(3,1,1)
%     plot(t_his, x_his(:,1).*mps2kts)
%     ylabel('u_b (kts)')
%     title('Speed history')
%     subplot(3,1,2)
%     plot(t_his, x_his(:,5))
%     ylabel('v_b (m/s)')
%     subplot(3,1,3)
%     plot(t_his, x_his(:,2).*mps2fpm)
%     ylabel('w_b (ft/min)')
% 
%     %Altitude history
%     figure()
% 
%     subplot(3,1,1)
%     plot(t_his, x_his(:,10))
%     ylabel('x_e (m)')
%     title('Location history')
%     subplot(3,1,2)
%     plot(t_his, x_his(:,11))
%     ylabel('y_e (m)')
%     subplot(3,1,3)
%     plot(t_his, -x_his(:,12))
%     ylabel('alt (m)')
%     
%     %Orientation history
%     figure()
% 
%     subplot(3,1,1)
%     plot(t_his, x_his(:,4).*rad2deg)
%     ylabel('\theta (deg)')
%     title('Orientation history')
%     subplot(3,1,2)
%     plot(t_his, x_his(:,8).*rad2deg)
%     ylabel('\phi (deg)')
%     subplot(3,1,3)
%     plot(t_his, x_his(:,9).*rad2deg)
%     ylabel('\psi (m)')
%     
%     %Rate history
%     figure()
% 
%     subplot(3,1,1)
%     plot(t_his, x_his(:,3).*rad2deg)
%     ylabel('q (deg/sec)')
%     title('Angular rate history')
%     subplot(3,1,2)
%     plot(t_his, x_his(:,6).*rad2deg)
%     ylabel('p (deg/sec)')
%     subplot(3,1,3)
%     plot(t_his, x_his(:,7).*rad2deg)
%     ylabel('r (m)')
% end


