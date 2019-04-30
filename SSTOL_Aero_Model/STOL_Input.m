                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file defines the aircraft who's performance is to be analyzed.  The
% aircraft is represented by an "airplane" data structure which
% contains all the parameters that describe the airplane.  This
% data structure contains several substructures with the parameters
% describing those subsystems.  The fields of the airplane data
% structure are described below:

% create the airplane data structure and initialize the common
% parameters.

%Useful unit conversion factors
ftsq2msq    = 0.0929;
ft2m        = 0.3048;
lbf2N       = 4.44822;
fpm2mps     = 0.00508;
kts2mps     = 0.51444;
nmi2m       = 1852.0;
in2m        = 0.0254;
slugftsq2kgmsq = 1.35581795;
deg2rad     = pi/180;

g = 9.81;

% %%%%%%%%%%%%%%%%%%%%%%%
% %VEHICLE GEOMETRY     %
% %%%%%%%%%%%%%%%%%%%%%%%
% geometry.Wing.S              = 153.0   *ftsq2msq;
% geometry.Wing.b                 = 37. * ft2m;
% geometry.Wing.cbar              = 5.7 * ft2m;     %Approximately hershey bar
% geometry.Wing.AR                = 6.3;
% geometry.Wing.i                 = 3 * deg2rad;
% 
% geometry.Wing.f_flap_span       = .6;
% geometry.Wing.f_flap_chord      = .3;
% 
% geometry.Wing.f_spoil_span      = .5;
% geometry.Wing.f_spoil_chor      = .2;
% 
% geometry.Htail.S                = 43 * ftsq2msq;   %square feet
% geometry.Htail.l                = 16 * ft2m;
% geometry.Htail.i                = 0 * deg2rad;
% geometry.Htail.sweep            = 0;
% 
% geometry.Vtail.S                = 43 * ftsq2msq;
% geometry.Vtail.AR               = 16 * ft2m;
% geometry.Vtail.l                = 16 * ft2m;
% 
% geometry.Fuse.Swet              = (200.6 + 166.4)  * ftsq2msq; %counting tail boom 
% geometry.Fuse.l                 = 30   * ft2m;
% geometry.Fuse.W_max             = 5.14 * ft2m;
% geometry.Fuse.fr                = 1.875; %Fineness ratio
% 
% 
% %%%%%%%%%%%%%%%%%
% %VEHICLE WEIGHTS%
% %%%%%%%%%%%%%%%%%
% weights.MTOW        = (2700)     * lbf2N;
% weights.payload     = 4*203     * lbf2N;
% weights.MLW         = weights.MTOW;
% weights.Ixx         = 1048 * slugftsq2kgmsq;
% weights.Iyy         = 3000 * slugftsq2kgmsq;
% weights.Izz         = 3530 * slugftsq2kgmsq;
% weights.Ixz         = 0.0 * slugftsq2kgmsq;
% weights.xcg         = .0295*geometry.Wing.cbar;
% 
% %%%%%%%%%%%
% %STABILITY%
% %%%%%%%%%%%
% %Currently each JVL stability derivative is given only as a function of
% %flap deflection.  At high flap deflections there is a positive CJ in the
% %JVL model; the variation with this is not yet captured.  This is valid
% %only as an early estimate; currently only derivatives for flap angles [0,
% %25, 45] deg are provided. TODO: Automate this input
% 
% 
% % stability.flap_settings = [0, 25, 45];
% % stability.CLq = [14.45, 16.5, 15.18];
% % stability.Cmq = [-42.66, -44.54, -42.5];     %%%OLD
% % stability.CLde = [.0144 .0152 .0139] ./ deg2rad;
% % stability.Cmde = [-0.0624 -.0667 -.0639] ./ deg2rad;
% stability.flap_settings = [0, 25, 45];
% stability.CLq = [9.89, 9.55,  8.83];
% stability.Cmq = [-37.07,-36, -35];
% stability.CLde = [.014 0.014 .0129] ./ deg2rad;
% stability.Cmde = [-.056 -0.055 -.052] ./ deg2rad;
% stability.CYB = [-.728, -1.22,-1.233];
% stability.ClB = [-.030, -.136, -.201];
% stability.CnB = [.241, .315, .362];
% stability.CYdA = [.00018, .000135, .00045] ./ deg2rad;
% stability.CYdR = [-.00537, -.00527, -.00486] ./ deg2rad;
% stability.CldA = [-.004136, -.00415, -.00389] ./ deg2rad;
% stability.CldR = [-.000264, -.000241, -.000198] ./ deg2rad;
% stability.CndA = [-.000043, -.000303, 0.000469] ./ deg2rad;
% stability.CndR = [.00299, .00291, .002677] ./ deg2rad;
% stability.CYr = [.555, .644, .541];
% stability.CYp = [.0461, .452, .747];
% stability.Clr = [.1141, .4115, .6210];
% stability.Clp = [-.5877, -.6260, -.5873];
% stability.Cnr = [-.2911, -.3746, -.4040];
% stability.Cnp = [-.0311, -.2357, -.2974];
% 
% 
% %%%%%%%%%%%%%%%%%%%
% %PROPULSION SYSTEM%
% %%%%%%%%%%%%%%%%%%%
% %Propulsion system consists of two types of power sources; cruise motors
% %and blowing motors.  Each has an associated maximum shaft power, and
% %actual shaft power is varied from zero to 100% via a throttle command. 
% propulsion.left_blower.N            = 4;
% propulsion.left_blower.P_shaft_max  = (240*1000);
% propulsion.left_blower.eta_v        = .8;
% propulsion.left_blower.eta_add      = .7;
% propulsion.left_blower.R            = .381;
% propulsion.left_blower.r_hub        = .06;
% propulsion.left_blower.b            = geometry.Wing.b/2;
% propulsion.right_blower.N           = 4;
% propulsion.right_blower.P_shaft_max = (240*1000);
% propulsion.right_blower.eta_v        = .8;
% propulsion.right_blower.eta_add      = .7;
% propulsion.right_blower.R            = .381;
% propulsion.right_blower.r_hub        = .06;
% propulsion.right_blower.b            = geometry.Wing.b/2;
% propulsion.left_cruiser.N            = 1;
% propulsion.left_cruiser.P_shaft_max  = (240*1000);
% propulsion.left_cruiser.eta_v        = .85;
% propulsion.left_cruiser.eta_add      = .7;
% propulsion.left_cruiser.R            = .381;
% propulsion.left_cruiser.r_hub        = .06;
% propulsion.left_cruiser.b            = geometry.Wing.b/2;
% propulsion.right_cruiser.N           = 1;
% propulsion.right_cruiser.P_shaft_max = (240*1000);
% propulsion.right_cruiser.eta_v       = .85;
% propulsion.right_cruiser.eta_add     = .7;
% propulsion.right_cruiser.R           = .381;
% propulsion.right_cruiser.r_hub       = .06;
% propulsion.right_cruiser.b           = geometry.Wing.b/2;
% 
% propulsion.wheels.mu_brk    = .5;
% propulsion.wheels.mu_roll   = .02;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%
% %AERODYNAMIC DEFINITION%
% %%%%%%%%%%%%%%%%%%%%%%%%
% aero.Wing.e              = .75;
% aero.Wing.x_ac           = .25*geometry.Wing.cbar;
% aero.Wing.cl_max_clean   = 1.6;
% aero.Htail.tau            = .65;
% aero.Htail.cla            = 2*pi;
% 
% load 40df_fits.mat
% 
% %aero.Wing.cl_fit        = cl_fit;
% %aero.Wing.cx_fit        = cx_fit;
% %aero.Wing.cm_fit        = cm_fit;
% 
% aero.Wing.fits.alpha_range = -10:40;
% aero.Wing.fits.dCJ_range = 0:8;
% aero.Wing.fits.flaps_range = 40;
% 
% %%% To do: Fix the way these data tables are generated so they work for
% %%% more than one flap angle
% aero.Wing.fits.cls = create_cl_datatable(cl_fit,aero.Wing.fits);
% aero.Wing.fits.cxs = create_cx_datatable(cx_fit,aero.Wing.fits);
% aero.Wing.fits.cms = create_cm_datatable(cm_fit,aero.Wing.fits);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%
% %ENVIRONMENT DEFINITION%
% %%%%%%%%%%%%%%%%%%%%%%%%
% environment.g = 9.8067;
% %%%%%%%%%%%%%%%%%%%%%%%
% %SIMULATION DEFINITION%
%%%%%%%%%%%%%%%%%%%%%%%


% stability.flap_settings = [0, 25, 45];
% stability.CLq = [14.45, 16.5, 15.18];
% stability.Cmq = [-42.66, -44.54, -42.5];     %%%OLD
% stability.CLde = [.0144 .0152 .0139] ./ deg2rad;
% stability.Cmde = [-0.0624 -.0667 -.0639] ./ deg2rad;
flap_settings = [-2 -2 -2 6 6 6 15 15 15]';
dCJ_settings = [0 3 6 0 3 6 0 3 6]';

load('f_JVLderivatives.mat')
% stability.CLq = @(x,y)A.f_CLq(x,y);
% stability.Cmq = @(x,y)A.f_Cmq(x,y);
% stability.CLde = @(x,y)A.f_CLde(x,y)./ deg2rad;
% stability.Cmde = @(x,y)A.f_Cmde(x,y) ./ deg2rad;
% stability.CYB = @(x,y)A.f_CYB(x,y);
% stability.ClB = @(x,y)A.f_ClB(x,y);
% stability.CnB = @(x,y)A.f_CnB(x,y);
% stability.CYdA = @(x,y)A.f_CYdA(x,y)./ deg2rad;
% stability.CYdR = @(x,y)A.f_CYdR(x,y)./ deg2rad;
% stability.CldA = @(x,y)A.f_CldA(x,y)./ deg2rad;
% stability.CldR = @(x,y)A.f_CldR(x,y) ./ deg2rad;
% stability.CndA = @(x,y)A.f_CndA(x,y) ./ deg2rad;
% stability.CndR = @(x,y)A.f_CndR(x,y) ./ deg2rad;
% stability.CYr = @(x,y)A.f_CYr(x,y);
% stability.CYp = @(x,y)A.f_CYp(x,y);
% stability.Clr = @(x,y)A.f_Clr(x,y);
% stability.Clp = @(x,y)A.f_Clp(x,y);
% stability.Cnr = @(x,y)A.f_Cnr(x,y);
% stability.Cnp = @(x,y)A.f_Cnp(x,y);
stability=A;
db=load('POC_motor_db.mat');
volt=1:1:22;
speed=0:5:65;
Thrust=reshape(db.T,[], 1);
dv=reshape(db.DV,[], 1);
dv(dv<-10) = -10; %remove outliers from data
[Volt,Speed]=meshgrid(volt,speed);
Volt=reshape(Volt,[],1);
Speed=reshape(Speed,[],1);
max_voltage=22;

f_thrust=coeffvalues(fit([Volt Speed], Thrust,'poly22'));
f_dv=coeffvalues(fit([Volt Speed], dv,'poly22'));


%Combine Data Structures

load('POC_airplane.mat')
airplane.stability=stability;
airplane.propulsion.right_cruiser.f_thrust=f_thrust;
airplane.propulsion.left_cruiser.f_thrust=f_thrust;
airplane.propulsion.right_blower.f_thrust=f_thrust;
airplane.propulsion.left_blower.f_thrust=f_thrust;

airplane.propulsion.right_cruiser.f_dv=f_dv;
airplane.propulsion.left_cruiser.f_dv=f_dv;
airplane.propulsion.right_blower.f_dv=f_dv;
airplane.propulsion.left_blower.f_dv=f_dv;

airplane.propulsion.right_cruiser.max_voltage=max_voltage;
airplane.propulsion.left_cruiser.max_voltage=max_voltage;
airplane.propulsion.right_blower.max_voltage=max_voltage;
airplane.propulsion.left_blower.max_voltage=max_voltage;
airplane.weights.xcg=0.154;

airplane.weights.Ixx=8;
airplane.weights.Iyy=6.5;
airplane.weights.Izz=14.5;



airplane.aero.Wing.x_ac=0.154;
save("Airplane.mat","airplane")
clearvars -except airplane