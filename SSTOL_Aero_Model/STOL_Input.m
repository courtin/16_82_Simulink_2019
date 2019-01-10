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

%%%%%%%%%%%%%%%%%%%%%%%
%VEHICLE GEOMETRY     %
%%%%%%%%%%%%%%%%%%%%%%%
geometry.Wing.S              = 153.0   *ftsq2msq;
geometry.Wing.b                 = 37. * ft2m;
geometry.Wing.cbar              = 5.7 * ft2m;     %Approximately hershey bar
geometry.Wing.AR                = 6.3;
geometry.Wing.i                 = 3 * deg2rad;

geometry.Wing.f_flap_span       = .6;
geometry.Wing.f_flap_chord      = .3;

geometry.Wing.f_spoil_span      = .5;
geometry.Wing.f_spoil_chor      = .2;

geometry.Htail.S                = 43 * ftsq2msq;   %square feet
geometry.Htail.l                = 16 * ft2m;
geometry.Htail.i                = 0 * deg2rad;
geometry.Htail.sweep            = 0;

geometry.Vtail.S                = 43 * ftsq2msq;
geometry.Vtail.AR               = 16 * ft2m;
geometry.Vtail.l                = 16 * ft2m;

geometry.Fuse.Swet              = (200.6 + 166.4)  * ftsq2msq; %counting tail boom 
geometry.Fuse.l                 = 30   * ft2m;
geometry.Fuse.W_max             = 5.14 * ft2m;
geometry.Fuse.fr                = 1.875; %Fineness ratio


%%%%%%%%%%%%%%%%%
%VEHICLE WEIGHTS%
%%%%%%%%%%%%%%%%%
weights.MTOW        = (2700)     * lbf2N;
weights.payload     = 4*203     * lbf2N;
weights.MLW         = weights.MTOW;
weights.Ixx         = 1048 * slugftsq2kgmsq;
weights.Iyy         = 3000 * slugftsq2kgmsq;
weights.Izz         = 3530 * slugftsq2kgmsq;
weights.Ixz         = 0.0 * slugftsq2kgmsq;
weights.xcg         = .0295*geometry.Wing.cbar;

%%%%%%%%%%%
%STABILITY%
%%%%%%%%%%%

% stability.CL = 4.306;
% stability.CD = -.55;
% stability.CLa = 6.64;
% stability.CDa = .33;
% stability.Cma = -.736;
% stability.CLadot = 0.0;
% stability.Cmadot = -4.36;
stability.CLq = 10.33;
stability.Cmq = -24.7;
% stability.CLM = 0.0;
% stability.CDM = 0.0;
% stability.CmM = 0.0;
stability.CLde = .010 / deg2rad;
stability.Cmde = -0.038 / deg2rad;
% stability.CyB = -1.324;
% stability.ClB = -.204;
% stability.CnB = .236;
% stability.Clp = -.445;
% stability.Cnp = -.154;
% stability.Clr = .276;
% stability.Cnr = -.324;
% stability.Clda = -.134;
% stability.Cnda = -.0035;
% stability.Cydr = -.0025;
% stability.Cldr = -.00009;
% stability.Cndr = -0.001472;

%%%%%%%%%%%%%%%%%%%
%PROPULSION SYSTEM%
%%%%%%%%%%%%%%%%%%%
%Propulsion system consists of two types of power sources; cruise motors
%and blowing motors.  Each has an associated maximum shaft power, and
%actual shaft power is varied from zero to 100% via a throttle command. 
propulsion.left_blower.N            = 4;
propulsion.left_blower.P_shaft_max  = (240*1000);
propulsion.left_blower.eta_v        = .8;
propulsion.left_blower.eta_add      = .7;
propulsion.left_blower.R            = .381;
propulsion.left_blower.r_hub        = .06;
propulsion.left_blower.b            = geometry.Wing.b/2;
propulsion.right_blower.N           = 4;
propulsion.right_blower.P_shaft_max = (240*1000);
propulsion.right_blower.eta_v        = .8;
propulsion.right_blower.eta_add      = .7;
propulsion.right_blower.R            = .381;
propulsion.right_blower.r_hub        = .06;
propulsion.right_blower.b            = geometry.Wing.b/2;
propulsion.left_cruiser.N            = 1;
propulsion.left_cruiser.P_shaft_max  = (240*1000);
propulsion.left_cruiser.eta_v        = .85;
propulsion.left_cruiser.eta_add      = .7;
propulsion.left_cruiser.R            = .381;
propulsion.left_cruiser.r_hub        = .06;
propulsion.right_cruiser.N           = 1;
propulsion.right_cruiser.P_shaft_max = (240*1000);
propulsion.right_cruiser.eta_v       = .85;
propulsion.right_cruiser.eta_add     = .7;
propulsion.right_cruiser.R           = .381;
propulsion.right_cruiser.r_hub       = .06;

propulsion.wheels.mu_brk    = .5;
propulsion.wheels.mu_roll   = .02;

%%%%%%%%%%%%%%%%%%%%%%%%
%AERODYNAMIC DEFINITION%
%%%%%%%%%%%%%%%%%%%%%%%%
aero.Wing.e              = .75;
aero.Wing.x_ac           = .25*geometry.Wing.cbar;
aero.Htail.tau            = .65;
aero.Htail.cla            = 2*pi;
%%%%%%%%%%%%%%%%%%%%%%%%
%ENVIRONMENT DEFINITION%
%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%
%SIMULATION DEFINITION%
%%%%%%%%%%%%%%%%%%%%%%%


%Combine Data Structures
airplane.geometry       = geometry;
airplane.weights        = weights;
airplane.propulsion     = propulsion;
airplane.aero           = aero;
airplane.stability      = stability;

save("Airplane.mat","airplane")

