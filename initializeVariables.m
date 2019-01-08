close all; clc;

%%% this script is ran everytime run is hit, so all the variables are
%%% defined here first. The initial conditions for the simulator are also
%%% set here. Everything is in SI units. 

%% include aircraft path

addpath('BizJet_Test')

%% aircraft model properties

%run the initialization script that will create the aircraft data
GeoMassAero;

%mass is defined as 'm'

%inertia matrix is not defined the the components are. Collect them:

inertia_matrix = zeros(3);

inertia_matrix(1,1) = Ixx;
inertia_matrix(1,3) = -Ixz;
inertia_matrix(3,1) = -Ixz;
inertia_matrix(2,2) = Iyy;
inertia_matrix(3,3) = Izz;






%% Initial conditions

%run the script to define the initial conditions and then extract the
%properties into the simulink model

bizjet_test;

Xe_init = 0;
Ye_init = 0;
Ze_init = -hm; %m

initial_position = [Xe_init, Ye_init, Ze_init];

%true airspeed = V;

U_init = V * cos(alphar) * cos(betar) - windb(1); %m/s
V_init = V * sin(betar) - windb(2);
W_init = V * sin(alphar) * cos(betar) - windb(3);

initial_velocity = [U_init, V_init, W_init];



roll_init = phir;
pitch_init = thetar;
yaw_init = psir;

initial_orientation = [roll_init, pitch_init, yaw_init];

p_init = p * 0.01745329;
q_init = q * 0.01745329;
r_init = r * 0.01745329;
initial_rotation_rate = [p_init, q_init,r_init];

%print initial conditions vector:
disp('initial state:')
disp(x)
%print initial control inputs vector:
disp('initial control input:')
disp(u);
