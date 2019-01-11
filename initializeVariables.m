close all; clc;

%%% this script is ran everytime run is hit, so all the variables are
%%% defined here first. The initial conditions for the simulator are also
%%% set here. Everything is in SI units. 

%% include aircraft path

addpath('SSTOL_Aero_Model/')

%% aircraft model properties

%run the initialization script that will create the aircraft data
STOL_Input;

%mass is defined as 'm'
m=airplane.weights.MTOW;

%inertia matrix is not defined the the components are. Collect them:

inertia_matrix = zeros(3);

inertia_matrix(1,1) = airplane.weights.Ixx;
inertia_matrix(1,3) = -airplane.weights.Ixz;
inertia_matrix(3,1) = -airplane.weights.Ixz;
inertia_matrix(2,2) = airplane.weights.Iyy;
inertia_matrix(3,3) = airplane.weights.Izz;






%% Initial conditions

%run the script to define the initial conditions and then extract the
%properties into the simulink model

sstol_test_Simulink;

initial_position = x(10:12);

initial_velocity = [x(1), x(5), x(3)];

initial_orientation = [x(8), x(4), x(9)];

initial_rotation_rate = [x(6), x(3), x(7)];

%print initial conditions vector:
disp('initial state:')
disp(x)
%print initial control inputs vector:
disp('initial control input:')
disp(u);
