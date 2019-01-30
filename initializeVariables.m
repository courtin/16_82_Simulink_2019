clear all; close all; clc;

%%% this script is ran everytime run is hit, so all the variables are
%%% defined here first. The initial conditions for the simulator are also
%%% set here. Everything is in SI units. 

%% include aircraft path

addpath('SSTOL_Aero_Model/')

%% aircraft model properties

%run the initialization script that will create the aircraft data
STOL_Input;

%mass is defined as 'm'

m=airplane.weights.MTOW/airplane.environment.g;

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

initial_velocity = [x(1), x(5), x(2)];

initial_orientation = [x(8), x(4), x(9)];

initial_rotation_rate = [x(6), x(3), x(7)];

%print initial conditions vector:
disp('initial state:')
disp(x)
%print initial control inputs vector:
disp('initial control input:')
disp(u);


%% set up wing coefficients

alpha_range = -10:5:40;
dCJ_range = 0:8;
flap_range = 40;

load('SSTOL_Aero_Model/40df_fits.mat');

cls=zeros(length(dCJ_range),length(alpha_range),length(flap_range));
cxs=zeros(length(dCJ_range),length(alpha_range),length(flap_range));
cms=zeros(length(dCJ_range),length(alpha_range),length(flap_range));

for k = 1:length(flap_range)
    for i =1:length(alpha_range)
        for j =1:length(dCJ_range)
            cls(j,i,k)=cl_fit(alpha_range(i),dCJ_range(j));
            cxs(j,i,k)=cx_fit(alpha_range(i),dCJ_range(j));
            cms(j,i,k)=cm_fit(alpha_range(i),dCJ_range(j));
        end
    end
end

figure;
carpet(alpha_range,dCJ_range,cls,0);
carpetlabel(alpha_range,dCJ_range,cls,0,0);
title('Carpet Plot of CL')

figure;
carpet(alpha_range,dCJ_range,cxs,0);
carpetlabel(alpha_range,dCJ_range,cxs,0,0);
title('Carpet Plot of CX')

figure;
carpet(alpha_range,dCJ_range,cms,0);
carpetlabel(alpha_range,dCJ_range,cms,0,0);
title('Carpet Plot of CM')


airplane.aero.fits.cl.cls = cls;
airplane.aero.fits.cl.alpha_range = alpha_range;
airplane.aero.fits.cl.dCJ_range = dCJ_range;
airplane.aero.fits.cl.flap_range = flap_range;




