%Blown Wing Lifting Line
clear all;
close all;

dCJ_i   = 4.5;
dF_i    = 40 * pi/180;
alfa    = 15 * pi/180; %Angle of attack, rad
V       = 10;             %Speed, m/s
N       = 150;   %Must be even

[CL,CDi,Cl, y] = run_NLL(alfa, dCJ_i,dF_i,V,N, 1);

