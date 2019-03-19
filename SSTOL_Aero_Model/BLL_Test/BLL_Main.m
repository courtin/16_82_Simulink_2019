%Blown Wing Lifting Line
clear all;
close all;

dCJ_i   = 4;
dF_i    = 50 * pi/180;
alfa    = 20 * pi/180; %Angle of attack, rad
V       = 10;             %Speed, m/s
N       = 48;

[CL_linear, CL_pos, ...
CDi, CXnet,CX_p, ...
CM,Cl, Cni, cl_nominal, e, ...
cls, cxs, cms] = run_LL(alfa, dCJ_i,dF_i,V,N, 1);

