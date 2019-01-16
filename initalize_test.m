clear all; close all; clc;

alpha = 15;
deltaCJ =1;
delta_f = 40;


alpha_vals = -10:5:40;
deltaCJ_vals = 0:8;

load('SSTOL_Aero_Model/40df_fits.mat');

cl=zeros(length(deltaCJ_vals),length(alpha_vals));

for i =1:length(alpha_vals)
    for j =1:length(deltaCJ_vals)
        cl(j,i)=cl_fit(alpha_vals(i),deltaCJ_vals(j));
    end
end

figure(1)
carpet(alpha_vals,deltaCJ_vals,cl,0);
carpetlabel(alpha_vals,deltaCJ_vals,cl,0,0);