clear all
A=load('Airplane.mat');
airplane = A.airplane;
thr=0.5;
V=5;
flap_deg=0;

i=0;
alpha_list=-90:1:90;
for alpha=alpha_list
i=i+1;
[cl(i),cx(i),cm(i)] = get_coeffs_wing(alpha,flap_deg,V,airplane,thr);
end

plot(alpha_list,cm)
