clear all
A=load('Airplane.mat');
airplane = A.airplane;
thr=0;
V=10;
flap_deg=2;
thr_B=0;
thr_C=0;c

i=0;
alpha_list=-90:1:90;
for alpha=alpha_list
i=i+1;
[cl(i),cx(i),cm(i)] = get_coeffs_wing(alpha,flap_deg,V,airplane,thr_B,thr_C);
end

plot(alpha_list,cx)
