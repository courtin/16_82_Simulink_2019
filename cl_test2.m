
A=load('Airplane.mat');
airplane = A.airplane;
thr=0;
V=8;

flap_list=0:10:90
for i=1:length(flap_list)

flap_deg=flap_list(i);
thr_B=0.5;
thr_C=0;

i=0;
alpha_list=-180:1:180;
for alpha=alpha_list
i=i+1;
[cl(i),cx(i),cm(i)] = get_coeffs_wing(alpha,flap_deg,V,airplane,thr_B,thr_C);
cl_t(i)=cl_airfoil(alpha);
end
plot(alpha_list,cl_t)
hold on
end
grid on
hold off