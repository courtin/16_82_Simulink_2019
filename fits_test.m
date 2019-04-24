clear all
cl_coeffs =[0.4807    0.0801   -0.0001    0.0137   -0.0000;
           -0.0459    0.0392   -0.0000    0.0214   -0.0000];


cx_coeffs =[0.3430   -0.0086    0.0005   -0.0034    0.0000; 
           -1.1986    0.0136    0.0004    0.0158   -0.0001];

cm_coeffs = [-0.0346   -0.0009   -0.0000   -0.0031    0.0000;
             0.0419    0.0003    0.0000   -0.0037    0.0000];


flap_deg=0
j=0;
for dCJ_B=[0:1:4]
i=0;
j=j+1;
for a_w_deg=-60:1:60
    i=i+1;
    [cl(i,j),cx(i,j),cm(i,j)]=regression_results(a_w_deg, flap_deg, dCJ_B,cl_coeffs,cx_coeffs,cm_coeffs);
end
end
plot([-60:1:60], cl)
grid on