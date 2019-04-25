clear all
A=load('Airplane.mat');
airplane = A.airplane;
    
cbar = airplane.geometry.Wing.cbar;
S= airplane.geometry.Wing.S;
V=5;
% cl_coeffs =[0.4807    0.0801   -0.0001    0.0137   -0.0000;
%            -0.0459    0.0392   -0.0000    0.0214   -0.0000];
% 
% 
% cx_coeffs =[0.3430   -0.0086    0.0005   -0.0034    0.0000; 
%            -1.1986    0.0136    0.0004    0.0158   -0.0001];
% 
% cm_coeffs = [-0.0346   -0.0009   -0.0000   -0.0031    0.0000;
%              0.0419    0.0003    0.0000   -0.0037    0.0000];

cl_coeffs=[0.490927114504241,0.0803944513259610,-5.97042400014295e-05,0.0134407597127205,-1.05919019186962e-06;
    -0.0459684292278043,0.0391914220929362,-2.71016907726558e-06,0.0213771607218634,-1.98920123994317e-06];
cx_coeffs=[0.312377594540931,-0.00885854312502718,0.000552954739484276,-0.00226260338575595,3.99172634244547e-05;
    -1.19826977207631,0.0135032316623816,0.000410396375098594,0.0157944570100272,-6.33328477290926e-05];
cm_coeffs=[-0.0332558354208018,-0.000868583556481255,-2.24349435852672e-06,-0.00317198820079184,2.69054484675545e-07;
    0.0418882774945461,0.000322441716076417,3.64583226085055e-06,-0.00369143517007464,3.56224503348132e-07];
         
flap_deg=0;
j=0

for thr=[1]
j=j+1
i=0
a_w_list=-90:1:90;
hold on;
for a_w_deg=a_w_list
    i=i+1;
    [dCJ_s,Vj, T_s, CT] = propulsor_perf_qprop(thr, airplane.propulsion.right_blower,cbar,S, 0 ,V*cosd(a_w_deg-10))
    [dCJ,~,~,~] = propulsor_perf_qprop(thr, airplane.propulsion.right_blower,cbar,S, 0 ,V)
    a=[1 dCJ]*cl_coeffs(:,3);
    b=0;
    c=[1 dCJ]*cl_coeffs(:,2);
    d=[1 dCJ]*cl_coeffs(:,1);
    %scatter([a_w_deg], [a*a_w_deg^3 + b*a_w_deg^2 + c*a_w_deg + d]);
    alpha_max=(-b+sqrt(b^2-3*a*c))/(3*a)
    alpha_min=(-b-sqrt(b^2-3*a*c))/(3*a)
    CT=2*T_s/(0.5*1.225*S*V^2)
    [cl(i,j),cx(i,j),cm(i,j)]=regression_results(a_w_deg, flap_deg, dCJ,cl_coeffs,cx_coeffs,cm_coeffs);
    [cl0,~,~]=regression_results(0, flap_deg, dCJ,cl_coeffs,cx_coeffs,cm_coeffs)
    cl_ps(i,j)=CT*sind(a_w_deg+flap_deg)+cl_airfoil(a_w_deg);%+cl0-CT*sind(flap_deg);
end
plot([a_w_list], cm)%,'-',[a_w_list],cl_ps,'*')
ylim([-1 1])
grid on
end



