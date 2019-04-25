function [cl,cx,cm] = get_coeffs_wing(a_w_deg,flap_deg,V,airplane,thr_b,thr_c)

%for 10 degree motor mount (get_regression_coefficients(MAT,10,1,1,1), accurate as of 2/11/2019):
% cl_coeffs =[0.4807    0.0801   -0.0001    0.0137   -0.0000;
%            -0.0459    0.0392   -0.0000    0.0214   -0.0000];
% 
% 
% cx_coeffs =[0.3430   -0.0086    0.0005   -0.0034    0.0000; 
%            -1.1986    0.0136    0.0004    0.0158   -0.0001];
% 
% cm_coeffs = [-0.0346   -0.0009   -0.0000   -0.0031    0.0000;
%              0.0419    0.0003    0.0000   -0.0037    0.0000];
         
         
%April 25th 2019
cl_coeffs=[0.490927114504241,0.0803944513259610,-5.97042400014295e-05,0.0134407597127205,-1.05919019186962e-06;
    -0.0459684292278043,0.0391914220929362,-2.71016907726558e-06,0.0213771607218634,-1.98920123994317e-06];
cx_coeffs=[0.312377594540931,-0.00885854312502718,0.000552954739484276,-0.00226260338575595,3.99172634244547e-05;
    -1.19826977207631,0.0135032316623816,0.000410396375098594,0.0157944570100272,-6.33328477290926e-05];
cm_coeffs=[-0.0332558354208018,-0.000868583556481255,-2.24349435852672e-06,-0.00317198820079184,2.69054484675545e-07;
    0.0418882774945461,0.000322441716076417,3.64583226085055e-06,-0.00369143517007464,3.56224503348132e-07];
         
cbar = airplane.geometry.Wing.cbar;
S= airplane.geometry.Wing.S;
[~,~, T_b,~] = propulsor_perf_qprop(thr_b, airplane.propulsion.right_blower,cbar,S, 0 ,V*cosd(90-10));
[~,~, T_c,~] = propulsor_perf_qprop(thr_b, airplane.propulsion.right_blower,cbar,S, 0 ,V*cosd(90-10));
T_c=0.5*T_c;

[dCJ_B,~, ~,~] = propulsor_perf_qprop(thr_b, airplane.propulsion.right_blower,cbar,S, 0 ,V);
[dCJ_C,~, ~,~] = propulsor_perf_qprop(thr_c, airplane.propulsion.right_blower,cbar,S, 0 ,V);
dCJ_B=(airplane.propulsion.right_blower.N*dCJ_B+0.5*dCJ_C)/(airplane.propulsion.right_blower.N+.5);
CT=2*(T_b+T_c)/(0.5*1.225*S*V^2);

Cl_ps_90=CT*sind(flap_deg+90);
Cl_ps_n90=CT*sind(flap_deg-90);
Cx_ps_90=CT*cosd(flap_deg+90)+1.8;
Cx_ps_n90=CT*cosd(flap_deg+90)+1.8;

a=[1 dCJ_B]*cl_coeffs(:,3);
b=0;
c=[1 dCJ_B]*cl_coeffs(:,2);
d=[1 dCJ_B]*cl_coeffs(:,1);
alpha1=(-b+sqrt(b^2-3*a*c))/(3*a);
alpha2=(-b-sqrt(b^2-3*a*c))/(3*a);
alpha_max=max([alpha1 alpha2]);
alpha_min=min([alpha1 alpha2]);
%load data

cm=0;


if a_w_deg>=alpha_max
    [cl_amax,cx_amax,cm_amax]=regression_results(alpha_max, flap_deg, dCJ_B,cl_coeffs,cx_coeffs,cm_coeffs);
    cl=cl_amax+(a_w_deg-alpha_max)*(Cl_ps_90-cl_amax)/(90-alpha_max);
    cx=cx_amax+(a_w_deg-alpha_max)*(Cx_ps_90-cx_amax)/(90-alpha_max);
    cm=cm_amax-0.0035*(a_w_deg-alpha_max);
elseif a_w_deg<=alpha_min
    [cl_amin,cx_amin,cm_amin]=regression_results(alpha_min, flap_deg, dCJ_B,cl_coeffs,cx_coeffs,cm_coeffs);
    cl=cl_amin+(a_w_deg-alpha_min)*(Cl_ps_n90-cl_amin)/(-90-alpha_min);
    cx=cx_amin+(a_w_deg-alpha_min)*(Cx_ps_n90-cx_amin)/(-90-alpha_min);
    cm=cm_amin-0.0035*(a_w_deg-alpha_min);
else
    [cl,cx,cm]=regression_results(a_w_deg, flap_deg, dCJ_B,cl_coeffs,cx_coeffs,cm_coeffs);
end
%%% TO DO: Include the flap range fitting to this function%%%%%%%

% 
%     if round(flap_deg) == 40 
%         %Interpolate from wind tunnel data
%         [CL,CD,CM]=regression_results(a_w_deg, flap_deg, dCJ_B,CL_coeffs,CD_coeffs,CM_coeffs)
%     else
%         %Use linear TAT model with fixed cl_max
%         cl = min(2*pi*a_w_deg*pi/180, airplane.aero.Wing.cl_max_clean);
%     end
%     
    