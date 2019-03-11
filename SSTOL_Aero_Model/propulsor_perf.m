function [dCJ, T, CT] = propulsor_perf(thr, propulsor,c,S, alt, Vi) 
%%%%%%%%%%%%%%%%%%%%%%
%Computes two key propulsor outputs as a function of the throttle setting,
%aircraft state, and high-level parameters. 
%
%Inputs:    thr         Throttle setting for the propulsor (0-1)
%           propulsor   propulsor data structure, containing
%               .eta_v
%               .eta_add
%               .R
%           alt         Vehicle altitude
%           Vi          Flight speed
%           c           wing chord
%

%Don't allow throttle less than zero
thr = max(thr, 0);

%Compute the thrust/jet velocity of a single propulsor
Pshaft_one = propulsor.P_shaft_max*thr;
[~, ~, rho] = int_std_atm(alt);

[T_one, Vj] = get_motor_T(Pshaft_one, Vi, rho,propulsor);
T = T_one * propulsor.N;
CT_calc=T/(0.5*rho*1.7*Vi^2);
%This is just to allow for the case where there
%are multiple non-blowers ganged into a single propulsor. 

%Correct the coefficients for the multiple propulsors distributed across
%the span, if required
R = propulsor.R;
rh = propulsor.r_hub;

if propulsor.N > 1
    b = propulsor.b;
    hd_c = (pi*(R^2-rh^2)*propulsor.N)/(b*c)
    [~, ~, dCJ, CT, ~] = get_wake_coeffs(Vj,Vi, hd_c);
    CT=CT*(b*c)/S;
else
    hd_c = pi*(R^2-rh^2)/(2*R*c)
    [~, ~, dCJ, CT, ~] = get_wake_coeffs(Vj,Vi, hd_c);
    CT=CT*(2*R*c)/S;
end
