function [dCJ,Vj, T, CT] = propulsor_perf_qprop(thr, propulsor,c,S, alt, Vi) 
%%%%%%%%%%%%%%%%%%%%%%
%Computes two key propulsor outputs as a function of the throttle setting,
%aircraft state, and high-level parameters. 
%
%Inputs:    thr         Throttle setting for the propulsor (0-1)
%           propulsor   propulsor data structure
%           alt         Vehicle altitude
%           Vi          Flight speed
%           c           wing chord
%

if Vi==0
    Vi=0.01;
end
%Don't allow throttle less than zero
thr = max(thr, 0);

%Compute the thrust/jet velocity of a single propulsor
voltage=propulsor.max_voltage*thr;

[~, ~, rho] = int_std_atm(alt);

T_one=propulsor.f_thrust*[1 voltage Vi voltage^2 voltage*Vi Vi^2]';
T = T_one * propulsor.N;
%This is just to allow for the case where there
%are multiple non-blowers ganged into a single propulsor. 

Vj=propulsor.f_dv*[1 voltage Vi voltage^2 voltage*Vi Vi^2]'+Vi;
CT=T/(0.5*rho*S*Vi^2);


%Correct the coefficients for the multiple propulsors distributed across
%the span, if required
R = propulsor.R;
rh = propulsor.r_hub;

if propulsor.N > 1
    b = propulsor.b;
    hd_c = (pi*(R^2-rh^2)*propulsor.N)/(b*c);
    [~, ~, dCJ, ~, ~] = get_wake_coeffs(Vj,Vi, hd_c);
else
    hd_c = pi*(R^2-rh^2)/(2*R*c);
    [~, ~, dCJ, ~, ~] = get_wake_coeffs(Vj,Vi, hd_c);
end
