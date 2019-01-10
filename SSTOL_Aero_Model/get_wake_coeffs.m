function [CQ, CJ, dCJ, CT, hj_c] = get_wake_coeffs(Vj, Vi, hd_c)
%%%%%%%%%%%%%%%%%
%Computes the dimensionless jet coefficients as a function of jet velocity,
%flight condition, and propeller radius, as well as the downstream jet height.
%
%Inputs:    r_prop          Propeller radius (same units as c)
%           Vj              Wake jet velocity (same units as V_inf)
%           Vi              Free-stream velocity (same units as V_j)
%           c               Wing chord (same units as r_prop)
%
%Outputs:   CQ              Jet mass flow coefficient
%           CJ              Jet momentum coefficient
%           dCJ             Net jet momentum coefficient
%           CT              Thrust coefficient


hj_c = hd_c/2*(1+Vi/Vj);
CQ  = 1/2*(Vj/Vi+1)*hd_c;
CJ  = 2*CQ*Vj/Vi;
dCJ = 2*CQ*(Vj/Vi-Vi/Vj);
CT  = CJ - 2*CQ;
end