function [cla,a0,cl0] = get_lin_model(alpha,dCJ, dF, unblown, verbose)
%Linearized section model for the POC vehicle, blending wind tunnel data
%and xfoil analysis of bw02b airfoil section.


alpha_lin = 10*pi/180; %Angle used for blown linear model
alpha_lin_ub = 10*pi/180; %Angle used for unblown linear model

if (dCJ == 0) && (alpha > alpha_lin_ub)
    alpha = alpha_lin_ub;
elseif alpha > alpha_lin
    alpha = alpha_lin;
end

[cla, cl0, a0, ~] = get_blown_cla(alpha, dCJ, dF, unblown.cl, unblown.cd, unblown.alpha, unblown.cm);

end

