function [cla_b,cla_ub,...
    a0_b,a0_ub,...
    cl_nom_b,cl_nom_ub,...
    cl0_b,cl0_ub] = POC_lin(alpha_nom,dCJ, dF, verbose)
%Linearized section model for the POC vehicle, blending wind tunnel data
%and xfoil analysis of bw02b airfoil section.

%Load unblown airfoil polar
load bw02b_polar.mat cl cd alpha cm;
cl_unblown = cl;
cd_unblown = cd;
cm_unblown = cm;
alpha_unblown = alpha;



alpha_lin = min(alpha_nom,20*pi/180); %Angle used for blown linear model
alpha_lin_ub = min(alpha_nom,10*pi/180); %Angle used for unblown linear model
if verbose
    %M&S Theory
    CLdf_ms = 2*sqrt(pi*dCJ)*sqrt(1+.151*sqrt(dCJ) + .139*dCJ);
    CLa_ms = 2*pi.*(1+.151*sqrt(dCJ) + .219*dCJ);

    AoAs = [-5:1:30];   
    I = length(AoAs);
    cls_b = zeros(1,I);
    cls_ub = zeros(1,I);
    cxs_b = zeros(1,I);
    cxs_ub = zeros(1,I);
    cl_ms = zeros(1,I);
    for i = 1:I
        [cls_ub(i), cxs_ub(i)] =  get_unblown_coeffs(AoAs(i), cl_unblown, cd_unblown, alpha_unblown, cm_unblown);
        [cls_b(i), cxs_b(i)] =  get_coeffs_wing(AoAs(i),dCJ,dF*180/pi,1);
        cl_ms(i) = CLdf_ms*dF + CLa_ms*AoAs(i)*pi/180;
    end
    figure()
    plot(AoAs, cls_ub, 'b')
    hold on
    plot(AoAs, cls_b, 'r')
    plot(AoAs, cl_ms, '--g')
    xlabel('Angle of attack (deg)')
    ylabel('c_l')
end
[cla_ub, cl0_ub, a0_ub, cl_nom_ub] = get_blown_cla(alpha_lin_ub, 0, 0, cl_unblown, cd_unblown, alpha_unblown, cm_unblown);

if verbose
    plot([a0_ub, alpha_lin_ub].*180/pi, [0, interp1(AoAs, cls_ub, alpha_lin_ub*180/pi)], 'kx--')
end
if dCJ ~= 0 %only use the wind tunnel model if there is blowing
    [cla_b, cl0_b, a0_b, cl_nom_b] = get_blown_cla(alpha_lin, dCJ, dF, cl_unblown, cd_unblown, alpha_unblown, cm_unblown);
else
    cla_b = cla_ub;
    cl0_b = cl0_ub;
    a0_b = a0_ub;
    cl_nom_b = cl_nom_ub;
end
if verbose
    plot([a0_b, alpha_lin].*180/pi, [0, interp1(AoAs, cls_b, alpha_lin*180/pi)], 'kx--')
    legend('Unblown section', ['Blown Section: dCJ=',num2str(dCJ),', dF=',num2str(dF*180/pi)],'TAT Estimate','Linear Fit', 'Location', 'northwest')
    title('2D Sections Lift Curve & Linear Approximations')
    figure()
    plot(AoAs, cxs_ub, 'b')
    hold on
    plot(AoAs, cxs_b, 'r')
    xlabel('Angle of attack (deg)')
    ylabel('c_x')
    legend('Unblown section', ['Blown Section: dCJ=',num2str(dCJ),', dF=',num2str(dF*180/pi)], 'Location', 'northwest')
    title('2D Section c_x curves')
end
end

