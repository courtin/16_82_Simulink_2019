%Blown Wing Lifting Line
clear all;
close all;

dCJ_i   = 4.5;
dF_i    = 40 * pi/180;
alfa    = 15 * pi/180;    %Angle of attack, rad
V       = 10;             %Speed, m/s
N       = 88;
fprintf(1,"===========================\n")
fprintf(1,"=       Linear Model      =\n")
fprintf(1,"===========================\n")
[CL_linear, CL_pos, ...
CDi, CXnet,CX_p, ...
CM,Cl, Cni, cl_nominal, e, ...
cls, cxs, cms, y_lin, a_i_lin] = run_LL(alfa, dCJ_i,dF_i,V,N, 1);
fprintf(1,"===============================\n")
fprintf(1,"=       Non-Linear Model      =\n")
fprintf(1,"===============================\n")
[CL,CDi,CX,Cl, y, a_i_nl] = run_NLL(alfa, dCJ_i,dF_i,V,N, 1);

f = figure();
plot(y_lin./.3048, a_i_lin)
hold on
plot(y(2:end-1)./.3048, -a_i_nl(2:end-1))
legend("linear", "non-linear")
ylabel('\alpha_i, rad')
xlabel('Spanwise location (ft)')
saveas(f,'induce_angle_comp.pdf')

M = 10;
dcjs = linspace(1,6,M);
alfas = [15, 20, 25].*pi/180;
CL = zeros(1,M);
CX = zeros(1,M);
dF = [35, 40, 45].*pi/180;
for k = 1:3
    figure()
    for j = 1:length(alfas)
        for i = 1:length(dcjs)
            [CL(i),CDi,CX(i),Cl, y, a_i_nl] = run_NLL(alfas(j), dcjs(i),dF(k),V,N, 0);
        end

        plot(dcjs, CL, '--')
        hold on 
        plot(dcjs, CX, '-')
        xlabel('CJ')
    end
    legend('CL - AoA:15', 'CX - AoA: 15','CL - AoA:20', 'CX - AoA: 20','CL - AoA:25', 'CX - AoA: 25')
end    