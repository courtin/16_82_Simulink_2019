clear all
close all
clc
%alpha = [0,8, 13];
alpha = 13;
A = length(alpha);
dCJ = [.5:.5:5.5];
C = length(dCJ);
dF = [10:10:75];
F = length(dF);
CL = zeros(C,F,A);
CL1 = zeros(C,F,A);
CX = zeros(C,F,A);
CLCX = zeros(C,F,A);

CL_max = zeros(A,F);
CL1_max = zeros(A,F);
CJ_max = zeros(A,F);
CJ1_max = zeros(A,F);
for a = 1:A
    for f = 1:F
        for c = 1:C
        fprintf(1, 'AoA: %3.1f dCJ %2.1f dF %2.1f', alpha(a), dCJ(c), dF(f))
        [CL(c,f,a),CL1(c,f,a), CX(c,f,a)] = run_NWVL(alpha(a), dCJ(c), dF(f), 0);
        CLCX(c,f,a) = CL(c,f,a)/CX(c,f,a);
        end
        CL_max(a,f) = interp1(CX(:,f,a),CL(:,f,a),0);
        CL1_max(a,f) = interp1(CX(:,f,a),CL1(:,f,a),0);
        CJ_max(a,f) = interp1(CX(:,f,a),dCJ,0);
    end

    surf(dF, dCJ, CL(:,:,a), CX(:,:,a))
    %surf(dF, dCJ, CLCX(:,:,a))
    ylabel('dCJ')
    xlabel('\delta_{flap}')
    zlabel('C_L')
    colorbar
    caxis([-1, 1])
    hold on
    plot3(dF, CJ_max(a,:), CL_max(a,:), 'r','linewidth',2)
    plot3(dF, CJ_max(a,:), CL1_max(a,:), 'y--', 'linewidth',2)
end