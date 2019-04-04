function [cla, cl0, a0, cl1] = get_blown_cla(alfa, dCJ, dF, cl_u, cd_u, alpha_u, cm_u)
    dalfa = -1 * pi/180;
    if dCJ == 0
        %Use lookup for bw02b airfoil
        [cl1,~] = get_unblown_coeffs(alfa*180/pi, cl_u, cd_u, alpha_u, cm_u);
        [cl2,~] = get_unblown_coeffs((alfa+dalfa)*180/pi, cl_u, cd_u, alpha_u, cm_u);
        cla = (cl2-cl1)/(dalfa);
        %cla = 2*pi;
        cl0 = cl1-cla*alfa;
    else
        %Use wind tunnel data
        [cl1,~,~] = get_coeffs_wing(alfa*180/pi,dCJ,dF*180/pi,1);

        [cl2,~,~] = get_coeffs_wing((alfa+dalfa)*180/pi,dCJ,dF*180/pi,1);

        cla = (cl2-cl1)/(dalfa);
        cl0 = cl1-cla*alfa;
    end
    
    a0 = -cl0/cla;
end

