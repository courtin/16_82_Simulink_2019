function [cla, cl0, cl1] = get_blown_cla(alfa, dCJ, dF)
    dalfa = 2 * pi/180;
    [cl1,~,~] = get_coeffs_wing(alfa*180/pi,dCJ,dF*180/pi,1);
    
    [cl2,~,~] = get_coeffs_wing((alfa+dalfa)*180/pi,dCJ,dF*180/pi,1);
    
    cla = (cl2-cl1)/(dalfa);
    cl0 = cl1-cla*alfa;
end

