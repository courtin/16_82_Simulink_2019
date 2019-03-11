function cl_left = clleft(a_w_deg, dCJ_BL)

s = load('40df_fits.mat');

cl_left = s.cl_fit(a_w_deg, dCJ_BL);
%cl_right= s.cl_fit(a_w_deg, dCJ_BR);