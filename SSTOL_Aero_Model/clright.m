function cl_right = clright(a_w_deg, dCJ_BR)

s = load('40df_fits.mat');

cl_right = s.cl_fit(a_w_deg, dCJ_BR);
%cl_right= s.cl_fit(a_w_deg, dCJ_BR);