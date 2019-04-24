function [cl, cd] = get_cl(alpha)
%Placeholder function for a fit to a cl-alpha curve.
%   Simple TAT model and constant drag
cl_max = 1.8;

cl = max(min(2*pi.*alpha, cl_max), -cl_max);

cd = .01;
end
