function [V] = vorvel(r, ra, rb)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%i = [-1,0,0;0,1,0;0,0,1];
%dri = i*dri;
%ra = ri-dri/2;
%rb = ri+dri/2;
% scatter3([ri(1)], [ri(2)], [ri(3)])
% scatter3([r(1)], [r(2)], [r(3)], 'b')
% scatter3([ra(1)], [ra(2)], [ra(3)])
% scatter3([rb(1)], [rb(2)], [rb(3)])
a = r-ra;
b = r-rb;

xh = [1;0;0];

na = norm(a);
nb = norm(b);

T1 = cross(a,b)/(na*nb+dot(a,b))*(1/na+1/nb);
T2 = cross(a,xh)/(na-dot(a,xh))*(1/na);
T3 = cross(b,xh)/(nb-dot(b,xh))*(1/nb);

V = 1/(4*pi)*(T1+T2-T3);
end

