function [xdotj] = LinMod(tj,xj)
%LINMOD Combine the state and control vectors for integration

global u
x = xj(1:12);
u = xj(13:end);
xdot = EoM3(tj,x);

xdotj = [xdot
    0
    0
    0
    0
    0
    0
    0
    0
    0];
end

