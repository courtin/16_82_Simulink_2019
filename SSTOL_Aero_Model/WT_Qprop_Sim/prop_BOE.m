b = 1.2;
r = [.1:.1:b];
N = length(r);
cl = 1;
close all;
c = .12;

rho = 1.225;

omega = 1000; %RPM
omega = omega*(2*pi)/60;

V = omega*r;

Lp =.5*rho*V.^2*c*cl;

L = trapz(r, Lp);
S = trapz(r,c*ones(1,N));

plot(r,Lp, 'b');
hold on 
c_root = c;
c_tip  = .75*c_root;
c = zeros(1,N);
for i = 1:N
    c(i) = interp1([r(1), r(end)], [c_root, c_tip], r(i));
end

Lp =.5*rho*V.^2.*c*cl;
L3 = trapz(r,Lp);
plot(r,Lp, 'r');


cl_root = 1.2;
cl_tip = .2
cl = zeros(1,N);
for i = 1:N
    cl(i) = interp1([r(1), r(end)], [cl_root, cl_tip], r(i));
end

Lp =.5*rho*V.^2.*c_root.*cl;
L3 = trapz(r,Lp);
plot(r,Lp, 'r');
