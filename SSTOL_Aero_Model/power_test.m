P = 550;

R = .1;

V = 0;
rho = 1.225;
eta_v = .85;
eta_add = .7;


eta_p_i = .9;
err = 1e6;
thr = 1e-3;
iter = 0;
itermax = 100;
while err > thr && iter < itermax
    T = P*eta_p_i/V;
    Tc = T/(rho*V^2*.5*pi*R^2);
    eta_i = 2/(2+(sqrt(1+Tc)-1)/eta_add);
    eta_p = eta_i*eta_v;
    err = abs(eta_p_i - eta_p);
    eta_p_i = eta_p;
    iter = iter+1;
end

w = V*(1/eta_i-1);
Vj = V + 2*w
