function [T, Vj] = get_motor_T(Pshaft, V, rho,propulsor)
%%%%%%%%%%%%%%%%%%%%%
%Computes motor thrust as a function of input power, motor size, and flight
%condition from actuator disk theory. 
%
%Inputs:    propulsor           Propulsor object, which contains the
%                               following attributes:
%               .eta_v          Prop viscous losses
%               .eta_add        Prop swirl losses
%               .R               Prop radius, m
%           Pshaft              Motor shaft power, W
%           V                   Freestream velocity, m/s
%           rho                 air density, kg/m^3
%
%Outputs:   T                   Net thrust produced by the disk in
%                               isolation, N
%           Vj                  Far downstream wake velocity, m/s

eta_v = propulsor.eta_v;
eta_add = propulsor.eta_add;
R = propulsor.R;

eta_p_i = .9;
err = 1e6;
thr = 1e-3;
iter = 0;
itermax = 100;


eta_i = 1;

T=0;


while err > thr && iter < itermax
    T = Pshaft*eta_p_i/V
    Tc = T/(rho*V^2*.5*pi*R^2)
    eta_i = 2/(2+(sqrt(1+Tc)-1)/eta_add)
    eta_p = eta_i*eta_v
    err = abs(eta_p_i - eta_p)
    eta_p_i = eta_p
    iter = iter+1
end

disp(err)
disp(iter)

w = V*(1/eta_i-1);
Vj = V + 2*w;
