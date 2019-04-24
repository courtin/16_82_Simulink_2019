%Vehicle state transitions from cruise to approach configurations.

V = [20:-.1:5.6];
W = 36 * 4.45;
Sref = 17.95*.3048^2;
bref = 13*.3048;
bh = 4.4*.3048;
bv = 2.3*.3048;
rho = 1.225;

CLr = 2*W./(rho*V.^2*Sref);

CLm_clean   = 1.4;
CLm_ubf     = 1.7;
CL8         = 2.5;
CL7         = 3.2;
%CL6         = 3.2;

f1 = figure()
plot(V, CLr);
set(gca,'xdir','reverse', 'fontsize', 15)

hold on

Vm_clean    = interp1(CLr, V, CLm_clean);
Vm_ubf      = interp1(CLr, V, CLm_ubf);
Vm_8        = interp1(CLr, V, CL8);
Vm_7        = interp1(CLr, V, CL7);

plot([V(1), Vm_clean], [CLm_clean, CLm_clean], 'k--')
plot([Vm_clean, Vm_clean], [0, CLm_clean], 'k--')

plot([V(1), Vm_ubf], [CLm_ubf, CLm_ubf], 'k--')
plot([Vm_ubf, Vm_ubf], [0, CLm_ubf], 'k--')

plot([V(1), Vm_8], [CL8, CL8], 'k--')
plot([Vm_8, Vm_8], [0, CL8], 'k--')

plot([V(1), Vm_7], [CL7, CL7], 'k--')
plot([Vm_7, Vm_7], [0, CL7], 'k--')

xlabel('V (m/s)')
ylabel('C_L')

saveas(f1,'decel_profile.pdf')

