%Compare propulsion performance models between the constant shaft power
%actuator disk case and the constant RPM case
addpath ../SSTOL_Aero_Model
close all; clear all;
%Useful unit conversion factors
ftsq2msq    = 0.0929;
ft2m        = 0.3048;
lbf2N       = 4.44822;
fpm2mps     = 0.00508;
kts2mps     = 0.51444;
nmi2m       = 1852.0;
in2m        = 0.0254;
slugftsq2kgmsq = 1.35581795;
deg2rad     = pi/180;

%Define test propulsor
propulsor.N           = 1;
propulsor.P_shaft_max = (240*1000);
propulsor.eta_v       = .85;
propulsor.eta_add     = .7;
propulsor.R           = .1143; %m
propulsor.r_hub       = .019;
propulsor.b           = propulsor.R*2;

%Pshaft = 467; %W
Pshaft = 290; %W
%%Set up speed range

V_nom = 40 * kts2mps;

V_offset = [-10:.1:10];

V = V_offset + V_nom;
rho = 1.225;

N = length(V);
T_act = zeros(1,N);
Vj_act = zeros(1,N);
CQ_act = zeros(1,N);
CJ_act = zeros(1,N);
dCJ_act = zeros(1,N);
hd_c = .4;    %Nominal hd_c
for n = 1:N
    [T_act(n), Vj_act(n)] = get_motor_T(Pshaft, V(n), rho,propulsor);
    CQ_act(n) = .5*(1+(Vj_act(n)/V(n)))*hd_c;
    CJ_act(n) = 2*CQ_act(n)*(Vj_act(n)/V(n));
    dCJ_act(n) = CJ_act(n) - 2*hd_c;
end

[V_qp, RPM_qp, T_N_qp, Q_Nm_qp, Pshaft_W_qp, ...
        Volts_qp, Amps_qp, eta_mot_qp, eta_prop_qp, DV_qp] = read_qprop_sweep("speed_sweep_volt.out");
[V_rpm, RPM_rpm, T_N_rpm, Q_Nm_rpm, Pshaft_W_rpm, ...
    Volts_rpm, Amps_rpm, eta_mot_rpm, eta_prop_rpm, DV_rpm] = read_qprop_sweep("speed_sweep_rpm.out");

Vj_qp = V_qp + DV_qp;
CQ_qp = .5*(1+(Vj_qp./V_qp))*hd_c;
CJ_qp = 2*CQ_qp.*(Vj_qp./V_qp);
dCJ_qp = CJ_qp - 2*hd_c;

Vj_rpm = V_rpm + DV_rpm;
CQ_rpm = .5*(1+(Vj_rpm./V_rpm))*hd_c;
CJ_rpm = 2*CQ_rpm.*(Vj_rpm./V_rpm);
dCJ_rpm = CJ_rpm - 2*hd_c;

h1 = figure();
hold on
plot(V./kts2mps, T_act./lbf2N)
plot(V_qp./kts2mps, T_N_qp./lbf2N)
plot(V_rpm./kts2mps, T_N_rpm./lbf2N)
xlabel("V_\infty (kts)")
ylabel("Thrust (lbf)")
legend("Actuator Disk", "QPROP - Constant 18V", "QPROP - Constant 13930 RPM")
title("Thrust Variation with Airspeed")
saveas(h1,'Thrust.pdf')

h2 = figure();
hold on
plot(V./kts2mps, Vj_act./kts2mps)
plot(V_qp./kts2mps, Vj_qp./kts2mps)
plot(V_rpm./kts2mps, Vj_rpm./kts2mps)
xlabel("V_\infty (kts)")
ylabel("V_j (kts)")
legend("Actuator Disk", "QPROP - Constant 18V", "QPROP - Constant 13930 RPM")
title("Wake Velocity Variation with Airspeed")
saveas(h2, 'Vj.pdf')

h3 = figure();
hold on
plot(V./kts2mps, dCJ_act)
plot(V_qp./kts2mps, dCJ_qp)
plot(V_rpm./kts2mps, dCJ_rpm)
xlabel("V_\infty (kts)")
ylabel("\Delta C_J")
legend("Actuator Disk", "QPROP - Constant 18V", "QPROP - Constant 13930 RPM")
title("Jet Momentum Coefficient Variation with Airspeed")
saveas(h3, 'dCJ.pdf')

h4 = figure();
hold on
plot([V(1), V(end)]./kts2mps, [Pshaft, Pshaft])
plot(V_qp./kts2mps, Pshaft_W_qp)
plot(V_rpm./kts2mps, Pshaft_W_rpm)
legend("Actuator Disk", "QPROP - Constant 18V", "QPROP - Constant 13930 RPM")
xlabel("V_\infty (kts)")
ylabel("P_{shaft} (W)")
saveas(h4, 'Pshaft.pdf')

h5 = figure();
hold on
plot(V_qp./kts2mps, RPM_qp)
plot(V_rpm./kts2mps, RPM_rpm)
legend("QPROP - Constant 18V", "QPROP - Constant 13930 RPM")
xlabel("V_\infty (kts)")
ylabel("RPM")
saveas(h5, 'RPM.pdf')

h6 = figure();
subplot(3,1,1)
hold on
plot(V_qp./kts2mps, Volts_qp)
plot(V_rpm./kts2mps, Volts_rpm)
legend("QPROP - Constant 18V", "QPROP - Constant 13930 RPM", 'Location', 'south')
xlabel("V_\infty (kts)")
ylabel("Voltage")
title("Motor electrical draw")

subplot(3,1,2)
hold on
plot(V_qp./kts2mps, Amps_qp)
plot(V_rpm./kts2mps, Amps_rpm)
xlabel("V_\infty (kts)")
ylabel("Current (A)")

subplot(3,1,3)
hold on
plot(V_qp./kts2mps, Volts_qp.*Amps_qp)
plot(V_rpm./kts2mps, Volts_rpm.*Amps_rpm)
xlabel("V_\infty (kts)")
ylabel("Motor Power (W)")

saveas(h6, 'elec.pdf')