function [V, RPM, T_N, Q_Nm, Pshaft_W, ...
        Volts, Amps, eta_mot, eta_prop, DV] = read_qprop_sweep(filename)

fid = fopen(filename);

for i = 1:18
    l = fgetl(fid);
end
V       = [];
RPM     = [];
T_N     = [];
Q_Nm    = [];
Pshaft_W= [];
Volts   = [];
Amps    = [];
eta_mot = [];
eta_prop= [];
DV      = [];
i = 1;
while l ~= -1
    data = split(l);

    V(i) = str2num(data{2,1});
    RPM(i) = str2num(data{3,1});
    T_N(i) = str2num(data{5,1});
    Q_Nm(i) = str2num(data{6,1});
    Pshaft_W(i) = str2num(data{7,1});
    Volts(i) = str2num(data{8,1});
    Amps(i) = str2num(data{9,1});
    eta_mot(i) = str2num(data{10,1});
    eta_prop(i) = str2num(data{11,1});
    DV(i) = str2num(data{15,1}); 
    l = fgetl(fid);
    i=i+1;
end
end
