function [V, RPM, T_N, Q_Nm, Pshaft_W, ...
        Volts, Amps, eta_mot, eta_prop, DV] = read_qprop_out(filename)

fid = fopen(filename);

for i = 1:17
    l = fgetl(fid);
end
l = fgetl(fid);
data = split(l);
length(data)

V = str2num(data{2,1});
RPM = str2num(data{3,1});
T_N = str2num(data{5,1});
Q_Nm = str2num(data{6,1});
Pshaft_W = str2num(data{7,1});
Volts = str2num(data{8,1});
Amps = str2num(data{9,1});
if length(data{10,1}) >= 11%hack for qprop highspeed low power formatting error 
    eta_mot = str2num(data{10,1}(1:11));
    eta_prop = str2num(data{10,1}(11:end));
    DV = str2num(data{15,1});
elseif length(data) == 19
    eta_mot = str2num(data{10,1}(1:11));
    eta_prop = str2num(data{10,1}(11:end));
    DV = str2num(data{15,1});
else
    eta_mot = str2num(data{10,1});
    eta_prop = str2num(data{11,1});
    DV = str2num(data{15,1});
end


