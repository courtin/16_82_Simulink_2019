motorfile = 'sc3014.txt';
propfile  = 'cam9x6.txt';

speed = [0:5:65];
volt = [1:1:22];
S = length(speed);
V = length(volt);

RPM     = zeros(S,V);
Pshaft  = zeros(S,V);   %W 
T       = zeros(S,V);   %N
DV      = zeros(S,V);   %m/s
Q       = zeros(S,V);   %N*m
eta_m   = zeros(S,V);   
eta_p   = zeros(S,V);   
Amps   = zeros(S,V);   
for s = 1:S
    for v = 1:V
        cmd = ['~/tools/Qprop/bin/qprop ',propfile, ' ', motorfile, ' ', num2str(speed(s)),' 0 ',num2str(volt(v)) ' > result.out'];
        system(cmd);
        cmd
        [~, RPM(s,v), T(s,v), Q(s,v), Pshaft(s,v), ...
        Volts, Amps(s,v), eta_m(s,v), eta_p(s,v), DV(s,v)] = read_qprop_out('result.out');
        %VJ = V+DV;
    end
end

save('POC_motor_db.mat', 'RPM', 'Pshaft', 'T', 'DV', 'Q', 'eta_m', 'eta_p', 'Amps')