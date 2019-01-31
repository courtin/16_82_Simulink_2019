function [CLde, CLq, ...
    Cmde, Cmq, ...
    CYb, CYdA, CYdR, CYr, CYp,...
    Clb, CldA, CldR, Clr, Clp,...
    Cnb, CndA, CndR, Cnr, Cnp] = get_JVL_derivatives(delta_flap, delta_CJ, airplane)


flaps = airplane.stability.flap_settings;

%Check if the commanded flap setting is valid
df = delta_flap;


if ~ismember(df, flaps, 'legacy')
    df = 0;
end

%Find the index of the commanded flap setting 
%i = find(flaps==df,1);
i=1;
%Get the correct value from the table
CLde    = airplane.stability.CLde(i);
CLq     = airplane.stability.CLq(i);
Cmde    = airplane.stability.Cmde(i);
Cmq     = airplane.stability.Cmq(i);
CYb     = airplane.stability.CYB(i);
CYdA     = airplane.stability.CYdA(i);
CYdR     = airplane.stability.CYdR(i);
CYr     = airplane.stability.CYr(i);
CYp     = airplane.stability.CYp(i);
Clb     = airplane.stability.ClB(i);
CldA     = airplane.stability.CldA(i);
CldR     = airplane.stability.CldR(i);
Clr     = airplane.stability.Clr(i);
Clp     = airplane.stability.Clp(i);
Cnb     = airplane.stability.CnB(i);
CndA     = airplane.stability.CndA(i);
CndR     = airplane.stability.CndR(i);
Cnr     = airplane.stability.Cnr(i);
Cnp     = airplane.stability.Cnp(i);