function [CLde, CLq, ...
    Cmde, Cmq, ...
    CYb, CYdA, CYdR, CYr, CYp,...
    Clb, CldA, CldR, Clr, Clp,...
    Cnb, CndA, CndR, Cnr, Cnp] = get_JVL_derivatives(delta_flap, delta_CJ, airplane)

deg2rad=pi/180;
%flaps = airplane.stability.flap_settings;

%Check if the commanded flap setting is valid
df = delta_flap;
dcj=delta_CJ;

%if ~ismember(df, flaps, 'legacy')
%    df = 0;
%end

%Find the index of the commanded flap setting 
%i = find(flaps==df,1);
i=1;
%Get the correct value from the table
vect=[1 df dcj];
CLde    = airplane.stability.f_CLde*vect'./ deg2rad;
CLq     = airplane.stability.f_CLq*vect';
Cmde    = airplane.stability.f_Cmde*vect'./ deg2rad;
Cmq     = airplane.stability.f_Cmq*vect';
CYb     = airplane.stability.f_CYB*vect';
CYdA     = airplane.stability.f_CYdA*vect'./ deg2rad;
CYdR     = airplane.stability.f_CYdR*vect'./ deg2rad;
CYr     = airplane.stability.f_CYr*vect';
CYp     = airplane.stability.f_CYp*vect';
Clb     = airplane.stability.f_ClB*vect';
CldA     = airplane.stability.f_CldA*vect'./ deg2rad;
CldR     = airplane.stability.f_CldR*vect'./ deg2rad;
Clr     = airplane.stability.f_Clr*vect';
Clp     = airplane.stability.f_Clp*vect';
Cnb     = airplane.stability.f_CnB*vect';
CndA     = airplane.stability.f_CndA*vect'./ deg2rad;
CndR     = airplane.stability.f_CndR*vect'./ deg2rad;
Cnr     = airplane.stability.f_Cnr*vect';
Cnp     = airplane.stability.f_Cnp*vect';