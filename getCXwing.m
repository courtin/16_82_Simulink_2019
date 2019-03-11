function cx = getCXwing(a_w_deg,dCJ_B,CT_B,flap_deg,airplane)


%load data
alpha_range = airplane.aero.Wing.fits.alpha_range;
dCJ_range = airplane.aero.Wing.fits.dCJ_range;
flaps_range = airplane.aero.Wing.fits.flaps_range;
cxs = airplane.aero.Wing.fits.cxs;


%%% TO DO: Include the flap range fitting to this function%%%%%%%


    if round(flap_deg) == 40 
        %Interpolate from wind tunnel data
        cx = interpn(alpha_range, dCJ_range, cxs,a_w_deg, dCJ_B);
    else
        cd0 = .02; %WAG at wing profile drag coefficient
        cx = cd0 - CT_B;      
    end
    
    