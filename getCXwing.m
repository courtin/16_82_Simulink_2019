function cx = getCXwing(a_w_deg,dCJ_B,flap_deg,airplane)


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
        %Use linear TAT model with fixed cl_max
        error('update getCXwing for non-40 degree flaps')
       
    end
    
    