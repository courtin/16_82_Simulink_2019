function cl = getCLwing(a_w_deg,dCJ_B,flap_deg,airplane)


%load data
alpha_range = airplane.aero.Wing.fits.alpha_range;
dCJ_range = airplane.aero.Wing.fits.dCJ_range;
flaps_range = airplane.aero.Wing.fits.flaps_range;
cls = airplane.aero.Wing.fits.cls;


%%% TO DO: Include the flap range fitting to this function%%%%%%%


    if round(flap_deg) == 40 
        %Interpolate from wind tunnel data
        cl = interpn(alpha_range, dCJ_range, cls,a_w_deg, dCJ_B);
    else
        %Use linear TAT model with fixed cl_max
        cl = min(2*pi*a_w_deg*pi/180, airplane.aero.Wing.cl_max_clean);
    end
    
    