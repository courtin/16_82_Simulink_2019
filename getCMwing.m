function cm = getCMwing(a_w_deg,dCJ_B,flap_deg,airplane)


%load data
alpha_range = airplane.aero.Wing.fits.alpha_range;
dCJ_range = airplane.aero.Wing.fits.dCJ_range;
flaps_range = airplane.aero.Wing.fits.flaps_range;
cms = airplane.aero.Wing.fits.cms;


%%% TO DO: Include the flap range fitting to this function%%%%%%%


    if round(flap_deg) == 40 
        %Interpolate from wind tunnel data
        cm = interpn(alpha_range, dCJ_range, cms,a_w_deg, dCJ_B);
    else
        %Use linear TAT model with fixed cl_max
        cm = -.1; %WAG at reasonable wing CM
       
    end
    
    