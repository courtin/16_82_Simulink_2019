function unblown = setup_VLaero(airplane)
%SETUP_VLAERO Setup the aero inputs to the VL method, which currently
%consists of loading the lookup tables for the unblown airfoil.
%Load unblown airfoil polar

load bw02b_polar.mat cl cd cm alpha;
unblown.cl = cl;
unblown.cd = cd;
unblown.cm = cm;
unblown.alpha = alpha;

end

