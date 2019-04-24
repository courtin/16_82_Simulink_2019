%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PROFILE DRAG MODULE
%
%   This estimates the profile drag coefficient from a simple wetted area calculation and component
%   buildup.
%
%   All drag coefficients are referenced based on the wing reference area
%   Sref
%
%   This method uses the wing parameterization from TASOPT
%   (http://web.mit.edu/drela/Public/N+3/Final_Report_App.pdf), but without
%   the provision for a planform break (single taper/sweep). 
%
%   Profile drag of the wing, tails, and nacelles is from Raymer Ch. 12.
%   x/c_max is assumed to be .3
%
%   
%
%   OUTPUTS:
%
%   Params      CD_p            Float giving the total profile drag on the aircraft
%                               at the specified flight condition     
%               Drag_Decomp     Structure containing any other relevant
%                               drag values calculated in the buildup of CD_p 
%
%                           .CDp_f        Fuselage profile drag coefficient
%                           .CDp_w        Wing profile drag coefficient
%                           .CDp_ht       Htail profile drag coefficient
%                           .CDp_vt       Vtail profile drag coefficient
%
%                           --------TO IMPLEMENT------------
%                           .CDp_lg       Landing gear profile drag coefficient
%                           .CDp_nacelle  Propeller profile drag coefficient
%                           .CDp_spoiler  Spoiler profile drag coefficient 
%                           .CDp_flap     Flap profile drag coefficient
%
%   INPUTS:
%   
%   airplane    High-level airplane data structure.  Must contain the
%               following sub-variables. 
%               .geometry
%                        .Wing
%                             .AR                Main wing aspect ratio 
%                             .Sref              Main wing reference area
%                             .Swet              Main wing wetted area
%                             .x_c_m             Avg. airfoil max thickness
%                                                chordwise location 
%                             .t_c_avg           Avg. airfoil t/c
%                           .Htail
%                             .AR                Htail aspect ratio 
%                             .S                 Htail reference area
%                             .Swet              Htail wetted area
%                             .x_c_m             Avg. airfoil max thickness
%                                                chordwise location 
%                             .t_c_avg           Avg. airfoil t/c 
%
%                           .Vtail
%                             .AR                Vtail aspect ratio 
%                             .S                 Vtail area
%                             .Swet              wetted area
%                             .x_c_m             Avg. airfoil max thickness
%                                                chordwise location 
%                             .t_c_avg           Avg. airfoil t/c 
%                           .Fuse
%                             .l;                Fuselage length 
%                             .Swet              Fuselage wetted area
%                             .fr                Fuselage fineness ratio
%
%               .aerodynamics
%                           .Wing
%                                .e              Main wing span efficiency
%             
%
%   alt         Operating altitude (m) 
%   M           Operating Mach number (-) 
%   CL          Operating lift coefficient (-)
%
%
    
function [CDp, Drag_Decomp] = getProfileDrag(airplane, alt, M, config)
    flight_condition = airplane.sim.flight_condition; %Controls hot/cold day setting for atmosphere
    Sref = airplane.geometry.Wing.Sref;
    %Main wing 
    %Get geometry variables
    %CDp_w = wing_CDp(airplane.geometry.Wing.Sref,...
    %                airplane.geometry.Wing.Swet,...
    %                airplane.geometry.Wing.x_c_m,...
    %                airplane.geometry.Wing.t_c_avg,...
    %                airplane.geometry.Wing.c_ma,...
    %                alt, M, flight_condition);
    CDp_w = 0;
    Drag_Decomp.CDp_w = CDp_w;
    
    %HTail
    CDp_h = wing_CDp(Sref,...
                    airplane.geometry.Htail.Swet,...
                    airplane.geometry.Htail.x_c_m,...
                    airplane.geometry.Htail.t_c_avg,...
                    airplane.geometry.Htail.c_ma,...
                    alt, M, flight_condition);
    
    CDp_h = CDp_h*1.1; %10% factor for control surfaces
    
    Drag_Decomp.CDp_h = CDp_h;
    
    %VTail
    CDp_v = wing_CDp(Sref,...
                    airplane.geometry.Vtail.Swet,...
                    airplane.geometry.Vtail.x_c_m,...
                    airplane.geometry.Vtail.t_c_avg,...
                    airplane.geometry.Vtail.c_ma,...
                    alt, M, flight_condition);
    
    CDp_v = CDp_v*1.1; %10% factor for control surfaces
    
    Drag_Decomp.CDp_v = CDp_v;
    
    %Fuselage
    fr_f    = airplane.geometry.Fuse.fr;
    Swet_f  = airplane.geometry.Fuse.Swet;
    
    Re_f = get_Re(airplane.geometry.Fuse.l, alt, M, flight_condition);
    FF_f = (1+60/fr_f^3+fr_f/400);
    Cf_f = .455/(log10(Re_f))^2.58;
    
    CDp_f = FF_f*Cf_f*Swet_f/Sref;
    
    Drag_Decomp.CDp_f = CDp_f;
    
    %Flap
    %Drag from flaps 
%     switch config
%         case 'landing'
%             beta_flap   = airplane.aero.delta_flap_land;
%             beta_spoil  = airplane.aero.delta_spoil_land;
%         case 'clean'
%             beta_flap   = 0;
%             beta_spoil  = 0;
%     end
%     f_b_flap = airplane.geometry.Wing.f_flap_span;
%     f_c_flap = airplane.geometry.Wing.f_flap_chord;
%     b        = airplane.geometry.Wing.b;
%     c        = airplane.geometry.Wing.c_ma;
%     A_flap_tot = f_b_flap*b*c*f_c_flap;
%     A_flap_stream = A_flap_tot*sind(beta_flap);
%     
    %Atmosphere
    [~, ~, rho, a, mu] = int_std_atm(alt, airplane.sim.flight_condition);
    %Flat plate drag
%     CDp_flap = (1.0*beta_flap/90.0)*A_flap_stream/Sref;
%     
%     %Spoiler
%     %Drag from spoilers. 
%     f_b_spoil = airplane.geometry.Wing.f_spoil_span;
%     f_c_spoil = airplane.geometry.Wing.f_spoil_chord;
%     b        = airplane.geometry.Wing.b;
%     c        = airplane.geometry.Wing.c_ma;
%     A_spoil_tot = f_b_spoil*b*c*f_c_spoil;
%     A_spoil_stream = A_spoil_tot*sind(beta_spoil);
%     
%     %Flat plate drag
%     CDp_spoil = 1.0*A_spoil_stream/Sref;
    CDp_spoil = 0;
    CDp_flap = 0;
    %Total
    
    CDp = CDp_w+CDp_h+CDp_v+CDp_f+CDp_flap+CDp_spoil;
    
    
end

function [CDp] = wing_CDp(Sref, Swet, x_c_m, t_c, c_ma, alt, M, fc)
    FF      = 1+(.6/x_c_m)*(t_c)+100*(t_c)^4;
    
    Re      = get_Re(c_ma, alt, M, fc);
    
    Cf      = .455/(log10(Re))^2.58;     %Assume fully turbulent flow
    
    CDp     = Cf*FF*Swet/Sref;
end