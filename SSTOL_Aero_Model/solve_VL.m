function [CL,CX,CM, eps] = solve_VL(alpha_in, dCJ_in, dF_in, airplane, verbose)
%SOLVE_VL Solves the linear WVL model for a given motor setting, flap
%setting, and airplane model which contains the results of the setup_VLgeom
%script

useTAT = 0;

%Flight condition
%alpha = alpha_in*pi/180;   %Alpha should be in rad already
alpha = alpha_in;
beta = 0;
rho = 1;
Vinf = 1;
Ubar = [-(cos(alpha)*cos(beta));sin(beta);-sin(alpha)*cos(beta)];   %Can easily update for p,q,r, etc.
dCJ = dCJ_in;
dF = dF_in; %Flaps should be in rad already
N = airplane.vl.N;

%dF = dF_in*pi/180;

%Floor dCJ at zero for lift calculations; bookkeep as drag later
for i = 1:length(dCJ)
    dCJ(i) = max(dCJ(i),0);
end

if useTAT
    %TAT_model - control point at 3c/4
    control = airplane.vl.vortex;
    cs = airplane.vl.cs;
    for i = 1:length(control)
        control(1,i) = control(1,i) + cs(i)/2; 
    end
    a0s = zeros(1,N);
else
    %Locate control point depending on local cla
    
    [diff_t, idt, it] = unique(dCJ);    %Figure out how many different throttle settings there are. 
    [diff_f, idf, iff] = unique([dF,0]);     %Figure out how many different flap settings there are. 
    T = length(diff_t);
    F = length(diff_f);
    
    combos = zeros(F,T);
    
    cla_diff    = zeros(F,T);                    %Get the cla of each unique motor/flap setting
    a0_diff     = zeros(F,T);                    %Get the a0 of each unique motor/flap setting
    cl0_diff    = zeros(F,T);                    %Get the cl0 of each unique motor/flap setting
    
    dCJ_diff = zeros(1,T);
    for i = 1:T
        for j = 1:F
            [cla_diff(j,i),a0_diff(j,i),cl0_diff(j,i)]  = get_lin_model(alpha,diff_t(i), diff_f(j), airplane.vl.unblown, 0);
        end
    end
    C_motor = airplane.vl.C_motor;
    C_flap = airplane.vl.C_flap;
    
    dCJ_vl = C_motor * (dCJ');
    dF_vl  = C_flap * (dF');
    control = airplane.vl.vortex;
    vortex  = airplane.vl.vortex;
    cs = airplane.vl.vortex_cs;
    a0s = zeros(1,N);
    for i = 1:length(control)
        cj_l = dCJ_vl(i);
        dF_l = dF_vl(i);
        c_l  = cs(i);
        cla_l = interp2(diff_f, diff_t, cla_diff, dF_l, cj_l);
        control(1,i) = control(1,i) + cla_l/(2*pi)*c_l/2;
        a0s(i) = interp2(diff_f, diff_t, a0_diff, dF_l, cj_l);
    end
end

%Create AIC matrix
A = zeros(N,N);
n = zeros(3,N);
ra = airplane.vl.ra;
rb = airplane.vl.rb;
for i = 1:N
    for j = 1:N
            panel_n = getNormal(vortex(:,i)-control(:,i), ra(:,i)-rb(:,i));
            a_aero = -a0s(i);
            Raero = [cos(a_aero),0,sin(a_aero);0,1,0;-sin(a_aero),0,cos(a_aero)];
            n(:,i) = Raero*panel_n;
            Vh = vorvel(control(:,i), ra(:,j), rb(:,j));
            A(i,j) = dot(Vh,n(:,i));
    end
end


Ainv = inv(A);

r= zeros(N,1);

xh = [1;0;0];
yh = [0;1;0];
zh = [0;0;1];

%Set up RHS of solution matrix
for k = 1:N
    r(k) = Ubar(1)*dot(xh,n(:,k)) ...
            + Ubar(2)*dot(yh,n(:,k)) ...
            + Ubar(3)*dot(zh,n(:,k));
end

%Solve for circulation
Gamb = Ainv*r;

%Calculate velocity distribution
Vbar    = zeros(3, N);
Fi      = zeros(3, N);
ls      = zeros(3, N);
ai      = zeros(1,N);
a_eff   = zeros(1,N);

for i = 1:N
    for j = 1:N
        LBOUND = ~(i==j);% || i == j+1 || i == j-1);
        %LBOUND = 0;
        ri = .5*(ra(:,i)+rb(:,i));
        Vh = vorvel2(ri, ra(:,j), rb(:,j),LBOUND);
        Vbar(:,i) = Vbar(:,i) + Vh*Gamb(j);
    end
    Vbar(:,i) = Vbar(:,i)-Ubar;
    a_eff(i) = Vbar(3,i)/Vbar(1,i);
    ai(i) = -(alpha - a_eff(i));
end
Sref = airplane.vl.Sref;
b    = airplane.vl.bref;
T = [cos(alpha),0,sin(alpha);0,1,0;-sin(alpha),0,cos(alpha)];

%Solve for force and downwash distribution
V = Vbar*Vinf;
for i = 1:N
    l = (rb(:,i)-ra(:,i));
    ls(:,i) = l;
    Fi(:,i) = rho*Vinf*Gamb(i)*cross(V(:,i), l);
end

Fbari = Fi./(.5*rho*Vinf^2*Sref);

Fbar = zeros(3,1);

Fbar(1) = sum(Fbari(1,:));
Fbar(2) = sum(Fbari(2,:));
Fbar(3) = sum(Fbari(3,:));

ai2 = Fbari(1,:)./Fbar(3,:);
%Caluculate coefficients

Coeffs = T*Fbar;
Cdi = Coeffs(1);
CY = Coeffs(2);
CL = Coeffs(3);

cbar = Sref/b;

cl_t = zeros(N,1);
cx_t = zeros(N,1);
cm_t = zeros(N,1);
cls = zeros(N,1);
cxs = zeros(N,1);
cms = zeros(N,1);

[Di, ai_T] = trefftz(Gamb, vortex, ra, rb, 1);
a_eff = alpha + ai_T;
if useTAT
    %TaT mode
    cl_t = 2*pi.*a_eff';
    cx_t = .0;
    cm_t = -.1;
else
    for i = 1:N
        if dCJ_vl(i) == 0
            %Use lookup for bw02b airfoil
            [cl_t(i),cx_t(i), cm_t(i)] = get_unblown_coeffs(a_eff(i)*180/pi, airplane.vl.unblown.cl, airplane.vl.unblown.cd, airplane.vl.unblown.alpha, airplane.vl.unblown.cm);
        else
            %Use wind tunnel data
            [cl_t(i),cx_t(i),cm_t(i)] = get_coeffs_wing(a_eff(i)*180/pi,dCJ_vl(i),dF_vl(i)*180/pi,1);
        end
    end
end

cls = cl_t.*cos(-ai_T') - cx_t.*sin(-ai_T');
cxs = cl_t.*sin(-ai_T') + cx_t.*cos(-ai_T');
y = vortex(2,:);

CL2 = trapz(y,cls.*cs')/Sref;
CXp  = trapz(y,cxs.*cs')/Sref;
CM  = trapz(y,cm_t.*cs')/(Sref*cbar);

CX = CXp + Cdi;
AR = b^2/Sref;
CDi2 = CL^2/(pi*AR);

%Estimate tail downwash
Nh = 10;
bh = airplane.geometry.Htail.b;
yh = linspace(-bh/2, bh/2, Nh);
ai_h = zeros(1, Nh);
for i = 1:Nh
    ai_h(i) = interp1(y, ai_T, yh(i));
end
eps = ai_h*2;
if verbose
    disp('  ')
    disp('Initial Conditions')
    disp('==================')
    disp(['AoA      = ',num2str(alpha*180/pi),' deg'])
    disp(['TAS      = ',num2str(Vinf),' m/s'])
    disp(['AR       = ',num2str(AR)])
    disp(['Sref     = ',num2str(Sref),' m^2 (',num2str(Sref/.3048^2),' ft^2)'])


    disp('  ')
    disp('Resulting coefficients')
    disp('==================')
    disp(['CL (from F matrix)      = ',num2str(CL)])
    disp(['CL (check, from gamma)  = ',num2str(CL1)])
    disp(['CL (check, from a_eff)  = ',num2str(CL2)])
    disp(['cl (nominal)            = ',num2str(cl_nom_b)])
    disp(['CDi (from F matrix)     = ',num2str(Cdi)])
    disp(['CDi (check,CL^2/(AR*pi))= ',num2str(CDi2)])
    disp(['CDi (trefftz))          = ',num2str(Di/(.5*rho*Vinf^2*Sref))])
    disp(['CXp                     = ',num2str(CXp)])
    disp(['CXnet                   = ',num2str(CX)])
    disp(['CM                      = ',num2str(CM)])
    disp(['CY                      = ',num2str(CY)])
    %disp(['Cl (roll moment coeff.) = ',num2str(Cl)])
    %disp(['Cni                     = ',num2str(Cni)])
    %disp(['cl_section (nominal)    = ',num2str(cl_section)])
    %disp(['e                        = ',num2str(e)])
end
end


