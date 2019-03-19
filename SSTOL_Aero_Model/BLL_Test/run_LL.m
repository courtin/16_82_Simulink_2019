function [CL_linear, CL_pos, ...
            CDi, CXnet,CX_p, ...
            CM,Cl, Cni, cl_section, e, ...
            cls, cxs, cms, y] = run_LL(alfa, dCJ_i,dF_i,V,N, verbose)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function has geometry and control
%assumptions unique to the Spring 16.821 POC
%vehicle hard-coded!
%
%Outputs:
%CL_linear  - CL calculated using the linear lifing line coefficients
%CL_pos     - CL calculated by using the effective angle of attack
%distribution calculated using the lifting line, and looking up the 2D
%coefficients at each station/
%CDi        - Induced drag
%CX_p       - CX calulated using the effectinve AoA and integrating over he 2D
%coefficients
%CXnet      - CX_p+CDi
%Cm         - Pitching moment, calculated in the same manner as CL_pos and CX_p
%Cni        - Induced yawing moment coefficient
%Cl         - Induced rolling moment coefficient
%cl_section - Section cl at nominal angle of attack
%e          - span efficiency
%cls        - spanwise distribution of 2D lift coefficients
%cxs        - spanwise distribution of 2D cx coefficients
%cms        - spanwise distribution of 2D moment coefficients
%y          - control points for cls/cxs/cms
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Define Geometry and flight condition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = 13*.3048;       %span, m
rho = 1.225;        %Density, kg/m^3
     
twist = zeros(1,N); %twist distribution, rad
psi = linspace(pi, 0, N);   %Angle coordinate transform
%dpsi = (pi/(N-1));          
y = b/2*cos(psi);           %Corresponding y to psi distribution


%Calculate flap breaks in angle coordinates
% y_flap = 9.73*.3048/2;
% psi_flap = acos(2*y_flap/b);
% 
% r1 = get_cheb_nodes(psi_flap,0, n)
% r2 = get_cheb_nodes(pi-psi_flap, psi_flap, 2*n)
% r3 = get_cheb_nodes(pi,pi-psi_flap,n)
% 
% psi = [r1,r2,r3]
dpsi = zeros(1,N);
psi = linspace(pi, pi/N, N);
for i = 1:N-1
    dpsi(i) = -psi(i+1)+psi(i);
end
dpsi(end) = psi(end);
dy = -b/2*sin(psi).*dpsi;


%cla = 2*pi;
c_root = 1.5*.3048;
ytip = 1.75*.3048;
c_dist = ones(1,N)*c_root; %chord distribution, m
%POC wing
c_tip = .9*.3048;
for i = 1:length(c_dist)
    if abs(y(i)) > b/2-ytip
        yin = b/2-abs(y(i));
        c_dist(i) = c_tip + c_tip/c_root*yin*.3048 ;
    end
end
%c_dist = ctip+ctip*2/b*(b/2-abs(y));

%Asymmetric wing
%c_dist(1:N/2) = 1;
%c_dist(N/2:end) = 1;

S = cumtrapz(y,c_dist);
Sref = S(end);
AR = b^2/Sref;

%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%

M = N;
a = zeros(M,N);
r = zeros(M,1);
Psim = zeros(M,1);      %Psi_m; angle coordinate at index m
clas = zeros(M,1);      %cla distribution
cl0s = zeros(M,1);      %cl0 distribution
dFs = zeros(M,1);       %Flap distribution
CJs = zeros(M,1);       %Blowing distribution

%Collocation method; from Drela FVA, E.16 & E.17
for m = 1:M
    for n = 1:N
        %psim = pi*m/(N+1);
        psim = psi(m);
        Psim(m) = psim;
        c = get_c(psim, c_dist, y,b);
        
        [dCJ, dF] = get_CJ_dF(psim,b, dCJ_i, dF_i);
        CJs(m) = dCJ;
        dFs(m) = dF;
        if dF == 0
            cla = 2*pi;
            cl0 = 0;
        else
            [cla,cl0, cl_section] = get_blown_cla(alfa, dCJ, dF);
        end
        clas(m) = cla;
        cl0s(m) = cl0;
        a(m,n) = sin(n*psim)+c/(4*b)*cla*n*sin(n*psim)/sin(psim);
        r(m) = c/(4*b)*(cla*(alfa+twist(m))+cl0);
    end
end

A = a\r;        %Calculate Fourier coefficients

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Force and Moment Calculation%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CL_linear = pi*AR*A(1);

delta = 0;
for n = 2:N
    delta = delta + n*(A(n)/A(1))^2;        
    
end

CDi = CL_linear^2/(pi*AR)*(1+delta);
draw_c(c_dist, y,b)

a_i     = zeros(1,N);   %Spanwise distribution of downwash angle
clp     = zeros(1,N);   %Spanwise distribution of lift
cmp     = zeros(1,N);   %Spanwise distribution of rolling moment
cnp     = zeros(1,N);   %Spanwise distribution of yawing moment

for i = 1:N
    sum1 = 0;
    sum2 = 0;
    for n = 1:N
        a_i(i) = a_i(i)-n*A(n)*sin(n*psi(i))/sin(psi(i));
        clp(i) = clp(i) + A(n)*sin(n*psi(i))*sin(psi(i))*dpsi(i);
        cmp(i) = cmp(i) + A(n)*sin(n*psi(i))*sin(psi(i))*cos(psi(i))*dpsi(i);
        sum1   = sum1+A(n)*sin(n*psi(i))*sin(psi(i));
        sum2   = sum2+n*A(n)*sin(n*psi(i));
    end
    cnp(i) = sum1*sum2*cos(psi(i))*dpsi(i);
end

CL_check = 2*AR*sum(clp);
Cl = AR*sum(cmp);
Cni = AR*sum(cnp);

%%%%%%%%%%%%%%%%%%%%%%
%Calculate CX and CM %
%%%%%%%%%%%%%%%%%%%%%%
a_eff = alfa + a_i;
X = 0;
L = 0;
M = 0;
cls = zeros(1,N);
cxs = zeros(1,N);
cms = zeros(1,N);
for i = 1:N
    [cl, cx, cm] = get_coeffs_wing(a_eff(i)*180/pi, CJs(i), dFs(i).*180/pi,1);
    cls(i) = cl;
    cxs(i) = cx;
    cms(i) = cm;
    psim = pi*i/(N+1);
    c = get_c(psim, c_dist, y,b);
    X = X+cx*.5*rho*V^2*c*abs(dy(i));
    L = L+cl*.5*rho*V^2*c*abs(dy(i));
    M = M+cm*.5*rho*V^2*c^2*abs(dy(i));
end

CL_pos = L/(.5*rho*V^2*Sref);
CX_p   = X/(.5*rho*V^2*Sref);
CM     = M/(.5*rho*V^2*Sref*mean(c_dist));
  
figure()

plot(y./.3048, cls)
xlabel('Spanwise location (ft)')
hold on
plot(y./.3048, cxs)
plot(y./.3048, cms)
legend('c_l', 'c_x', 'c_m')
title('Section coefficient distribution')
%%%%%%%%%%%%%%%%%%%%%%%%
%Write and plot results%
%%%%%%%%%%%%%%%%%%%%%%%%
figure()
plot(y/.3048, a_i*180/pi)
title('Downwash angle')
ylabel('Deg')
xlabel('Spanwise location (ft)')

figure()
plot(y/.3048, a_eff*180/pi)
title('Effective angle')
ylabel('Deg')
xlabel('Spanwise location (ft)')


disp('  ')
disp('Initial Conditions')
disp('==================')
disp(['AoA      = ',num2str(alfa*180/pi),' deg'])
disp(['TAS      = ',num2str(V),' m/s'])
disp(['AR       = ',num2str(AR)])
disp(['Sref     = ',num2str(Sref),' m^2 (',num2str(Sref/.3048^2),' ft^2)'])

e = 1/(1+delta);

disp('  ')
disp('Resulting coefficients')
disp('==================')
disp(['CL (lift coefficient)   = ',num2str(CL_linear)])
disp(['CL (check)              = ',num2str(CL_check)])
disp(['CL (post-processed)     = ',num2str(CL_pos)])
disp(['CDi                     = ',num2str(CDi)])
disp(['CXp                     = ',num2str(CX_p)])
disp(['CXnet                   = ',num2str(CX_p+CDi)])
disp(['CM                      = ',num2str(CM)])
disp(['Cl (roll moment coeff.) = ',num2str(Cl)])
disp(['Cni                     = ',num2str(Cni)])
disp(['cl_section (nominal)    = ',num2str(cl_section)])
disp(['e                        = ',num2str(e)])
CXnet = CX_p+CDi;

end


%%%%%%%%%%%%%%%%%%
%Helper Functions%
%%%%%%%%%%%%%%%%%%

function c = get_c(psi, c_dist, y_dist, b)
    y = b/2*cos(psi);
    c = interp1(y_dist, c_dist, y);
end

function [dCJ, dF] = get_CJ_dF(psi,b, dCJ_i, dF_i)
    y = b/2*cos(psi);
    %Hard-coded for POC
    if abs(y)>9.73*.3048/2
        dCJ = 0;
        dF  = 0;
    else
        dCJ = dCJ_i;
        dF = dF_i;
    end
end

function c = draw_c(c_dist, y_dist,b)
    plot(y_dist, zeros(1,length(y_dist)), 'k')
    hold on 
    plot(y_dist,c_dist, 'k')
    plot([y_dist(1),y_dist(1)], [0, c_dist(1)], 'k')
    plot([y_dist(end),y_dist(end)], [0, c_dist(end)], 'k')
    axis([-b/2-1, b/2+1, -1,2])
    title('Wing planform')
end

function x = get_cheb_nodes(a,b, N)
    x = .5*(a+b)+.5*(b-a)*cos(pi/N*((1:(N))-.5));
end


