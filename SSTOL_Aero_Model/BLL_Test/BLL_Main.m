%Blown Wing Lifting Line
clear all;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Define Geometry and flight condition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = 13*.3048;             %span, m
%Sref = 12;          %reference area
alfa = 15 * pi/180; %Angle of attack, rad
V = 10;             %Speed, m/s
N = 4*50;             %Number of integration points
rho = 1.225;        %Density, kg/m^3
     

dCJ_i = 4.5;
dF_i = 40*pi/180;
%cla = 2*pi;         %lift slope coefficient, currently constant

%cla = 2*pi;
c_dist = ones(1,N)*1.5*.3048; %chord distribution, m
twist = zeros(1,N); %twist distribution, rad
psi = linspace(pi, 0, N);   %Angle coordinate transform
dpsi = (pi/(N-1));          
y = b/2*cos(psi);           %Corresponding y to psi distribution
dy = -b/2*sin(psi)*dpsi;

%Tapered wing
ctip = .5;
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
        psim = pi*m/(N+1);
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
CL = pi*AR*A(1);

delta = 0;
for n = 2:N
    delta = delta + n*(A(n)/A(1))^2;        
    
end

CDi = CL^2/(pi*AR)*(1+delta);
draw_c(c_dist, y,b)

a_i     = zeros(1,N);   %Spanwise distribution of downwash angle
clp     = zeros(1,N);   %Spanwise distribution of lift
cmp     = zeros(1,N);   %Spanwise distribution of rolling moment
cnp     = zeros(1,N);   %Spanwise distribution of yawing moment

for i = 1:N
    sum1 = 0;
    sum2 = 0;
    for n = 1:N
        if i == 1; %special case to deal with 0/0 error
           a_i(i) = a_i(i)-n*A(n)*sign(sin(n*pi)); %Questionable...
        else
            a_i(i) = a_i(i)-n*A(n)*sin(n*psi(i))/sin(psi(i));
        end
        clp(i) = clp(i) + A(n)*sin(n*psi(i))*sin(psi(i))*dpsi;
        cmp(i) = cmp(i) + A(n)*sin(n*psi(i))*sin(psi(i))*cos(psi(i))*dpsi;
        sum1   = sum1+A(n)*sin(n*psi(i))*sin(psi(i));
        sum2   = sum2+n*A(n)*sin(n*psi(i));
    end
    cnp(i) = sum1*sum2*cos(psi(i));
end

CL_check = 2*AR*sum(clp);
Cl = AR*sum(cmp);
Cni = AR*sum(cnp);

%%%%%%%%%%%%%%%%%%%%%%%%
%Write and plot results%
%%%%%%%%%%%%%%%%%%%%%%%%
figure()
plot(y/.3048, a_i*180/pi)
title('Downwash angle')
ylabel('Deg')
xlabel('Spanwise location (ft)')
axis([(-b/2-1)/.3048, (b/2+1)/.3048, -30, 30]) 

disp('  ')
disp('Initial Conditions')
disp('==================')
disp(['AoA      = ',num2str(alfa*180/pi),' deg'])
disp(['TAS      = ',num2str(V),' m/s'])
disp(['AR       = ',num2str(AR)])
disp(['Sref     = ',num2str(Sref),' m^2 (',num2str(Sref/.3048^2),' ft^2)'])

disp('  ')
disp('Resulting coefficients')
disp('==================')
disp(['CL (lift coefficient)   = ',num2str(CL)])
disp(['CL (check)              = ',num2str(CL_check)])
disp(['CDi                     = ',num2str(CDi)])
disp(['Cl (roll moment coeff.) = ',num2str(Cl)])
disp(['Cni                     = ',num2str(Cni)])
disp(['cl_section (nominal)    = ',num2str(cl_section)])
disp(['e                        = ',num2str(1/(1+delta))])


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
