%Unblown TAT Lifting Line Test, via convergence on a_i input vs calculated
clear all

b = 12;             %span, m
Sref = 12;          %reference area
a = 5 * pi/180;              %Angle of attack
V = 25;             %Speed, m/s
N = 25;             %Number of spanwise sections (must be even)
rho = 1.225;

c = ones(N,1);      %Chord at each station
y = linspace(-b/2, b/2, N); %Space stations evenly along the span
%y =  (-b/2+b/2)/2 + (b/2+b/2)/2*cos(pi/N*((1:N)-.5)); 
%y =  [-b/2,-(b)/2*cos(pi/N*((1:(N-2))+.5)), b/2]; 
dCJ = zeros(N,1);   %Blowing control vector for each spanwise location

%dCJ(2:9) = 6;       %Sections 2-9 have constant blowing
%dCJ = [0;4;4;4;4;4;4;4;4;0];

inputs.b = b;
inputs.N = N;
inputs.c = c;
inputs.y = y;
inputs.dCJ = dCJ;
%inputs.cl_fit = cl_fit;
a_i = zeros(N,1)-2*pi/180;

f = @(a_i) get_downwash_R(a_i, a,V,inputs);
options = optimoptions('fsolve','MaxFunctionEvaluations', 5000);
a_i_c = fsolve(f, a_i, options);
cl = get_cl(a+a_i_c);
R = get_downwash_R(a_i_c, a,V,inputs);
%cl(1) = 0;
%cl(end) = 0;
Lp = zeros(N-1,1);
Dp = zeros(N-1,1);
Mp = zeros(N-1,1);
y_avg = zeros(N-1,1);
for j = 1:N-1
    c_avg = (c(j+1)+c(j))/2;
    cl_avg = (cl(j+1)+cl(j))/2;
    
    dy = (y(j+1)-y(j));
    
    Lp(j) = .5*rho*V^2*c_avg*cl_avg*dy;
    
    a_i = (a_i_c(j+1)+a_i_c(j))/2;
    Dp(j) = Lp(j)*-a_i;
    
    y_avg(j) = (y(j+1)+y(j))/2;
    Mp(j) = Lp(j)*y_avg(j);
end

%Gam = V.*c.*cl*.5;
%Lp = Gam.*V.*rho
%Lp(1) = 0;
%Lp(end) = 0;
%Mp = Lp'.*y;

L = sum(Lp);
M = sum(Mp);
Di = sum(Dp);

CL = L/(.5*rho*V^2*Sref);
Cl = M/(.5*rho*V^2*Sref*b);
CDi = Di/(.5*rho*V^2*Sref);

disp('  ')
disp('Initial Conditions')
disp('==================')
disp(['AoA      = ',num2str(a*180/pi),' deg'])
disp(['TAS      = ',num2str(V),' m/s'])
disp(['AR       = ',num2str(b^2/Sref)])

disp('  ')
disp('Resulting coefficients')
disp('==================')
disp(['CL (lift coefficient)   = ',num2str(CL)])
disp(['CDi                     = ',num2str(CDi)])
disp(['Cl (rolling moment coefficient)   = ',num2str(Cl)])

disp('  ')
disp('Sanity Check')
disp('==================')
AR = b^2/Sref;
CLa = 2*pi/(1+2*pi/(pi*AR));
disp(['CL (simple approx)   = ',num2str(a*CLa)])

plot(y_avg, Lp)
hold on
scatter(y_avg, Lp, 'x')
ylabel('Lift')
xlabel('y')

figure()
plot(y, a_i_c.*180/pi)
ylabel('\alpha_i')
xlabel('y')

figure()
plot(y, cl)
ylabel('c_l')
xlabel('y')
