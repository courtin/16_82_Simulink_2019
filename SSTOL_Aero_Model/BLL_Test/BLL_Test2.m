%Blown Lifting Line Test
clear all
load 40df_fits.mat
b = 12;             %span, m
Sref = 10;          %reference area
a = 15;              %Angle of attack
V = 25;             %Speed, m/s
N = 10;             %Number of spanwise sections (must be even)
rho = 1.225;

c = ones(N,1);      %Chord at each station
y = [-b/2,linspace(-b/2*(1-1/N), b/2*(1-1/N), N), b/2]; %Space stations evenly along the span

dCJ = zeros(N,1);   %Blowing control vector for each spanwise location


%dCJ(2:9) = 6;       %Sections 2-9 have constant blowing
%dCJ = [0;4;4;4;4;4;4;4;4;0];


inputs.b = b;
inputs.N = N;
inputs.c = c;
inputs.y = y;
inputs.dCJ = dCJ;
inputs.cl_fit = cl_fit;
a_i = zeros(N,1);
get_downwash_R(a_i, a, V, inputs)



f = @(a_i) get_downwash_R(a_i, a,V,inputs);

a_i_c = fsolve(f, a_i);

Lp = .5*rho*V^2.*c.*cl_fit(a+a_i_c,dCJ)*b/N;
Mp = Lp'.*y(2:end-1);

L = sum(Lp);
M = sum(Mp);

CL = L/(.5*rho*V^2*Sref);
Cl = M/(.5*rho*V^2*Sref*b);

disp('  ')
disp('Initial Conditions')
disp('==================')
disp(['AoA      = ',num2str(a),' deg'])
disp(['TAS      = ',num2str(V),' m/s'])

disp('  ')
disp('Resulting coefficients')
disp('==================')
disp(['CL (lift coefficient)   = ',num2str(CL)])
disp(['Cl (rolling moment coefficient)   = ',num2str(Cl)])

disp('  ')
disp('Sanity Check')
disp('==================')
disp(['CL (simple approx)   = ',num2str(2*pi*a*.9)])


plot(y(2:end-1), Lp)
hold on
scatter(y(2:end-1), Lp, 'x')
ylabel('Lift')
xlabel('y')

figure()
plot(y(2:end-1), a_i_c)
ylabel('\alpha_i')
xlabel('y')

cls = inputs.cl_fit(a+a_i_c, dCJ);
figure()
plot(y(2:end-1), cls)
ylabel('c_l')
xlabel('y')

