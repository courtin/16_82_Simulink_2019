function [R] = get_downwash_R(a_i, alphad, V, inputs)

b = inputs.b;               %span, m
N = inputs.N;               %Number of control points
c = inputs.c;               %Chord at each station
y = inputs.y;               %Spanwise locations
dCJ = inputs.dCJ;           %Blowing control vector
Gam = zeros(N,1);           %Circulation at each station

w_i = zeros(N,1);   %Downwash velocity at each station

%cls = inputs.cl_fit(alphad+a_i, dCJ);
cls = get_cl(alphad+a_i);
Gam = .5*V.*c.*cls;

Gam(1) = 0;     %Circulation goes to zero at the wing tips
Gam(end) = 0;

gam  = zeros(N-1,1);   %Wake circulation at each station   
ybar = zeros(N-1,1);   %effective location of wake circulation   

for i = 1:N-1
    dGam = Gam(i+1)-Gam(i);
    dy = y(i+1)-y(i);
    %dy = 1;
    gam(i) = -dGam/dy;
    %ybar(i) = .5*(y(i+1)+y(i));
    ybar(i) = (y(i));
end

for i=1:N
   for j = 2:N-1
       %Put the wake vorticity in the middle of the two control points
       dy = y(j+1)-y(j);
       w_i(i) = w_i(i) + gam(j)*dy/(4*pi*(y(i)-ybar(j)));
   end
end

a_i_n = w_i/V;
R = a_i - a_i_n;
