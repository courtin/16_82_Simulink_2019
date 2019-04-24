function [R] = get_downwash_R(a_i, alphad, V, inputs)
b = inputs.b;             %span, m
N = inputs.N;             %Number of spanwise sections (must be even)
c = inputs.c;      %Chord at each station
y = inputs.y;       %Spanwise locations
dCJ = inputs.dCJ;   %Blowing control vector
Gam = zeros(N+2,1);   %Circulation at each station 

w_i = zeros(N+2,1);   %Downwash velocity at each station

cls = inputs.cl_fit(alphad+a_i, dCJ);
Gam(2:end-1)     = V.*c.*cls;

gam = zeros(N+2,1);   %Wake circulation at each station    

for i = 1:(N/2)+1
    dGam = Gam(i+1)-Gam(i);
    dy = y(i+1)-y(i);
    %dy = 1;
    gam(i) = -dGam/dy;
end
for i = N+2:-1:(N/2)+2
    dGam = Gam(i)-Gam(i-1);
    dy = y(i)-y(i-1);
    %dy = 1;
    gam(i) = -dGam/dy;
end

for i=1:N+2
   for j = 1:N+2
       if i ~= j
        w_i(i) = w_i(i) + gam(j)/(4*pi*(y(i)-y(j)));
       end
   end
end

a_i_n = w_i(2:end-1)/V*180/pi;
R = a_i - a_i_n;
