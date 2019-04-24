%Blown Lifting Line Test
clear all
load 40df_fits.mat
b = 10;             %span, m
a = 5;              %Angle of attack
V = 25;             %Speed, m/s
N = 10;             %Number of spanwise sections (must be even)

c = ones(N,1);      %Chord at each station
y = [-b/2,linspace(-b/2*(1-1/N), b/2*(1-1/N), N), b/2]; %Space stations evenly along the span

Gam = zeros(N+2,1);   %Circulation at each station 

load("a_i.mat")
%a_i = zeros(N,1);   %Downwash angle at each station

w_i = zeros(N+2,1);   %Downwash velocity at each station

dCJ = zeros(N,1);   %Blowing control vector for each spanwise location

dCJ(2:9) = 0;       %Sections 2-9 have constant blowing


Gam(2:end-1)     = V.*c.*2*pi.*(a+a_i)*pi/180%cl_fit(a+a_i, dCJ);

gam = zeros(N+2,1);   %Wake circulation at each station    

for i = 1:(N/2)-1
    dGam = Gam(i+1)-Gam(i);
    dy = y(i+1)-y(i);
    %dy = 1;
    gam(i) = -dGam/dy;
end
for i = N+2:-1:(N/2)+1
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
plot(y(2:end-1), a_i)
hold on
plot(y(2:end-1), a_i_n)
R = a_i - a_i_n
a_i = a_i_n;
save("a_i.mat", "a_i")
plot(y(2:end-1),a_i)




