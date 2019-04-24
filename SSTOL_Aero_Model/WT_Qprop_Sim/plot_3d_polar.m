function plot_3d_polar(cases)

T = length(cases);
figure(1);
hold on;
xlabel('\alpha')
ylabel('C_L')

figure(2);
hold on;
xlabel('\alpha')
ylabel('C_X')

figure(3);
hold on;
xlabel('\alpha')
ylabel('C_{Mw}')

figure(4);
hold on;
xlabel('\alpha')
ylabel('\delta_e')

CLt_max = 1.5;
Sh = 639;
S = 2500;
lw = .16*12;
lt = 6.5*12;
cref = 1.41*12;
de_max = 25;
de_min = -25;
for i = 1:T
    filename = ["runs/"+cases(i)+".mat"]
    [alpha, CLs, CXs, CMs, throttles, flaps, V, ai_t, CL_tb] = read_case(filename);
    A = length(alpha);
    CM_max = zeros(1,A);
    de_req = zeros(1,A);
    
    for j = 1:A
        cosa = cos(alpha(j)*pi/180);
        sina = sin(alpha(j)*pi/180);
        i_tail = 5*pi/180;
        M_net_w = CMs(j) + (CLs(j)*cosa+CXs(j)*sina)*lw/cref;
        a_tail = alpha(j)*pi/180 + ai_t(j)+i_tail;
        CL_tail_r = M_net_w/(Sh/S*lt/cref*cos(a_tail));
        CL_de = .871*S/Sh/2;   %per radian
        ARh = 4.6;
        %CLat = 2*pi*(ARh/(2+ARh));
        
        de_req(j) = (CL_tail_r - CL_tb(j))/CL_de;
    end
        
    
    a0 = interp1(CXs, alpha, 0);
    a_max = interp1(de_req, alpha, de_max*pi/180);
    a_min = interp1(de_req, alpha, de_min*pi/180);
    CL_lim = interp1(alpha, CLs, a0);
    CL_min = interp1(alpha, CLs, a_min);
    CL_max = interp1(alpha, CLs, a_max);
    figure(1)
    plot(alpha, CLs)
    plot([a0, a0], [0, CL_lim], '--b')
    plot([a_min, a_min], [0, CL_min], '--r')
    plot([a_max, a_max], [0, CL_max], '--r')
    
    figure(2)
    plot(alpha, CXs)
    
    figure(3)
    plot(alpha, CMs)
    plot(alpha, CM_max, '--')
    
    figure(4)
    plot(alpha, de_req*180/pi)
end

figure(1)
legend(cases)
figure(2)
legend(cases)
figure(3)
legend(cases)
figure(4)
legend(cases)

max_de = ones(1, length(alpha))*de_max;
min_de = ones(1, length(alpha))*de_min;
plot(alpha, max_de, 'k--')
plot(alpha, min_de, 'k--')
end
function [alpha, CLs, CXs, CMs, throttles, flaps, V, ai_t, CL_tb] = read_case(filename)

    load(filename, 'alpha', 'CLs', 'CXs', 'CMs', 'throttles','CL_tb','ai_t', 'flaps', 'V')
end