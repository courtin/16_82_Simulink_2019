function [CL,CDi,Cl, y] = run_NLL(alfa, dCJ_i,dF_i,V,N, verbose)
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

c_root = 1.5*.3048;
ytip = 1.75*.3048;
c_dist = ones(1,N+1)*c_root; %chord distribution, m
y = linspace(-b/2, b/2, N+1);
%POC wing
c_tip = .9*.3048;
for i = 1:N+1
    if abs(y(i)) > b/2-ytip
        yin = b/2-abs(y(i));
        c_dist(i) = c_tip + c_tip/c_root*yin*.3048 ;
    end
end


S = cumtrapz(y,c_dist);
Sref = S(end);
AR = b^2/Sref;

%cl = get_coeffs_wing(alfa*180/pi, CJs(i), dFs(i).*180/pi,1)
cl = 2*pi*alfa;
c = get_c(0, c_dist, y);

Gam0 = .5*c*cl*V;
Gam = Gam0*sqrt(1-(2*y/b).^2);
plot(y, Gam)
title('Initial \Gamma distribution')
dy = b/N;
a_i = get_a_i(Gam, y,dy,V, N);
%plot(y, a_i)
a_eff = alfa - a_i;
cl = zeros(1,N+1);
c  = zeros(1,N+1);
dCJs  = zeros(1,N+1);
dFs  = zeros(1,N+1);
for i = 1:N+1
    [dCJs(i), dFs(i)] = get_CJ_dF(y(i), dCJ_i, dF_i);
    if dCJs(i) == 0
        cl(i) = 2*pi*alfa;
    else
        cl(i) = get_coeffs_wing(a_eff(i)*180/pi, dCJs(i), dFs(i).*180/pi,1);
    end
    c(i) = get_c(y(i), c_dist, y);
end

Gam_new = .5.*V.*c.*cl;
%plot(y, Gam)

err = 1e6;
max_iter = 100;
iter = 1;
threshold = .0001;
D = .05;
history = zeros(5, N+1);
history(1,:) = Gam_new;
history(2,:) = Gam;
while err > threshold && iter <= max_iter
    Gam_in = Gam+D*(Gam_new-Gam);
    a_i = get_a_i(Gam_in, y, dy, V, N);
    a_eff = alfa - a_i;
    for i = 1:N+1
        if dCJs(i) == 0
            cl(i) = 2*pi*a_eff(i);
        else
            cl(i) = get_coeffs_wing(a_eff(i)*180/pi, dCJs(i), dFs(i).*180/pi,1);
        end
    end
    Gam_new = .5.*V.*c.*cl;
    Gam_new(1) = 0;
    Gam_new(end) = 0;
    for k = 5:-1:2
        history(k,:) = history(k-1,:);
    end
    history(1,:) = Gam_new;
    Gam = Gam_in;
    
    max_err = zeros(1,N+1);
    for j = 1:N+1
        e1 = history(2,j) - history(1,j);
        e2 = history(3,j) - history(2,j);
        e3 = history(4,j) - history(3,j);
        e4 = history(5,j) - history(4,j);
        
        avg_val = mean(history(:,j));
        max_err(j) = max(abs([e1,e2,e3,e4]))/avg_val;
    end
    
    err = max(max_err);
        
    iter = iter + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Force and Moment Calculation%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CL_linear = pi*AR*A(1);
% 
% delta = 0;
% for n = 2:N
%     delta = delta + n*(A(n)/A(1))^2;        
%     
% end

%CDi = CL_linear^2/(pi*AR)*(1+delta);
figure()
draw_c(c_dist, y,b)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate Forces and Moments %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lp = rho*V*Gam;
lMp = rho*V*Gam.*y;
Dip = rho.*Gam.*V.*a_i;

L = sum(Lp)*dy;
lM = sum(lMp)*dy;
Di = sum(Dip)*dy;

CL  = L/(.5*rho*V^2*Sref);
CDi = Di/(.5*rho*V^2*Sref);
Cl  = lM/(.5*rho*V^2*Sref*b);
  
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


disp('  ')
disp('Resulting coefficients')
disp('==================')
disp(['CL (lift coefficient)   = ',num2str(CL)])
disp(['CDi                     = ',num2str(CDi)])
disp(['Cl (roll moment coeff.) = ',num2str(Cl)])

end


%%%%%%%%%%%%%%%%%%
%Helper Functions%
%%%%%%%%%%%%%%%%%%

function c = get_c(y, c_dist, y_dist)
    c = interp1(y_dist, c_dist, y);
end

function [dCJ, dF] = get_CJ_dF(y, dCJ_i, dF_i)
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

function T = get_sum_term(j,n,dGam_dy, y)
    T = dGam_dy(j)/(y(n)-y(j));
end

function a_i = get_a_i(Gam, y,dy,V, N)
    dGam = zeros(1,N+1);
    for i = 1:(N+1)/2
        dGam(i) = Gam(i+1) - Gam(i);
    end

    for i = N+1:-1:(N+1)/2+1
        dGam(i) = Gam(i) - Gam(i-1);
    end
    

    a_i = zeros(1,N+1);
    for n = 1:N+1
        for j = 2:2:N
            if n == j-1
                if n == 1
                    left = 0;
                else
                    left = get_sum_term(j-1,n-1,dGam./dy, y);
                end
                right = get_sum_term(j-1,n+1,dGam./dy, y);
                T1 = (left+right)/2;
            else
                T1 = get_sum_term(j-1,n,dGam./dy, y);
            end
            if n == j
                left = get_sum_term(j,n-1,dGam./dy, y);
                right = get_sum_term(j,n+1,dGam./dy, y);
                T2 = (left+right)/2;
            else
                T2 = get_sum_term(j,n,dGam./dy, y);
            end
            if n == j+1
                left = get_sum_term(j+1,n-1,dGam./dy, y);
                if n == N+1
                    right = 0;
                else
                    right = get_sum_term(j+1,n+1,dGam./dy, y);
                end
                T3 = (left+right)/2;
            else
                T3 = get_sum_term(j+1,n,dGam./dy, y);
            end

            a_i(n) = a_i(n)+1/(4*pi*V)*dy/3*(T1+4*T2+T3);
        end
    end
end
