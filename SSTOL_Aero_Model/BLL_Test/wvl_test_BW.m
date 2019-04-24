%Test WVL Method 
clear all;
close all;
clc;

useTAT = 0;
%Set up geometry
b = 13;
N1 = 10;    %Inboard section
N2 = 5;     %Outboard section
root_LE = [0;0;0];
break_LE = [0;5;0];
tip_LE = [.42; 6.5; 0];
%tip_LE = [.125; 6.5;0];
%tip_LE = [0.; 6.5; 0];
c_root = 1.5;
c_tip = .92;
%c_tip = 1.5;

Sref = 17.95;

%Load unblown airfoil polar
load bw02b_polar.mat cl cd alpha cm;
cl_unblown = cl;
cd_unblown = cd;
alpha_unblown = alpha;
cm_unblown = cm;

%Flight condition
alpha = 15*pi/180;
beta = 0;
rho = 1;
Vinf = 1;
Ubar = [-(cos(alpha)*cos(beta));sin(beta);-sin(alpha)*cos(beta)];
dCJ = 4.5;
dF = 40*pi/180;


if useTAT
    %TAT_model
    cla_b = 2*pi;
    cla_ub = 2*pi;
    a0_b = 0;
    a0_ub = 0;
    cl_nom_b = 2*pi*alpha;
else
    [cla_b,cla_ub,a0_b,a0_ub,cl_nom_b] = POC_lin(alpha,dCJ, dF, 1); %POC aero model
end




%Create vortex and control points for one half wing
root_vortex = root_LE + [c_root/4;0;0];
break_vortex = break_LE + [c_root/4;0;0];
tip_vortex = tip_LE + [c_tip/4;0;0];



N = N1+N2;
%N = N1+N2+1;
lines1  = cosspace([root_vortex, break_vortex], N1+1);
lines2  = cosspace([break_vortex, tip_vortex], N2+1);

%lines = [lines1, lines2];
lines = [lines1(:,1:end-1), lines2];

%lines = half_cosspace([root_vortex, tip_vortex], N+1);
ra = lines(:,1:N);
rb = lines(:,2:N+1);

[vortex, dvortex] = midpoints(lines);
control = vortex;

cs= zeros(1,N);
dCJs = zeros(1,N);
dFs = zeros(1,N);
clas = zeros(1,N);
a0s = zeros(1,N);

%Locate the control points
for i = 1:N
    %Calculate the chord distribution (hard-coded for wing with a single
    %break (POC vehicle).
    y = vortex(2,i);
    yt = tip_LE(2);
    yb = break_LE(2);
    if abs(y) <= yb
        c = c_root;
        dCJl = dCJ; %Constant blowing across inboard portion
        dFl = dF;
        cla_l = cla_b;
        a0_l = a0_b;
    else
        c = c_root - (c_root-c_tip)*(y-yb)/(yt-yb);
        dCJl = 0; %No blowing at tips
        dFl = 0;
        cla_l = cla_ub;
        a0_l = a0_ub;
    end
    
    cs(i) = c;
    dCJs(i) = dCJl;
    dFs(i) = dFl;
    clas(i) = cla_l;
    a0s(i) = a0_l;
    
    control(1,i) = control(1,i) + cla_l/(2*pi)*c/2;
end
% 
% clines1  = cosspace([root_vortex + [c_root/2;0;0], break_vortex+ [c_root/2;0;0]], N1+1);
% clines2  = cosspace([break_vortex+ [c_root/2;0;0], tip_vortex+ [c_tip/2;0;0]], N2+1);
% 
% %lines = [lines1, lines2];
% clines = [clines1(:,1:end-1), clines2];
% [control, ~] = midpoints(clines);

%Add other half of wing, symmetric around y-axis
ra_sym = ra;
rb_sym = rb;
ra_sym(2,:) = ra_sym(2,:)*-1;
rb_sym(2,:) = rb_sym(2,:)*-1;
rb = [fliplr(ra_sym), rb];
ra = [fliplr(rb_sym), ra];

vortex = make_sym(vortex,2);
control = make_sym(control,2);
a0s = make_sym(a0s);
dCJs = make_sym(dCJs);
cs = make_sym(cs);
clas = make_sym(clas);

dvortex = [fliplr(dvortex), dvortex];

%draw_wing(root_LE, break_LE, tip_LE, c_tip, c_root, vortex, control,cs)

A = zeros(2*N,2*N);
n = zeros(3,2*N);
for i = 1:2*N
    for j = 1:2*N
            panel_n = getNormal(vortex(:,i)-control(:,i), ra(:,i)-rb(:,i));
            a_aero = -a0s(i);
            Raero = [cos(a_aero),0,sin(a_aero);0,1,0;-sin(a_aero),0,cos(a_aero)];
            n(:,i) = Raero*panel_n;
            Vh = vorvel(control(:,i), ra(:,j), rb(:,j));
            A(i,j) = dot(Vh,n(:,i));
    end
end

Ainv = inv(A);

r= zeros(2*N,1);

xh = [1;0;0];
yh = [0;1;0];
zh = [0;0;1];

%Set up RHS of solution matrix
for k = 1:2*N
    r(k) = Ubar(1)*dot(xh,n(:,k)) ...
            + Ubar(2)*dot(yh,n(:,k)) ...
            + Ubar(3)*dot(zh,n(:,k));
end

%Solve for circulation
Gamb = Ainv*r;

%Sanity check lift coefficient
vy = Ubar(1).*-dvortex(2,:);
vy = Ubar(1).*-(rb(2,:)-ra(2,:));
CL1 = 2*Gamb'*(vy')/Sref;

%Calculate velocity distribution
Vbar = zeros(3, 2*N);
Fi = zeros(3, 2*N);
ls = zeros(3, 2*N);
ai = zeros(1,2*N);
a_eff = zeros(1,2*N);

for i = 1:2*N
    for j = 1:2*N
        LBOUND = ~(i==j);% || i == j+1 || i == j-1);
        ri = .5*(ra(:,i)+rb(:,i));
        Vh = vorvel2(ri, ra(:,j), rb(:,j),LBOUND);
        Vbar(:,i) = Vbar(:,i) + Vh*Gamb(j);
    end
    Vbar(:,i) = Vbar(:,i)-Ubar;
    a_eff(i) = Vbar(3,i)/Vbar(1,i);
    ai(i) = -(alpha - a_eff(i));
end

T = [cos(alpha),0,sin(alpha);0,1,0;-sin(alpha),0,cos(alpha)];

%Solve for force and downwash distribution
V = Vbar*Vinf;
for i = 1:2*N
    l = (rb(:,i)-ra(:,i));
    %l(1) = 0;
    %l(3) = 0;
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

%a_eff = alpha+ai;
dy = rb(2,:)-ra(2,:);

cbar1 = Sref/b;
cbar = sum(cs.*dy)/b;
Scalc = sum(cs.*dy);

% figure()
% plot(vortex(2,:), ai2)
% hold on
% %plot(vortex(2,:), a_eff)
% title("Downwash Check")
% ylabel("\alpha_i")
% xlabel("Spanwise location (ft)")

cl_t = zeros(2*N,1);
cx_t = zeros(2*N,1);
cm_t = zeros(2*N,1);
cls = zeros(2*N,1);
cxs = zeros(2*N,1);
cms = zeros(2*N,1);

[Di, ai_T] = trefftz(Gamb, vortex, ra, rb, 1);
a_eff = alpha + ai_T;
if useTAT
    %TaT mode
    cl_t = 2*pi.*a_eff';
    cx_t = .0;
    cm_t = -.1;
else
    for i = 1:2*N
        if dCJs(i) == 0
            %Use lookup for bw02b airfoil
            [cl_t(i),cx_t(i)] = get_unblown_coeffs(a_eff(i)*180/pi, cl_unblown, cd_unblown, alpha_unblown, cm_unblown);
            cm_t(i) = -.1;%Placeholder cm
        else
            %Use wind tunnel data
            [cl_t(i),cx_t(i),cm_t(i)] = get_coeffs_wing(a_eff(i)*180/pi,dCJs(i),dF*180/pi,1);
        end
    end
end

cls = cl_t.*cos(-ai_T') - cx_t.*sin(-ai_T');
t1 = cl_t.*sin(-ai_T');
t2 = cx_t.*cos(-ai_T');
cxs = cl_t.*sin(-ai_T') + cx_t.*cos(-ai_T');


%Plot results
fc = figure();
plot(vortex(2,:), Gamb*2/(Vinf*cbar), 'x-')
hold on 
plot(vortex(2,:), Gamb*2/(Vinf)./cs')
plot(vortex(2,:), cls)
title("Circulation")
xlabel("Spanwise location (ft)")
legend("2\Gamma / (c_{ref} V_\infty)", "c_l (from \Gamma)", "c_l (from \alpha_{eff})")
saveas(fc, "circ.pdf")
fd = figure();
plot(vortex(2,:), ai, 'x-')
hold on
plot(vortex(2,:), ai_T)
title("Downwash")
ylabel("\alpha_i")
xlabel("Spanwise location (ft)")
legend("Near-field", "Trefftz Plane")
saveas(fd,"Downwash.pdf")


% figure()
% plot(vortex(2,:), cls)
% hold on
% plot(vortex(2,:), cxs)
% plot([vortex(2,1), vortex(2,end)], [cl_nom_b, cl_nom_b], 'k--')


%figureA
%plot(vortex(2,:), cls.*cs'./cbar)
CLp = cls.*cs'/cbar;
CL2 = mean(CLp);

CL2 = trapz(vortex(2,:),cls.*cs')/Sref;
CXp  = trapz(vortex(2,:),cxs.*cs')/Sref;
CX = CXp;
AR = b^2/Sref;
CDi2 = CL^2/(pi*AR);

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
disp(['CY                      = ',num2str(CY)])
%disp(['Cl (roll moment coeff.) = ',num2str(Cl)])
%disp(['Cni                     = ',num2str(Cni)])
%disp(['cl_section (nominal)    = ',num2str(cl_section)])
%disp(['e                        = ',num2str(e)])

function [sym_array] = make_sym(array, ax)
%If array is 3xN, flip the sign of the row specified by ax
%i.e. ax = 2 specifies y-symmetry
%If array is 1xN, assume no sign change
s =size(array);
array_sym = array;
    if s(1) == 3
        array_sym(ax,:) = array_sym(ax,:)*-1;
        sym_array = [fliplr(array_sym), array];
    else
        sym_array = [fliplr(array_sym), array];
    end
end