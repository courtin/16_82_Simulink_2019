%Test WVL Method using wind tunnel polar lookup
clear all;
close all;
clc;

%Set up geometry
b = 13;
N = 80;
root_LE = [0;0;0];
break_LE = [0;5;0];
tip_LE = [.42; 6.5; 0];
c_root = 1.5;
c_tip = .92;

Sref = 17.95;

%Flight condition
alpha = 5*pi/180;
beta = 0;
rho = 1.225;
Vinf = 10;
Ubar = [-(cos(alpha)*cos(beta));sin(beta);-sin(alpha)*cos(beta)];

%Create vortex and control points for one half wing
root_vortex = root_LE + [c_root/4;0;0];
break_vortex = break_LE + [c_root/4;0;0];
tip_vortex = tip_LE + [c_tip/4;0;0];

lines  = half_cosspace([root_vortex, break_vortex, tip_vortex], N+1);
ra = lines(:,1:N);
rb = lines(:,2:N+1);
[vortex, dvortex] = midpoints(lines);
control = vortex;
cs= zeros(1,N);

%Locate the control points
for i = 1:N
    y = vortex(2,i);
    yt = tip_LE(2);
    yb = break_LE(2);
    if abs(y) <= yb
        c = c_root;
    else
        c = c_root - (c_root-c_tip)*(y-yb)/(yt-yb);
    end
    cs(i) = c;
    control(1,i) = control(1,i) + c/2;
end
    
%Add other half of wing, symmetric around y-axis

vortex_sym = vortex;
control_sym = control;
ra_sym = ra;
rb_sym = rb;
ra0 = ra;
vortex_sym(2,:) = vortex_sym(2,:)*-1;
control_sym(2,:) = control_sym(2,:)*-1;
ra_sym(2,:) = ra_sym(2,:)*-1;
rb_sym(2,:) = rb_sym(2,:)*-1;
vortex = [fliplr(vortex_sym), vortex];
control = [fliplr(control_sym), control];
rb = [fliplr(ra_sym), rb];
ra = [fliplr(rb_sym), ra];

dvortex = [fliplr(dvortex), dvortex];

plot3([root_LE(1),break_LE(1), tip_LE(1)],[root_LE(2),break_LE(2), tip_LE(2)],[root_LE(3),break_LE(3), tip_LE(3)])
hold on
scatter3(vortex(1,:),vortex(2,:),vortex(3,:))
scatter3(control(1,:),control(2,:),control(3,:))
%axis equal
title("Wing bound-leg and control point locations")

A = zeros(2*N,2*N);
n = zeros(3,2*N);
for i = 1:2*N
    for j = 1:2*N
            n(:,i) = getNormal(vortex(:,i)-control(:,i), ra(:,i)-rb(:,i));
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
ai = zeros(1,2*N);

for i = 1:2*N
    for j = 1:2*N
        ri = .5*(ra(:,i)+rb(:,i));
        Vbar(:,i) = Vbar(:,i) + vorvel2(ri, ra(:,j), rb(:,j))*Gamb(j);
    end
    Vbar(:,i) = Vbar(:,i)-Ubar;
end

%Solve for force and downwash distribution
V = Vbar*Vinf;
for i = 1:2*N
    l = (rb(:,i)-ra(:,i));
    Fi(:,i) = rho*Vinf*Gamb(i)*cross(V(:,i), l);
    ai(i) = -Vbar(3,i)/Vinf;
end

Fbari = Fi./(.5*rho*Vinf^2*Sref);

Fbar = zeros(3,1);

Fbar(1) = sum(Fbari(1,:));
Fbar(2) = sum(Fbari(2,:));
Fbar(3) = sum(Fbari(3,:));

%Caluculate coefficients
T = [cos(alpha),0,sin(alpha);0,1,0;-sin(alpha),0,cos(alpha)];

Coeffs = T*Fbar;
Cdi = Coeffs(1);
CY = Coeffs(2);
CL = Coeffs(3);

%Plot results
figure()
plot(vortex(2,:), Gamb)
title("Circulation")
ylabel("\Gamma / V_\infty")
xlabel("Spanwise location (ft)")

figure()
plot(vortex(2,:), ai)
title("Downwash")
ylabel("\alpha_i")
xlabel("Spanwise location (ft)")


