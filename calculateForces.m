function [fx,fy,fz,mx,my,mz] = calculateForces(x,u)


A = load('Airplane.mat');
airplane = A.airplane;

rho = 1.225;

S = airplane.geometry.Wing.S;
cbar = airplane.geometry.Wing.cbar;

U = x(1);
V = x(5);
W = x(2);

roll = x(8);
pitch = x(4);
yaw = x(9);

[alphar, betar] = alphabeta([U V W]);

Vinf = norm([U,V,W]);

Mach = Vinf/330;

[CX,CL,CY,Cl,Cm,Cn]	=	AeroModelSSTOL(x,u,Mach,alphar,betar,Vinf);


f = [CX;CY;CL]*(0.5*rho*Vinf^2*S);
m = [Cl;Cm;Cn]*(0.5*rho*Vinf^2*S*cbar);


mx = m(1);
my = m(2);
mz = m(3);

dcm = angle2dcm(yaw,pitch,roll);

mg = dcm*[0;0;(airplane.weights.MTOW)];

f = f + mg;

fx = f(1);
fy = f(2);
fz = f(3);


