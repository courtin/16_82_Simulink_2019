

%get forces:

function f = getforces(alpha, dE, dB, theta, V)

beta = 0;
dA = 0;
dR = 0;
dF_L = 40;
dF_R = 40;

dB_L = dB;
dB_R = dB;

dT_L = 0;
dT_R = 0;

hdot = 0;

p = 0;
q = 0;
r = 0;

phi = 0;

psi =0;

xe = 0;
ye = 0;
ze = 1000;

soundSpeed = 330;


[x,u]= constructStateandControlVector(alpha,beta,dA, dE, dR,dF_L,dF_R, dB_L,dB_R, dT_L,dT_R, hdot, p,q,r, phi,theta, psi, xe,ye,ze,V,soundSpeed);

[fx,fy,fz] = calculateForces(x,u);

f = [fx,fy,fz];