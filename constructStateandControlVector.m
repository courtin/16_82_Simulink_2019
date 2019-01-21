function [x,u]=constructStateandControlVector(alpha,beta,dA, dE, dR,dF_L,dF_R, dB_L,dB_R, dT_L,dT_R, hdot, p,q,r, phi,theta, psi, xe,ye,ze,V,soundSpeed)

Mach	= 	V / soundSpeed;	
gamma	=	57.2957795 * atan(hdot / sqrt(V^2 - hdot^2));

phir	=	phi * 0.01745329;
thetar	=	theta * 0.01745329;
psir	=	psi * 0.01745329;

alphar	=	alpha * 0.01745329;
betar	=	beta * 0.01745329;

x	=	[V * cos(alphar) * cos(betar)
        V * sin(alphar) * cos(betar)
        q * 0.01745329
        thetar
        V * sin(betar)
        p * 0.01745329
        r * 0.01745329
        phir
        psir
        xe
        ye
        ze];

u	=	[dE * 0.01745329
        dA * 0.01745329
        dR * 0.01745329
        dF_L * 0.01745329
        dF_R * 0.01745329
        dB_L
        dB_R
        dT_L
        dT_R];
   