function V = V_Gam(r, rp, G)
    z = r(2);
    zp= rp(2);
    y = r(1);
    yp= rp(1);
    V(1) = G/(2*pi)*(z-zp)/((y-yp)^2+(z-zp)^2);
    V(2) = G/(2*pi)*-(y-yp)/((y-yp)^2+(z-zp)^2);
end