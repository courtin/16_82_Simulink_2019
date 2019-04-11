function [Di, ai] = trefftz(Gam, vortex, ra, rb, verbose)
N = length(Gam);
theta = zeros(1,N);
del_phi = zeros(3,N);
dphi_dn = zeros(1,N);
ns = zeros(3,N);
ds = zeros(1,N);
lines = [ra, rb(:,end)];
Gam_im1 = zeros(1,N+1);
for i = 1:N
    theta(i) = getTheta(ra(:,i), rb(:,i));
    ns(:,i) = [0;sin(theta(i));cos(theta(i))];
    ds(i) = norm(lines(:,i+1)-lines(:,i));
end

for i = 1:N+1
    if i == 1
        Gam_im1(i) = -Gam(1);
    elseif i == N+1
        Gam_im1(i) =Gam(end) ;
    else
        Gam_im1(i) = Gam(i-1)-Gam(i);
    end
end

for i =1:N
    for j = 1:N+1
        num = cross([1;0;0], vortex(:,i)-lines(:,j));
        den = norm(vortex(:,i)-lines(:,j))^2;
        del_phi(:,i) = del_phi(:,i) + 1/(2*pi)*Gam_im1(j)*num/den;
    end
    dphi_dn(i) = dot(del_phi(:,i), ns(:,i));
end

Di = 0;
for i = 1:N
    Di = Di-.5*Gam(i)*dphi_dn(i)*ds(i);
end
if verbose
    figure()
    plot(lines(2,:), Gam_im1)
end
%ai = .5*atan(del_phi(3,:)/del_phi(1,:));
ai = .5*del_phi(3,:);
end



