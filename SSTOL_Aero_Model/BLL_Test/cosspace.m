function [x_cos] = cosspace(p, N)
%COSSPACE return a cosine spacing array of N points on the interval a b,
%which are column vectors
%p (points) is an array of nodes along which the points should be spaced, with
%structure as shown below.
%
%   x1 x2 ... xn
%   y1 y2 ... yn
%   z1 z2 ... zn
%
theta = linspace(0, pi, N);

%Find total path length
L = size(p,2);
breaks = zeros(2,L); %first row is theta coordinate of break point, second row is 
%path length from the previous breakpoint to that point.  First point is
%assumed to be pi/2. A breakpoint is a point where the path direction
%changes.
x_cos = zeros(3, N);
s_tot = 0;
for i = 1:L-1   %calculate total path length
    s = norm(p(:,i+1)-p(:,i));
    s_tot = s_tot + s;
    breaks(2,i+1) = s;
end
breaks(1,1) = 0;
for i = 2:L     %Find the theta coordinate of each break point. 
    
    breaks(1,i) =   acos(-sum(breaks(2,1:i))/s_tot);
end

for t = 1:length(theta)     %Project each theta point onto the appropriate line segment
    i = 2;
    while theta(t) > breaks(1,i) && i < L
        i = i+1;
    end
    seg = p(:, i)-p(:,i-1);
    sx = -cos(theta(t));
    s0 = -cos(breaks(1,i-1));
    s1 = -cos(breaks(1,i));
    x_cos(:,t) = seg.*((sx-s0)/(s1-s0))+ p(:,i-1);
end
end



