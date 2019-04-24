function [V] = vorvel2(r, ra, rb, LBOUND)
%Same as vorvel but with 1st term left out
%x = ra(1)/2+rb(1)/2;
%ra(1) = x;
%rb(1) = x;
b = ra-r;
a = rb-r;


%a = [0;a(2);0];
%b = [0;b(2);0];
%scatter3(ra(1),ra(2), ra(3))
%scatter3(rb(1),rb(2), rb(3))

xh = [1;0;0];

na = norm(a);
nb = norm(b);

if LBOUND == 0
    T1 = 0;
else
    T1 = cross(a,b)/(na*nb+dot(a,b))*(1/na+1/nb);
end
T2 = cross(a,xh)/(na-dot(a,xh))*(1/na);
T3 = cross(b,xh)/(nb-dot(b,xh))*(1/nb);

V = 1/(4*pi)*(T1 + T2 - T3);
end

