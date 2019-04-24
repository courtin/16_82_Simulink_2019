function [V] = vorvel3(r, ra, rb, LBOUND)
%Copy of fortran vorvel implementation

b = ra-r;
a = rb-r;

%a = [0;a(2);0];
%b = [0;b(2);0];
%scatter3(ra(1),ra(2), ra(3))
%scatter3(rb(1),rb(2), rb(3))

xh = [1;0;0];

%na = norm(a);
%nb = norm(b);

na = sqrt(a(1)^2+a(2)^2+a(3)^2);
nb = sqrt(b(1)^2+b(2)^2+b(3)^2);

V = zeros(3,1);
if LBOUND && na*nb ~= 0
    axb(1,1) = a(2)*b(3)-a(3)*b(2);
    axb(2,1) = a(3)*b(1)-a(1)*b(3);
    axb(3,1) = a(1)*b(2)-a(2)*b(1);
    
    adb = a(1)*b(1)+a(2)*b(2)+a(3)*b(3);
    
    den = na*nb+adb;
    T = (1/na+1/nb)/den;
    V(1) = axb(1)*T;
    V(2) = axb(2)*T;
    V(3) = axb(3)*T;
end
if na ~= 0
   axisq = a(3)^2+a(2)^2;
   adi = a(1);
   rsq = axisq;
   
   T = -(1-adi/na)/rsq;
   
   V(2) = V(2) + a(3)*T;
   V(3) = V(3) - a(2)*T;
end

if nb ~= 0
   bxisq = b(3)^2+b(2)^2;
   bdi = b(1);
   rsq = bxisq;
   
   T = (1-bdi/nb)/rsq;
   
   V(2) = V(2) + b(3)*T;
   V(3) = V(3) - b(2)*T;
end
V = 1/(4*pi)*V;
end

