function [CL,CD,CM]=regression_results(alpha, delta, delta_cj,CL_coeffs,CD_coeffs,CM_coeffs)

X=[];
for i=0:size(CL_coeffs,1)-1
    X=[X (delta_cj.^i)];
end

CL=X*CL_coeffs(:,1)+alpha.*(X*CL_coeffs(:,2))+alpha.^3.*(X*CL_coeffs(:,3))+delta.*(X*CL_coeffs(:,4))+delta.^3.*(X*CL_coeffs(:,5));
X=[];
for i=0:size(CD_coeffs,1)-1
    X=[X (delta_cj.^i)];
end
CD=X*CD_coeffs(:,1)+alpha.*(X*CD_coeffs(:,2))+alpha.^2.*(X*CD_coeffs(:,3))+delta.*(X*CD_coeffs(:,4))+delta.^2.*(X*CD_coeffs(:,5));

X=[];
for i=0:size(CM_coeffs,1)-1
    X=[X (delta_cj.^i)];
end
CM=X*CM_coeffs(:,1)+alpha.*(X*CM_coeffs(:,2))+alpha.^3.*(X*CM_coeffs(:,3))+delta.*(X*CM_coeffs(:,4))+delta.^3.*(X*CM_coeffs(:,5));
end
