%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute the normal unit vector between two points V1 and V2
%2D only
function theta = getTheta(V1, V2)
    %Tangent vector
    dy = (V2(2)-V1(2));
    dz = (V2(3)-V1(3));
    %Tangent normal vector
    theta = atan(dz/dy);
end