%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute the normal unit vector between 2 3D vectors
%2D only
function V_n = getNormal(V1, V2)
    V = cross(-V1, -V2);
    V_n = V/norm(V);
end