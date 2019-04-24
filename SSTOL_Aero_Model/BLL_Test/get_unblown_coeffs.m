function [cl1, cd1, cm1] = get_unblown_coeffs(a, cl, cd, alphas, cm)
    %Expects a as AOA in degrees
    %cl, cd, alphas all vectors of equal length (alphas in degrees)
    if a > 89
        fprintf(1, 'Clipping AoA of %f to 89\n', a)
        a = 89;
    elseif a < min(alphas)
        a = min(alphas);
    end
    cl1 = interp1(alphas, cl, a);
    cd1 = interp1(alphas, cd, a); 
    cm1 = interp1(alphas, cm, a);
    
    if isnan(cl1)
        fprintf(1, 'Warning: cl is NaN and AoA is %f\n', a)
    end
end