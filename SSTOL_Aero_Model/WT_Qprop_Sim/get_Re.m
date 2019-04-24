%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   RE CALCULATOR
%
%   This estimates the Reynolds number for a given 
%   flight condition and characteristic length. 

function Re = get_Re(l, alt, M, airplane)
    [~, ~, rho, a, mu] = int_std_atm(alt, airplane);
    V = M*a;
    Re = l*V*rho/mu;
    
end
