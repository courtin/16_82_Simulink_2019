function [zeta, omega_n] = get_damping_freq(lambda)
%For a given eigenvalue lambda, return the damping ratio and natural
%frequency
mu = real(lambda);
nu = imag(lambda);

omega_n = sqrt(mu^2 + nu^2);
zeta    = -mu/sqrt(mu^2 + nu^2);
end

