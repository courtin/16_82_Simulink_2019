function [x,u,trimmed_inputs] = calculate_trimmed_state(inputs0,x0, u0, airplane)

% inputs is a 3 element vector:
% (1) alpha in degrees
% (2) elevator angle in degrees
% (3) blowing
% (4) thrust

%calculates trim around the 'calculate body force' function 

norm_body_force = calculate_body_force(inputs0,airplane);

myfunc = @(inputs) calculate_body_force(inputs,airplane);
%don't change power

min_inputs = [-50, -30,0,0];
max_inputs = [50, 30,0,1];

trimmed_inputs = fmincon(myfunc,inputs0,[],[],[],[],min_inputs,max_inputs);

[~, x, u] = calculate_body_force(trimmed_inputs,airplane)


