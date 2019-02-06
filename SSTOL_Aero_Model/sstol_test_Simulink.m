%%%%%%%%%%%%%%%%%%%%
%AeroModelSSTOL Test
%%%%%%%%%%%%%%%%%%%%

inputs0 = [8, -3, 0.2]; %assumes laterally symmetric

[norm_body_force, x, u] = calculate_body_force(inputs0,airplane) %calculates body force and the state vector

% if 1 find trim state
if 1
    [x,u, trimmed_inputs] = calculate_trimmed_state(inputs0,x, u, airplane);
end

[norm_body_force, x, u] = calculate_body_force(trimmed_inputs,airplane);

disp('Initial inputs')
disp(inputs0')
disp('Trimed Conditions')
disp(trimmed_inputs')
  