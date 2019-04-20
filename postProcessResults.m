%% this script runs at the end of every simulink run


% control inputs plot
control_plot(results_controlinput)
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.4, 0.5, 0.6, 0.25]);

%coefficients plot
coefficients_plot(results_coeffs, vis_data)
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.4, 0.75, 0.6, 0.25]);

%% plot a 2D/3D trajectory
trajectory_reconstruction(results_state)
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.5, 0.4, 0.5]);

%%
%plot state
state_plot(results_state,results_Vinf,results_force,results_moments)
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 0.5]);



