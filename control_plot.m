function control_plot(results_controlinput)
%%
t = results_controlinput.Time;

figure;

ax1 = subplot(2,3,1);
plot(t, rad2deg(results_controlinput.Data(:, 1)))
ylabel('Elevator (deg)')


ax2 = subplot(2,3,2);
plot(t, rad2deg(results_controlinput.Data(:, 2)))
ylabel('Rudder (deg)')

ax3 = subplot(2,3,3);
plot(t, rad2deg(results_controlinput.Data(:, 3)))
ylabel('Aileron (deg)')

ax4 = subplot(2,3,4);
plot(t, rad2deg(results_controlinput.Data(:, 4)),t, rad2deg(results_controlinput.Data(:, 5)))
ylabel('Flap (deg)')

ax5 = subplot(2,3,5);
plot(t, results_controlinput.Data(:, 6:end))
ylabel('Blowing')


linkaxes([ax1,ax2,ax3,ax4,ax5],'x')


