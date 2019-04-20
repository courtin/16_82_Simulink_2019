function control_plot(results_controlinput)
%%
t = results_controlinput.Time;

fig_h = figure;

ax1 = subplot(2,3,1);
plot(t, rad2deg(results_controlinput.Data(:, 1)))
ylabel('Elevator (deg)')
grid on


ax2 = subplot(2,3,2);
plot(t, rad2deg(results_controlinput.Data(:, 2)))
ylabel('Rudder (deg)')
grid on


ax3 = subplot(2,3,3);
plot(t, rad2deg(results_controlinput.Data(:, 3)))
ylabel('Aileron (deg)')
grid on

ax4 = subplot(2,3,4);
plot(t, rad2deg(results_controlinput.Data(:, 4)),t, rad2deg(results_controlinput.Data(:, 5)))
ylabel('Flap (deg)')
grid on

ax5 = subplot(2,3,5);
plot(t, results_controlinput.Data(:, 6:end))
ylabel('Throttle')
grid on
legend('Cruisers','Blowers')


linkaxes([ax1,ax2,ax3,ax4,ax5],'x')

end
