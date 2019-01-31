function coefficients_plot(results_coeffs)

t = results_coeffs.Time;

fig_h = figure;

ax1= subplot(2,3,1);
plot(t,results_coeffs.Data(:,1))
ylabel('CX')
grid on

ax2 = subplot(2,3,2);
plot(t,results_coeffs.Data(:,2))
ylabel('CL')
grid on

ax3 = subplot(2,3,3);
plot(t,results_coeffs.Data(:,3))
ylabel('CY')
grid on

ax4 = subplot(2,3,4);
plot(t,results_coeffs.Data(:,4))
ylabel('Cl')
grid on

ax5 = subplot(2,3,5);
plot(t,results_coeffs.Data(:,5))
ylabel('Cm')
grid on

ax6 = subplot(2,3,6);
plot(t,results_coeffs.Data(:,6))
ylabel('Cn')
grid on

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')
