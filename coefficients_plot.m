function coefficients_plot(results_coeffs)

t = results_coeffs.Time;

figure;

ax1= subplot(2,3,1);
plot(t,results_coeffs.Data(:,1))
ylabel('CX')

ax2 = subplot(2,3,2);
plot(t,results_coeffs.Data(:,2))
ylabel('CL')

ax3 = subplot(2,3,3);
plot(t,results_coeffs.Data(:,3))
ylabel('CY')


ax4 = subplot(2,3,4);
plot(t,results_coeffs.Data(:,4))
ylabel('Cl')

ax5 = subplot(2,3,5);
plot(t,results_coeffs.Data(:,5))
ylabel('Cm')

ax6 = subplot(2,3,6);
plot(t,results_coeffs.Data(:,6))
ylabel('Cn')

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')
