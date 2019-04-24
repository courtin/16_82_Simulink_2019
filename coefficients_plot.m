function coefficients_plot(results_coeffs, vis_data)

t = results_coeffs.Time;
vdata_shape = size(vis_data.data);
vdata = reshape(vis_data.data, vdata_shape(2:3));

fig_h = figure;

ax1= subplot(2,5,1);
plot(t,results_coeffs.Data(:,1))
ylabel('CX')
ylim([-10,10]);
grid on

ax2 = subplot(2,5,2);
plot(t,results_coeffs.Data(:,2))
ylabel('CL')
ylim([-10,10]);
grid on

ax3 = subplot(2,5,3);
plot(t,results_coeffs.Data(:,3))
ylabel('CY')
ylim([-10,10]);
grid on

%4 - CLw
ax4 = subplot(2,5,4);
plot(t,vdata(3,:))
ylabel('CL_w')
ylim([-10,10]);
grid on

%5 - CL_tail
ax5 = subplot(2,5,5);
plot(t,vdata(4,:))
ylabel('CL_{tail}')
ylim([-10,10]);
grid on

%6
ax6 = subplot(2,5,6);
plot(t,results_coeffs.Data(:,4))
ylabel('Cl')
ylim([-10,10]);
grid on

%7
ax7 = subplot(2,5,7);
plot(t,results_coeffs.Data(:,5))
ylabel('Cm')
ylim([-10,10]);
grid on

%8
ax8 = subplot(2,5,8);
plot(t,results_coeffs.Data(:,6))
ylabel('Cn')
ylim([-10,10]);
grid on

%9 - Cmw
ax9 = subplot(2,5,9);
plot(t, vdata(6,:))
ylabel('Cm_w')
ylim([-10,10]);
grid on

%10 - dCJ
ax10 = subplot(2,5,10);
hold on;
plot(t, vdata(1,:))
plot(t, vdata(2,:))
ylabel('dCJ')
ylim([-10,10]);
grid on

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9,ax10],'x')
