function state_plot(stateHistory,VinfHistory,forceHistory, momentHistory)


t = stateHistory.Time;


U = stateHistory.Data(:,1);
V = stateHistory.Data(:,5);
W = stateHistory.Data(:,2);

phi = rad2deg(stateHistory.Data(:,8));
theta = rad2deg(stateHistory.Data(:,4));
psi = rad2deg(stateHistory.Data(:,9));

p = rad2deg(stateHistory.Data(:,6));
q = rad2deg(stateHistory.Data(:,3));
r = rad2deg(stateHistory.Data(:,7));

Xe = stateHistory.Data(:,10);
Ye = stateHistory.Data(:,11);
Ze = -stateHistory.Data(:,12);

Vinf = VinfHistory.Data(:,1);
alpha = VinfHistory.Data(:,2);
beta  = VinfHistory.Data(:,3);

FX = forceHistory.Data(:,1);
FY = forceHistory.Data(:,2);
FZ = forceHistory.Data(:,3);


MX = momentHistory.Data(:,1);
MY = momentHistory.Data(:,2);
MZ = momentHistory.Data(:,3);




fig_h = figure

%location
ax1=subplot(2,4,1);
plot(t,Xe,t,Ye,t,Ze)
legend('Xe','Ye','Ze')
grid on
title('Position')

ax2=subplot(2,4,2);
plot(t,U,t,V,t,W)
legend('U','V','W')
grid on
title('Velocity (Body)')

ax3=subplot(2,4,3);
plot(t,Vinf)
grid on
title('Vinf')

ax4 = subplot(2,4,4);
plot(t,FX,t,FY,t,FZ)
legend('FX','FY','FZ')
grid on
title('Forces')

ax5=subplot(2,4,5);
plot(t,phi,t,theta,t,psi)
legend('phi','theta','psi')
grid on
title('orientation (deg)')

ax6=subplot(2,4,6);
plot(t,p,t,q,t,r)
legend('p','q','r')
grid on
title('rotation rate (deg/s)')

ax7=subplot(2,4,7);
plot(t,alpha,t,beta)
legend('alpha','beta')
grid on
title('free stream angle (deg)')

ax8 = subplot(2,4,8);
plot(t,MX,t,MY,t,MZ)
legend('MX','MY','MZ')
grid on
title('Moments')

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8],'x')


movegui(fig_h,'northeast')