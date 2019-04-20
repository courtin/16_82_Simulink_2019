load('Airplane.mat')
load('POC_motor_db.mat')
propulsor=airplane.propulsion.right_cruiser;
V=0:5:65;
volt=1:1:22;
S=airplane.geometry.Wing.S;
c=airplane.geometry.Wing.cbar;
alt=0;
for i = 1:length(volt)
    for j=1:length(V)
       [~, ~, T_sim(j,i), ~] = propulsor_perf_qprop(volt(i)/propulsor.max_voltage, propulsor,c,S, alt, V(j));
       [~, Vj_sim(j,i), ~,~] = propulsor_perf_qprop(volt(i)/propulsor.max_voltage, propulsor,c,S, alt, V(j));
    end
    subplot(211)
    plot(V,T(:,i))
    ylim([-10 40])
    ylabel('Thrust (N)')
    xlabel('Speed (m/s)')
    title('QPROP RAW DATA')
    hold on
    subplot(212)
    plot(V,T_sim(:,i))
    ylim([-10 40])
    ylabel('Thrust (N)')
    xlabel('Speed (m/s)')
    title('Simulation fit')
    hold on
end
hold off

