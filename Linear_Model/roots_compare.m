%Compare lowspeed and highspeeed roots

e1 = load('eig_low.mat');
e2 = load('eig_high.mat');
figure()
ax = axes();
scatter(real(e2.e), imag(e2.e), 'b')
hold on 
scatter(real(e1.e), imag(e1.e), 'r')
legend('High-speed roots: \delta_f = 0, V_{trim} = 100 kts', 'Low-speed roots: \delta_f = 40, V_{trim} = 30 kts')
title('Roots of F matrix')
grid
ax.YAxisLocation = 'origin';
ax.XAxisLocation = 'origin';