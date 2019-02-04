function fz = getlift(alpha, dE, dB, theta, V)

f = getforces(alpha, dE, dB, theta, V);

fz=f(3);