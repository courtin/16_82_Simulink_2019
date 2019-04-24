% Lifting Line Code 
clc;clear all;close all 
% Input Wing Geometry 
for Lambda = 0.25:0.25:1 
    % Number of wing segments 
    N_seg = 30; 
    % Wing Area [m^2] 
    S = 65; 
    % Aspect Ratio 
    AR = 25; 
    % Twist [deg] 
    twist = -6; 
    % Wing Setting Angle/ AoA [deg] 
    set_ang = 2; 
    % Wingspan [m] 
    b = sqrt(AR*S); 
    % Input Aerofoil Data 
    % Lift Curve Slope [1/rad] 
    slope = 6.9; 
    % Mean Aerodynamic Chord 
    MAC = S/b; 
    % Zero Lift Angle of Attack 
    zl_AoA = -6.56; 
    % root chord (m) 
    Root_Chord = (2*S) / ((1 + Lambda) * b); 
    % tip chord (m) 
    Tip_Chord = ((2*S) / ((1 + Lambda) * b)) * (1 - ((2* (1 - Lambda)) / b) * (b/2) );
    % MAC 
    MAC = (2/3) * (Root_Chord + Tip_Chord - (Root_Chord*Tip_Chord) / (Root_Chord +Tip_Chord)); 
    % Lifting Line Algorithm 
    theta = pi/(2*N_seg):pi/(2*N_seg):pi/2; 
    % create vector containing each segment's angle of attack 
    alpha = set_ang + twist: -twist/(N_seg-1):set_ang; z = (b/2)*cos(theta); 
    % Mean Aerodynamic Chord at each segment (m) 
    c = Root_Chord * (1 - (1-Lambda)*cos(theta));
    mu = c * slope / (4 * b); 
    LHS = mu .* (alpha-zl_AoA) * (pi/180); 
    % Determine Coefficients A(i) by Solving N_seg Equations: 
    for i=1:N_seg 
        for j=1:N_seg 
            B(i,j) = sin((2*j-1) * theta(i)) * (1 + (mu(i) * (2*j-1)) / sin(theta(i))); 
        end
    end
    A=B\transpose(LHS); 
    for i = 1:N_seg 
        sum1(i) = 0; 
        sum2(i) = 0; 
        for j = 1 : N_seg 
            sum1(i) = sum1(i) + (2*j-1) * A(j)*sin((2*j-1)*theta(i)); 
            sum2(i) = sum2(i) + A(j)*sin((2*j-1)*theta(i)); 
        end
    end
    % Determine Lift Coefficient For Each Segment 
    CL = 4*b*sum2 ./ c; 
    % Plot Lift Distribution 
    CL1=[0 CL]; 
    y_s=[b/2 z]; 
    if Lambda == 0.25 
        plot(y_s,CL1,'-o') 
    elseif Lambda == 0.5 
        plot(y_s,CL1,'-*') 
    elseif Lambda == 0.75 
        plot(y_s,CL1,'-s') 
    else
        plot(y_s,CL1,'-d')
    end
    hold on 
    % Output Lift Coefficient for Wing 
    CL_wing = pi * AR * A(1) 
    % Output Oswald Efficiency Factor Induced Drag Coefficent 
    for n = 2:30 
        delta(n) = n * (A(n)/A(1))^2; 
    end
    delta1 = sum(delta); 
    e = 1 / (1 + delta1) 
    CD_induced = CL_wing^2 / (pi * e * AR) 
end
% Plot Elliptical Lift Distribution For Comparison 
y_ellipse = linspace(0,(b/2),100); 
for i = 1:length(y_ellipse) 
    CL_ellipse(i) = CL1(end) * sqrt(1 - ((2*y_ellipse(i)) / b)^2 ); 
end
plot (y_ellipse, CL_ellipse, 'r-') 
grid on 
title('Variation of Lift distribution with Taper Ratio') 

xlabel('Semi-Span Location [m]') 
ylabel ('C_L') 
legend('location','best','Lambda = 0.25','Lambda = 0.5','Lambda = 0.75','Lambda = 1','Elliptical Lift Distribution')
