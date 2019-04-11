function [CL, CXt, CM, ai_t_avg] = sim_POC(V, alpha, throttles, flaps, verbose)
    if nargin == 0
        V = 10;         %m/s
%                      M1  M2  M3  M4  M5  M6  M7  M8                     
        throttles   =  [1   1   1   1   1   1   1   1];
%                      F1  F2  F3  F4 
        flaps       =  [1   1   1   1].*40*pi/180;
        
        alpha = 15;
        
        verbose = 1;
    end
    
    useTAT = 0;
    
    
    v_oper = 22;    %Operating voltage
    hd_c = .5;
    Nmotor = 8;     %Assumptions that this is 8 are hard_coded into initialize_geometry.m 
    Nflaps = 4;     %Assumptions that this is 4 are hard_coded into initialize_geometry.m    
    
    motorfile = 'sc3014.txt';
    propfile  = 'cam9x6.txt';
    [diff_t, id, it] = unique(throttles);    %Figure out how many different throttle settings there are. 
    T = length(diff_t);
    
    CJ_diff = zeros(1,T);                    %Get the dCJ of each unique motor
    dCJ_diff = zeros(1,T);
    for i = 1:T
        if diff_t(i) > 0
            v_motor = diff_t(i)*v_oper;         %Operating voltage of each motor
            s = ['~/tools/Qprop/bin/qprop ',propfile, ' ', motorfile, ' ', num2str(V),' 0 ',num2str(v_motor) ' > result.out'];
            system(s);
            [V, RPM, T_N, Q_Nm, Pshaft_W, ...
            Volts, Amps, eta_mot, eta_prop, DV] = read_qprop_out('result.out');
            VJ = V+DV;
            CJ_diff(i) = hd_c*(VJ^2/V^2 + VJ/V);
            dCJ_diff(i) = hd_c*(VJ^2/V^2-1)*(V/VJ+1);
        else
            CJ_diff(i) = hd_c*2;
            dCJ_diff = 0;
        end
    end
    
    CJs = zeros(1,Nmotor);
    dCJs = zeros(1,Nmotor);
    
    for i = 1:Nmotor
        CJs(i) = CJ_diff(it(i));
        dCJs(i) = dCJ_diff(it(i));
    end
    
    [~,CL,CXw, CM, ai, y] = run_NWVL3(alpha, dCJs, flaps, verbose, useTAT);
    insq2msq = 0.00064516;
    W = 36 * 4.45;
    Sref = 17.95*.3048^2;
    bref = 13*.3048;
    bh = 4.4*.3048;
    bv = 2.3*.3048;
    rho = 1.225;
    airplane.geometry.Wing.Sref = Sref;
    airplane.geometry.Htail.S = 639*insq2msq;
    airplane.geometry.Htail.Swet = 2.1*airplane.geometry.Htail.S;
    airplane.geometry.Htail.x_c_m = .3;
    airplane.geometry.Htail.t_c_avg = .12;
    airplane.geometry.Htail.c_ma = 1.05*.3048;
    airplane.geometry.Htail.b = bh;
    airplane.geometry.Htail.AR = bh^2/airplane.geometry.Htail.S;
    
    airplane.geometry.Vtail.S = 480*insq2msq;
    airplane.geometry.Vtail.Swet = 2.1*airplane.geometry.Vtail.S;
    airplane.geometry.Vtail.x_c_m = .3;
    airplane.geometry.Vtail.t_c_avg = .12;
    airplane.geometry.Vtail.c_ma = 1.29*.3048;
    airplane.geometry.Vtail.b = bv;
    airplane.geometry.Vtail.AR = bv^2/airplane.geometry.Vtail.S;
    
    airplane.geometry.Fuse.l = 5.68*.3048;
    airplane.geometry.Fuse.fr = 5.68/1.79;
    airplane.geometry.Fuse.Swet = 2.5*airplane.geometry.Wing.Sref;
    
    airplane.sim.flight_condition = 'std day';
    
    [CDp, Drag_Decomp] = getProfileDrag(airplane, 0, 0.05, 'clean');
    fexcr = 1.1;
    CDp = CDp*fexcr;
    CL_stall = 2*W/(rho*V^2*Sref);
    CXt = CXw+CDp;
    disp(['CDp                     = ',num2str(CDp)])
    disp(['CXtot                   = ',num2str(CXt)])
    disp(['CL_stall                = ',num2str(CL_stall)])
    
    C = 0;
    for i = 1:length(ai)
        if abs(y(i)) <= bh/2
            C = C+1;
        end
    end
    ai_tail = zeros(1,C); 
    y_tail = zeros(1,C);
    k = 1;
    for i = 1:length(ai)
        if abs(y(i)) <= bh/2
            y_tail(k) = y(i);
            ai_tail(k) = ai(i);
            k = k+1;
        end
    end
    
    ai_t_avg = mean(ai_tail);
    if verbose
        figure()
        plot(y_tail, 2*ai_tail*180/pi);
        disp(['ai_tail (avg)           = ',num2str(2*ai_t_avg*180/pi), ' deg'])
    end
end

