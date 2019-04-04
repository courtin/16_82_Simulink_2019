function [] = sim_POC(V, throttles, flaps)
    V = 10;         %m/s
    v_oper = 18;    %Operating voltage
    
    throttles   = ones(1,8);
    flaps       = ones(1,4);
    
    motorfile = 'sc3014';
    propfile  = 'cam9x6'
    
    [diff_t, id, it] = unique(throttles)    %Figure out how many different throttle settings there are. 
    T = length(diff_t);
    for i = 1:T
        v_motor = diff_t(i)*v_oper;         %Operating voltage of each motor
        s = ['~/tools/Qprop/bin/qprop ',propfile, ' ', motorfile, ' ', num2str(V),' 0 ',num2str(v_motor) ' > result.out'];
        system(s)
    end
        
    
end

