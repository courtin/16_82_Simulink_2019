alpha = [0:5:40];
A = length(alpha);
V = 10;         %m/s
              % M1  M2  M3  M4  M5  M6  M7  M8                     
%throttles   =  [1   1   1   1   1   1   1   1;...   %All_on_max
%                0   1   1   1   1   1   1   0;...   %Outboard_off_max
%                0   .5  1   1   1   1   .5  0;...   %Case_3
%                0   .5  .75 1   1   .75 .5  0];     %Case_4
throttles   =  [0   1   1   1   1   1   1   0].*.8;   %Approach
              % F1  F2  F3  F4 
%flaps       =  [40  40  40  40;...                  %All_on_max
%                40  40  40  40;...                  %Outboard_off_max
%               40  40  40  40;...                  %Case_3
%                40  40  40  40].*pi/180;            %Case_4
flaps       =  [40  40  40  40].*pi/180;             %Approach
            
case_names  = ["All_on_max", "Outboard_off_max", "Case_3", "Case_4"];
case_names = ["Approach"];
T = length(throttles(:,1));
blow_center = 0;

for t = 1:T
    CLs = zeros(1,A);
    CXs = zeros(1,A);
    CMs = zeros(1,A);
    CL_tb = zeros(1,A);
    ai_t = zeros(1,A);
    for a = 1:A
        [CLs(a), CXs(a), CMs(a),ai_t(a),CL_tb(a)] = sim_POC(V, alpha(a), throttles(t,:), flaps(t,:), 0, blow_center);
    end
    throttle = throttles(t,:);
    flap = flaps(t,:);
    filename = ["runs/"+case_names(t)+".mat"];
    save(filename, 'alpha', 'CLs', 'CXs', 'CMs', 'ai_t','CL_tb', 'throttles', 'flaps', 'V')
end
cases_to_plot = ["All_on_max", "Outboard_off_max", "Case_3", "Case_4"];
cases_to_plot = ["Approach"];
plot_3d_polar(cases_to_plot)