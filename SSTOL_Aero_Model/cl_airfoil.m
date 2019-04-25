function cl_t=cl_airfoil(a_h)
%% IN DEGREES
%%
if abs(a_h)>=11 & abs(a_h) <= 15
        cl_t=sign(a_h)*(1/11*11-(abs(a_h)-11)*(1-0.6)/(15-11));
    elseif abs(a_h)>15 & abs(a_h)<=42
        cl_t=sign(a_h)*(1/11*11-(15-11)*(1-0.6)/(15-11)+(abs(a_h)-15)*(1.1-0.6)/27);
    elseif abs(a_h)>42
        cl_t=sign(a_h)*(1/11*11-(15-11)*(1-0.6)/(15-11)+(42-15)*(1.1-0.6)/27-1.1/(92-42)*(abs(a_h)-42));
    else
        cl_t=1/11*a_h;
end 
end