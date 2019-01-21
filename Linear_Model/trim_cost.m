function J = trim_cost(OptParams)
%TRIM_COST Calculates the trim cost function.
%OptParam(1) = Elevator deflection
%OptParam(2) = blowing throttle setting
%OptParam(3) = pitch angle

global TrimHist x u V

R	=	[1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1];
u(1) = OptParams(1);
u(6) = OptParams(2);
u(7) = OptParams(2);

x(1) = V*cos(OptParams(3));
x(2) = V*sin(OptParams(3));
x(4) = OptParams(3);

xdot = EoM3(1,x);
xCost = [xdot(1);xdot(2);xdot(3); xdot(12)];

J		=	xCost' * R * xCost;
ParamCost	=	[OptParams;J];
TrimHist	=	[TrimHist ParamCost];
end
