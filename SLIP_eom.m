function xdot = SLIP_eom(t,x)

global wn ltd g  l0
%---State variables----%
theta=x(1);
theta_dot=x(2);
l=x(3);
l_dot=x(4);
%---Time scaled by sqrt(g/ltd)
l1=(l/ltd); l1_dot=(l_dot/ltd); g1 = g/ltd;
%---State equations----%
xdot(1,1) = theta_dot;
xdot(2,1) =-(2/l1)*l1_dot*theta_dot +(1/l1)*sin(theta);
xdot(3,1) = l_dot;
xdot(4,1) = l*theta_dot^2 -ltd*cos(theta)-(wn^2/g1)*(l-l0);
end

