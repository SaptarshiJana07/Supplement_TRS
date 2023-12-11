function value = SLIP_optim(x0)
global ltd wn th_td l0 Del_ltd g 
%---Variable assignment---%
Theta_dot_0 = x0(1);
L_dot_0     = x0(2);
TP          = x0(3);
wn          = x0(4);

%---Conditions----%
l0 = (ltd + (g*cos(th_td)/wn^2) - Del_ltd); % free length

timespan = 0:.01:TP;
initial_conditions = [th_td, Theta_dot_0, ltd, L_dot_0]';
options = odeset('AbsTol', 1e-13, 'RelTol', 1e-13);

%---ODE solver----%
[t, x] = ode45(@SLIP_eom, timespan, initial_conditions, options);

%---suffix f =Just before impact----%
x_f     = x(length(timespan),:);
theta_f = x_f(1,1); 
l_f     = x_f(1,3);

%---Impact Matx, x_p=Just after impact----%
x_p = [-1 0 0 0;
        0 cos(2*theta_f) 0 sin(2*theta_f)/l_f;
        0 0 1 0;
        0 -l_f*sin(2*theta_f) 0 cos(2*theta_f)]*x_f';

%---objective function---%
value =norm(x_p - initial_conditions);
end