%% W/J gait optimization on flat ground
% [fmincon]*4 variable*K,theta_dot,l_dot,Time Period
% Del_ltd = const, Theta_td = const; find K? 
global l0 ltd wn T g Del_ltd th_td

%---model parameters----%
g       = 9.81; % gravity
m       = 80; % hip mass
ltd     = 1; % length at touchdown (TD)
Del_ltd = 0.06; % initial compression
K       = 8000; % spring stiffness
wn      = sqrt(K/m);

%---Initial guess----%
T     = 2.0; % Time period
th_td = 0.20;  % Leg angle at TD
theta_dot_0 = -2.4244889e-01;
l_dot_0     = -4.9481623e-02;

l0 = (ltd+(g*cos(th_td)/wn^2)-Del_ltd);
x0 = [theta_dot_0, l_dot_0, T, wn]; %first guess for fmincon
 
%---Optimization function calling----%
A = []; b = []; Aeq = []; beq = [];
lb = [-0.5, -0.2, 1, 7];
ub = [-0.1, 0, 4, 12];
nonlcon = [];
options = optimset('MaxIter',400*6,'TolFun',1e-15,'TolX',1e-15); % 'Display','iter',
[x, fval,exitflag,output] = fmincon(@SLIP_optim,x0,A,b,Aeq,beq,lb,ub,nonlcon,options); 

%---After optimization ODE solved----%
xf(1) = th_td; 
xf(2) = x(1); 
xf(3) = ltd; 
xf(4) = x(2); 
T     = x(3); 
wn    = x(4);
initial_condition = xf'; % Suitable IC found

timespan = linspace(0,T,200);
options = odeset('AbsTol',1e-12,'RelTol',1e-12);
[t, y] = ode45(@SLIP_eom, timespan, initial_condition, options);

K = m*wn^2; % leg stiffness
step_size = 2*ltd*abs(sin(th_td)); 
gait_speed = step_size/(T*sqrt(ltd/g)); 
disp(gait_speed) % locomotion speed display
%% State variables & Kinematics
theta     = y(:,1);
theta_dot = y(:,2)*sqrt(g/ltd); % time scaled back
l         = y(:,3);
l_dot     = y(:,4)*sqrt(g/ltd);
time = linspace(0,100,length(t))';
%---Position of the hip-mass----%%
x_h = -l.*sin(theta);
y_h =  l.*cos(theta);

%---Velocity of the hip-mass----%%
xd_h =-l_dot.*sin(theta) - l.*theta_dot.*cos(theta);
yd_h = l_dot.*cos(theta) - l.*theta_dot.*sin(theta);
v_h  = sqrt(xd_h.*xd_h + yd_h.*yd_h);

%---acclerationof the hip-mass----%
theta_dot_dot = -(2./l).*l_dot.*theta_dot + ltd*(g./l).*sin(theta);
l_dot_dot     = l.*theta_dot.^2 - g*cos(theta) - (wn.^2).*(l-l0);

%---accleration in XY-coordinate----%
x_hip_acc = -l_dot_dot.*sin(theta)-2*l_dot.*theta_dot.*cos(theta)...
    -l.*theta_dot_dot.*cos(theta)+l.*((theta_dot).^2).*sin(theta);
y_hip_acc = l_dot_dot.*cos(theta)-2*l_dot.*theta_dot.*sin(theta)...
    -l.*theta_dot_dot.*sin(theta)-l.*((theta_dot).^2).*cos(theta);

%% vertical Ground Reaction Force w.r.t. body weight
GRF_y = 1 + (1/g)*y_hip_acc;
%% Plot
figure
plot1=subplot(1,2,1);
plot2=subplot(1,2,2);

plot(plot1,time,GRF_y,'-.k','LineWidth',2)
set(gca,'LineWidth',1.5,'FontUnits','points','fontsize',10,'fontname','Times')
title(plot1,'vGRF vs time','fontsize',14,'fontname','Times','interpreter','latex')
ylabel(plot1,'$GRF_y$','fontsize',12,'fontname','Times','interpreter','latex')
xlabel(plot1,'time','fontsize',12,'fontname','Times','interpreter','latex')
text(plot1,45,0.7,['\Deltal_{td}: ' num2str(Del_ltd)])
axis (plot1,'tight')
 
plot(plot2,time,y_h,'-.b','LineWidth',2)
set(gca,'LineWidth',1.5,'FontUnits','points','fontsize',10,'fontname','Times')
title(plot2,'$COM_y$ vs time','fontsize',14,'fontname','Times','interpreter','latex')
ylabel(plot2,'$COM_y$','fontsize',12,'fontname','Times','interpreter','latex')
xlabel(plot2,'time','fontsize',12,'fontname','Times','interpreter','latex')
axis (plot2,'tight')

