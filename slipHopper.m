% SLIPHOPPER simulates Spring Loaded Inverted Pendulum (SLIP) 
% find limit cycles and then find Jacobian to evaluate linear stability
% Needs ODE113, FSOLVE, INTERP1. 
% If you find bugs in this code please mail, 
% Pranav A. Bhounsule, pab47@cornell.edu  
% Last updated: 18 May 2016
%
% For details about the model etc. see
% Schwind, William John. "Spring loaded inverted pendulum running: 
% A plant model." PhD diss., 1998.

function slipHopper  

clc
clear all
close all
format long

% %%% initial parameters (set 1)
% robot.g = 10;
% robot.ground = 0; %ground is at y co-ordinate equal to robot.ground
% robot.l = 1;
% robot.m = 100;
% robot.control.k = 10000;
% robot.control.theta  = 5*(pi/180); %pi/6; %angle between leg and vertical
% x0dot = 0.50435	;% 0.5;  
% y0 = 1.1873	;
% %Root finding will give this root
% %zstar = [0.513806075894484   1.238312718076982];

%%%% initial parameters (set 2)
robot.g = 10;
robot.ground = 0; %ground is at y co-ordinate equal to robot.ground
robot.l = 1;
robot.m = 1;
robot.control.k = 100;
robot.control.theta  = 10*(pi/180); %angle between leg and vertical
x0dot =  1; 
y0 = 1.2;
% % Root finding will give this unstable root 
% % zstar = [1.017450873821607   1.182441943200824];

% initial guess
z0 = [x0dot y0];

% %%%%%%%%%%%%%%%%%%%%%%%%%
steps = 5; %number of steps to animate
fps = 30; %Use low frames per second for low gravity

%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-10,'TolX',1e-10,'Display','iter');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,robot);
if exitflag == 1
    disp('Fixed point:');
    disp(zstar);
else
    error('Root finder not converged, change guess or change system parameters')
end
 
%%% Stability, using eigenvalues of Poincare map %%%
J=partialder(@onestep,zstar,robot)
disp('EigenValues for linearized map are');
eig(J)
 
%%%% Get data for all the steps %%%
[z,t] = onestep(zstar,robot,steps);

% %%% Animate result %%%
disp('Animating...');
animate(t,z,robot,steps,fps);

%%% Plot data %%%
disp('Some plots...')
figure(2)
subplot(2,1,1)
plot(t,z(:,1),'r',t,z(:,3),'b')
xlabel('time'); ylabel('Angle (m)');
legend('x','y');
subplot(2,1,2)
plot(t,z(:,2),'r',t,z(:,4),'b')
xlabel('time'); ylabel('Velocity (m/s)');
legend('vx','vy');

figure(3)
plot(z(:,1),z(:,3),'r'); %hold on
xlabel('x'); ylabel('y');
title('Trajectory y vs x');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FUNCTIONS START HERE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================================================================
function zdiff=fixedpt(z0,robot)
%===================================================================
zdiff=onestep(z0,robot)-z0; 

%===================================================================
function J=partialder(FUN,z,robot)
%===================================================================
pert=1e-5;
n = length(z);
J = zeros(n,n);

%%%% Using forward difference, accuracy linear %%%
% y0=feval(FUN,z,walker); 
% for i=1:n
%     ztemp=z;
%     ztemp(i)=ztemp(i)+pert; 
%     J(:,i)=(feval(FUN,ztemp,walker)-y0) ;
% end
% J=(J/pert);

%%% Using central difference, accuracy quadratic %%%
for i=1:n
    ztemp1=z; ztemp2=z;
    ztemp1(i)=ztemp1(i)+pert; 
    ztemp2(i)=ztemp2(i)-pert; 
    J(:,i)=(feval(FUN,ztemp1,robot)-feval(FUN,ztemp2,robot)) ;
end
J=J/(2*pert);

%===================================================================
function [z,t]=onestep(z0,robot,steps)  %DONE
%===================================================================

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
end

x0 = 0; x0dot = z0(1);  
y0 = z0(2); y0dot = 0;

z0 = [x0 x0dot y0 y0dot];

t0 = 0; 
dt = 5; %might need to be changed based on time taken for one step
time_stamps = 100;
t_ode = t0;
z_ode = [z0 ...
         x0+robot.l*sin(robot.control.theta) ...
         y0-robot.l*cos(robot.control.theta)];

for i=1:steps
    
    %%% apex to ground %%%
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp1,z_temp1]=ode113(@flight,tspan,z0,options1,robot);
   
    z_temp1 = [z_temp1 ...
               z_temp1(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp1(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp1(end);
    z0(1:4) = z_temp1(end,1:4);
    x_com = z0(1); %save the x position for future
    z0(1) = -robot.l*sin(robot.control.theta); %relative distance wrt contact point because of non-holonomic nature of the system
    x_foot = x_com + robot.l*sin(robot.control.theta); 
    y_foot = robot.ground;
   
    %%% stance phase %%%
    tspan = linspace(t0,t0+dt,time_stamps*10);
    options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
    [t_temp2,z_temp2]=ode113(@stance,tspan,z0,options2,robot);
    
    
    z_temp2(:,1) = z_temp2(:,1) + x_com + robot.l*sin(robot.control.theta); %absolute x co-ordinate
    z_temp2 = [z_temp2, ...
          x_foot*ones(length(z_temp2),1) y_foot*ones(length(z_temp2),1)]; %the distal end of leg is 0 when touching the ground.
    t0 = t_temp2(end);
    z0(1:4) = z_temp2(end,1:4);
    
    %%% ground to apex
    tspan = linspace(t0,t0+dt,time_stamps);
    options3 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@apex);
    [t_temp3,z_temp3]=ode113(@flight,tspan,z0,options3,robot);
    
     z_temp3 = [z_temp3 ...
               z_temp3(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp3(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp3(end);
    z0(1:4) = z_temp3(end,1:4);
    
    
    %%%%% Ignore time stamps for heelstrike and first integration point
    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end);  t_temp3(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:); z_temp2(2:end,:); z_temp3(2:end,:)];
    
end

z = [z0(2) z0(3)];

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=flight(t,z,robot)  
%===================================================================
zdot = [z(2) 0 z(4) -robot.g]';

%===================================================================
function [gstop, isterminal,direction]=contact(t,z,robot)
%===================================================================
gstop = z(3) - robot.l*cos(robot.control.theta); %position is 0;
direction = -1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function zdot=stance(t,z,robot)  
%===================================================================
x = z(1); y = z(3); %x & y position of com wrt ground
l = sqrt(x^2+y^2);
F_spring = robot.control.k*(robot.l-l);
Fx_spring =  F_spring*(x/l);
Fy_spring = F_spring*(y/l);
Fy_gravity = robot.m*robot.g;
xddot = (1/robot.m)*(Fx_spring);
yddot = (1/robot.m)*(-Fy_gravity+Fy_spring);
zdot = [z(2) xddot z(4) yddot]';

%===================================================================
function [gstop, isterminal,direction]=release(t,z,robot)
%===================================================================
l = sqrt(z(1)^2+z(3)^2);
gstop = l-robot.l;
direction = 1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function [gstop, isterminal,direction]=apex(t,z,robot)
%===================================================================
gstop = z(4) - 0; %ydot is 0;
direction = 0; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function animate(t_all,z_all,robot,steps,fps)
%===================================================================

%%% interpolate for animation %%
[m,n] = size(z_all);
tinterp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));
for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),tinterp);
end

[mm,nn] = size(z_interp);
min_xh = min(z_interp(:,1)); max_xh = max(z_interp(:,1)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -1.0*robot.l; window_xmax = robot.l;
window_ymin = -0.1; window_ymax = 1.9*robot.l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

figure(1);
for i=1:length(tinterp)
   
    plot(z_interp(i,1),z_interp(i,3),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',20); %com
    line([-1 max(z_interp(:,1))+1],[0 0],'Linewidth',2,'Color','black'); %ground
    line([z_interp(i,1) z_interp(i,5)],[z_interp(i,3) z_interp(i,6)],'Linewidth',4,'Color',[0 0.8 0]); %leg
     
    window_xmin = window_xmin + camera_rate;
    window_xmax = window_xmax + camera_rate;
    axis('equal')
    axis off
    axis([window_xmin window_xmax window_ymin window_ymax])

    pause(0.05);
end
