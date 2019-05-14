function rimlesswheel
% RIMLESSWHEEL simulates rimless wheel descending down a ramp.
%
% Needs ODE113, FSOLVE, INTERP1. 
% If you find bugs in this code please mail, 
% Pranav A. Bhounsule, pab47@disneyresearch.com
% Last updated: 25 August 2013
% This code is provided free of charge but without any guarantees 


format long
close all
clc

%%%%% Dimensions %%%%%%%%
parms.lambda = sqrt(2/3);
parms.mu = 2/3;
parms.n = 6; %spokes 
parms.gam = 0.2; %slope

%%%% Initial State %%%%%
q1 = -pi/6; %Clock-wise direction is positive. 
u1 = 0.3; 
%zstar =  [-0.523598775598299   0.460341126609455];

steps = 5; %number of steps to animate
fps = 30; %frames per second

z0 = [q1 u1];

%%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-12,'TolX',1e-12,'Display','off');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,parms);
if exitflag == 1
    disp('Fixed points are');
    zstar
else
    %disp('fsolve not converged');
    error('Root finder not converged, change the initial guess')
end

%%% Stability, using linearised eigenvalue %%%
disp('EigenValues for linearized map (finite difference) are');
J=partialder(@onestep,zstar,parms);
eig(J)


disp('EigenValues for linearized map (monodromy matrix) are using hand calculations');
J=monodromy(zstar,parms);
eig(J)


%%%%%% Get data for all the steps %%%
[z,t] = onestep(zstar,parms,steps);


%%% Animate result %%%
disp('Animating...');
disp('NOTE: Animation speed can be changed using fps defined in the code');
animate(t,z,parms,steps,fps);
 
%%% Plot data %%%
disp('Some plots...')
plot(t,z(:,1),'r',t,z(:,2),'b')
xlabel('time'); ylabel('Angle (rad), Angular Rate (rad/s)');
legend('Angle','Angular Rate');
title('State variables for Rimless wheel');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FUNCTIONS START HERE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%===================================================================
function zdiff=fixedpt(z0,parms)
%===================================================================
zdiff=onestep(z0,parms)-z0; 

%===================================================================
function [z,t]=onestep(z0,parms,steps)
%===================================================================

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state
    steps = 1;
end

t0 = 0; 
dt =5;
t_ode = t0;
z_ode = [z0 sin(z0(1,1)) cos(z0(1,1))];
x_trans = 0; %distace moved by center of wheel.

for i=1:steps
    options=odeset('abstol',2.3e-14,'reltol',2.3e-14,'events',@collision);
    tspan = linspace(t0,t0+dt,1000);
    [t_temp, z_temp] = ode113(@single_stance,tspan,z0,options,parms);
    
    zplus=heelstrike(t_temp(end),z_temp(end,:),parms); 
    
    z0 = zplus;
    t0 = t_temp(end);
   
    %post-processd data for storing
    xhtemp = sin(z_temp(:,1))+x_trans; %x of wheel
    yhtemp = cos(z_temp(:,1));    %y of wheel
    
    x_trans = x_trans + (xhtemp(end)-xhtemp(1));
    
    x0temp = sin(z0(1,1))+x_trans; %xhtemp(end);      
    y0temp = cos(z0(1,1));
    
   
    
    t_ode = [t_ode; t_temp(2:end); t0];
    
    z_ode = [z_ode;  ... 
             z_temp(2:end,:),xhtemp(2:end,1),yhtemp(2:end,1); ... 
             z0,x0temp,y0temp];
    
end

z = zplus(1:2);

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=single_stance(t,z,parms)  
%===================================================================

q1 = z(1);   u1 = z(2);     
f = [u1 (parms.lambda^2)*sin(q1+parms.gam)]';
zdot = f;

%===================================================================
function zplus=heelstrike(t,zminus,parms)      
%===================================================================

r1 = zminus(1);   
v1 = zminus(2);

g = [-r1, parms.mu*v1];
zplus = g;

%===================================================================
function [h, isterminal,direction]=collision(t,x,parms)
%===================================================================

n = parms.n;
q1 = x(1);

h = q1 - pi/n ;
isterminal=1; %Ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
direction= 1; % The t_final can be approached by any direction is indicated by this

%===================================================================
function J=partialder(FUN,z,parms)
%===================================================================
pert=1e-5;

%%%% Using forward difference, accuracy linear %%%
% y0=feval(FUN,z,GL_DIM); 
% for i=1:length(z)
%     ztemp=z;
%     ztemp(i)=ztemp(i)+pert; 
%     J(:,i)=(feval(FUN,ztemp,GL_DIM)-y0) ;
% end
% J=(J/pert);

%%%% Using central difference, accuracy quadratic %%%
for i=1:length(z)
    ztemp1=z; ztemp2=z;
    ztemp1(i)=ztemp1(i)+pert; 
    ztemp2(i)=ztemp2(i)-pert; 
    J(:,i)=(feval(FUN,ztemp1,parms)-feval(FUN,ztemp2,parms));
end
J=J/(2*pert);

%===================================================================
function J = monodromy(zstar,parms)
%===================================================================

F0 = eye(2);
F0vec = reshape(F0,1,4);
x0 = [zstar F0vec];

t0 = 0; dt = 5;
options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision);
tspan = linspace(t0,t0+dt,100);
[tt, xx] = ode113(@sensitivity,tspan,x0,options,parms);

F = reshape(xx(end,3:6),2,2);
FF = sensitivity(tt(end),xx(end,:),parms);
f = FF(1:2); %get first two elements of RHS 
hx = [1 0]; %% This is derivative of the poincare surface
gx = [-1 0;....
     0  parms.mu]; %% This is derivative of jump condition wrt to x 
J = F*(gx - (gx*f*hx)/(hx*f)); %monodromy matrix
  
%===================================================================
function dxdt = sensitivity(t,x,parms)
%===================================================================

F = reshape(x(3:6),2,2);
f = [x(2) (parms.lambda^2)*sin(x(1)+parms.gam)]';
dfdx = [0 1; (parms.lambda^2)*cos(x(1)+parms.gam) 0]; 
dFdt = dfdx*F;
dFdtvec = reshape(dFdt,4,1);

dxdt = [f;dFdtvec];

%===================================================================
function animate(t_all,z_all,parms,steps,fps)
%===================================================================

%%%% First, get the unique values of states %%%%
[t_unique,index] = unique(t_all,'first');

z_unique = [];
for i = 1:length(index)
   z_unique = [z_unique; z_all(index(i),1),  z_all(index(i),3),  z_all(index(i),4)];
end

%%%% Second, interpolate linearly using fps %%%%%
[m,n] = size(z_unique);
t = linspace(0,t_unique(end),fps*steps);
for i=1:n
    z(:,i) = interp1(t_unique,z_unique(:,i),t);
end

%%%% Lastly, animate the results
clf
   
dth = 2*pi/parms.n; %spokes spacing in radians. n = number of spokes.
[m,n]=size(z);

min_xh = min(z(:,2)); max_xh = max(z(:,2)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/m;

window_xmin = -2.0; window_xmax = 1.0;
window_ymin = -0.1; window_ymax = 2+0.1;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%% creat object for center of rimles wheel %%%%%
hingepic=line('xdata',0,'ydata',0, 'marker','.','markersize',20, ...
          'erase','xor','color','black');

%%%% create object for spokes %%%%
barref = [0 0; 0 1]; %%% bar along positive y-axis
      
for p = 1:parms.n
    barpic(p)=line('xdata',barref(1,:),'ydata',barref(2,:),'linewidth', 2, 'erase','xor','color','blue');
end

%%%% create ramp %%%%
rampref=[min_xh-2 max_xh+2 ; 0 0];

ramppic=line('xdata',rampref(1,:),'ydata',rampref(2,:), ...
            'linewidth', 1,'color','black');
     

moviescaling = 1;                      % slow down factor
delay =floor(moviescaling); %delay per frame in .001 secs


for i=1:m
   
   for j=1:100, log(1:delay*17); end %delay for graphics. 
	                                 %the number in this expression
									 %is machine dependent.
									 %The LOG is just something
									 %to keep the machine busy.

   q1 = z(i,1);  
   xh = z(i,2); yh = z(i,3);  
  
   
   window_xmin = window_xmin + camera_rate;
   window_xmax = window_xmax + camera_rate;
   axis('equal')
   axis([window_xmin window_xmax window_ymin window_ymax])

   hinge=[xh; yh];    

        for p = 1:parms.n
            A = q1-(p-1)*dth;
            bar(:,:,p) = [hinge, hinge] + R(A)*barref;
        end

    set(hingepic,'xdata',hinge(1),'ydata',hinge(2));
   
    for p=1:parms.n
        set(barpic(p),'xdata',bar(1,:,p),'ydata',bar(2,:,p));
    end

    drawnow  
  
end

%===================================================================
function rotation = R(A)
%===================================================================
rotation = [cos(A) -sin(A); sin(A) cos(A)];


