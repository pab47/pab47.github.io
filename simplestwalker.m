function simplestwalker
% SIMPLESTWALKER simulates simplest walker descending down a ramp.
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
parms.gam = 0.009;

%%%%% Initial State %%%%%
%%%% This guess gives the stable root %%%
q1 = 0.2; u1 = -0.2;
q2 = 2*q1; u2 = u1*(1-cos(2*q1));
%zstar = [0.200310900563820  -0.199832473025474   0.400621801127640  -0.015822999949030];
%t = 3.882

%%% This guess gives the unstable root. %%%
%q1 = 0.19; u1 = -0.2;
%q2 = 2*q1; u2 = u1*(1-cos(2*q1));
%zstar = [0.193937369810188  -0.203866927442012   0.387874739620375  -0.015144260853193];
%t = 3.096

steps = 1; %number of steps to animate
fps = 30; %frames per second

z0 = [q1 u1 q2 u2];


%%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-12,'TolX',1e-12,'Display','off');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,parms);
if exitflag == 1
    disp('Fixed points are');
    zstar
else
    %disp('fsolve not converged');
    error('Root finder not converged, change initial guess')
end

%%% Stability, using linearised eigenvalue %%%
disp('EigenValues for linearized map (finite difference) are');
J=partialder(@onestep,zstar,parms);
eig(J)
%abs(eig(J))

disp('EigenValues for linearized map (monodromy matrix) are using hand calculations');
J=monodromy(zstar,parms);
eig(J)
%abs(eig(J))

%%%%% Get data for all the steps %%%
[z,t] = onestep(zstar,parms,steps);

%%%% Animate result %%%
disp('Animating...');
disp('NOTE: Animation speed can be changed using fps defined in the code');
animate(t,z,parms,steps,fps);
 
%%%% Plot data %%%
disp('Some plots...')

subplot(2,1,1);
plot(t,z(:,1),'r',t,z(:,3),'b'); hold on;
title('State variables for Simplest Walkers');
ylabel('Angle (rad)');
legend('stance','swing','Location','Best'); 
subplot(2,1,2);
plot(t,z(:,2),'r',t,z(:,4),'b');
xlabel('time'); ylabel('Angular Rate (rad/s)');


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
    options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision);
    tspan = linspace(t0,t0+dt,1000);
    [t_temp, z_temp] = ode113(@single_stance,tspan,z0,options,parms);
    
    zplus=heelstrike(t_temp(end),z_temp(end,:),parms); 
    
    z0 = zplus;
    t0 = t_temp(end);
   
    %post-processd data for storing
    xhtemp = sin(z_temp(:,1))+x_trans; %x of wheel
    yhtemp = cos(z_temp(:,1));    %y of wheel
    
    x_trans = x_trans + (xhtemp(end)-xhtemp(1));
    
    x0temp = sin(z0(1,1))+x_trans;       
    y0temp = cos(z0(1,1));
    
    
    t_ode = [t_ode; t_temp(2:end); t0];
    
    z_ode = [z_ode;  ... 
             z_temp(2:end,:),xhtemp(2:end,1),yhtemp(2:end,1); ... 
             z0,x0temp,y0temp];
    
end

z = zplus(1:4);

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=single_stance(t,z,parms)  
%===================================================================

q1 = z(1);   u1 = z(2); 
q2 = z(3);   u2 = z(4); 

f = [u1 sin(q1-parms.gam) u2 sin(q1-parms.gam)+(u1^2)*sin(q2)-cos(q1-parms.gam)*sin(q2)]';
zdot = f;

%===================================================================
function [zplus,gx]=heelstrike(t,zminus,parms)      
%===================================================================

r1 = zminus(1);   v1 = zminus(2);
g = [-r1 cos(2*r1)*v1 -2*r1 cos(2*r1)*(1-cos(2*r1))*v1];

zplus = g;

gx = [-1 0 0 0; ...
      -2*sin(2*r1)*v1 cos(2*r1) 0 0; ...
      -2 0 0 0; ...
       (2*cos(2*r1)-1)*v1*2*sin(2*r1) cos(2*r1)*(1-cos(2*r1)) 0 0];


%===================================================================
function [h, isterminal,direction,hx]=collision(t,x,parms)
%===================================================================

q1 = x(1); q2 = x(3);

h = q2-2*q1;
if (q2>-0.05) %no collision detection for foot scuffing
    isterminal = 0;
else
    isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
end
direction=1; % The t_final can be approached by any direction is indicated by this

hx =  [-2 0 1 0];

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

F0 = eye(4);
F0vec = reshape(F0,1,16);
x0 = [zstar(1:4), F0vec(1:16)];

t0 = 0; dt = 5;
options=odeset('abstol',2.23e-13,'reltol',2.23e-13,'events',@collision);
tspan = linspace(t0,t0+dt,100);
[tt, xx] = ode113(@sensitivity,tspan,x0,options,parms);

F = reshape(xx(end,5:end),4,4);

FF = sensitivity(tt(end),xx(end,:),parms);
f = FF(1:4); %get first 4 elements of RHS 

[h, isterminal,direction,hx]=collision(tt(end),xx(end,1:4),parms);
[zplus,gx] = heelstrike(tt(end),xx(end,1:4),parms);
J = F*(gx - (gx*f*hx)/(hx*f)); %monodromy matrix
  
%===================================================================
function dxdt = sensitivity(t,x,parms)
%===================================================================

q1 = x(1);   u1 = x(2); 
q2 = x(3);   u2 = x(4); 
F = reshape(x(5:end),4,4);

f = [u1 sin(q1-parms.gam) u2 sin(q1-parms.gam)+(u1^2)*sin(q2)-cos(q1-parms.gam)*sin(q2)]';
dfdx = [0 1 0 0; ...
        cos(q1-parms.gam) 0 0 0; ...
        0 0 0 1; ...
        cos(q1-parms.gam)+sin(q1-parms.gam)*sin(q2), 2*u1*sin(q2), u1*u1*cos(q2)-cos(q1-parms.gam)*cos(q2), 0]; 
dFdt = dfdx*F;
dFdtvec = reshape(dFdt,16,1);

dxdt = [f;dFdtvec];


%===================================================================
function animate(t_all,z_all,parms,steps,fps)
%===================================================================

%%%% First, get the unique values of states %%%%
[t_unique,index] = unique(t_all,'first');

z_unique = [];
for i = 1:length(index)
   z_unique = [z_unique; z_all(index(i),1),  z_all(index(i),3),  z_all(index(i),5), z_all(index(i),6)];
end

%%%% Second, interpolate linearly using fps %%%%%
[m,n] = size(z_unique);
t = linspace(0,t_unique(end),fps*steps);
for i=1:n
    z(:,i) = interp1(t_unique,z_unique(:,i),t);
end

%%%% Lastly, animate the results
clf
   
[mm,nn]=size(z);

min_xh = min(z(:,3)); max_xh = max(z(:,3)); 
dist_travelled = min_xh-max_xh;
camera_rate = dist_travelled/mm;

window_xmin = -0.75; window_xmax = 1.25;
window_ymin = -0.1; window_ymax = 1.1;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%% creat object for hinge %%%%%
hingepic=line('xdata',0,'ydata',0, 'marker','.','markersize',[20], ...
          'erase','xor','color','black');

   
%%%% create object for legs and feet %%%%
barref = [0 0; 0 -1]; %%% bar along negative y-axis
y = [0;-1]; %%% vector along negative y
O = [0; 0]; %%%% origin

%%%% legs in red %%%      
for p = 1:2
    barpic(p)=line('xdata',barref(1,:),'ydata',barref(2,:),'linewidth', 2, 'erase','xor','color','red');
end


%%%% create ramp %%%%
rampref=[min_xh-1 max_xh+1 ; 0 0];

ramppic=line('xdata',rampref(1,:),'ydata',rampref(2,:), ...
            'linewidth', 1,'color','black');
     

moviescaling = 1;                      % slow down factor
delay =floor(moviescaling); %delay per frame in .001 secs

for i=1:mm
   
   for j=1:100, log(1:delay*17); end %delay for graphics. 
	                                 %the number in this expression
									 %is machine dependent.
									 %The LOG is just something
									 %to keep the machine busy.

   q1 = z(i,1); q2 = z(i,2); 
   xh = z(i,3); yh = z(i,4);  

   window_xmin = window_xmin + camera_rate;
   window_xmax = window_xmax + camera_rate;
   axis('equal')
   axis([window_xmin window_xmax window_ymin window_ymax])

   %%% hinge coordinates
   hinge=[xh; yh];   
   
   %%% leg coordinates
        A = [q1 -(q2-q1)];

        for p = 1:2
            bar(:,:,p) = [hinge, hinge] + R(A(p))*barref;
            center(:,:,p) = hinge + R(A(p))*y; %%% center of each circle on foot 
        end
                       
    %%% animate now    
    set(hingepic,'xdata',hinge(1),'ydata',hinge(2));

    for p=1:2
        set(barpic(p),'xdata',bar(1,:,p),'ydata',bar(2,:,p));
    end
    
    drawnow  
  
end


%===================================================================
function rotation = R(A)
%===================================================================
rotation = [cos(A) -sin(A); sin(A) cos(A)];


