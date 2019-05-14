% PASSIVEWALKER simulates passive ramp walkers.
% passivewalker(1) - simplest walker as a limiting case of a passive walker (Garcia et. al 1998)
% passivewalker(2) - more general passive walker with round feet.
% 
% Needs ODE113, FSOLVE, INTERP1. 
% If you find bugs in this code please mail, 
% Pranav A. Bhounsule, pab47@cornell.edu
% Last updated: 26 December 2009


function passivewalker(flag)  

clc
clear all
close all
format long

if nargin == 0
    flag = 1; %simulates simplest walker by default
end

if flag == 1
    %% Garcia's simplest walker with roots for Validation
    %%% Dimensions %%
    %% c = COM on the leg from hip, w = COM fore-aft offset, r = radius of feet
    %% M = hip mass, m = leg mass, I = leg inertia, l = leg length
    %%%%% To get results close to Garcia's walker increase M %%%%%%
    walker.M = 1000; walker.m = 1.0; walker.I = 0.00; walker.l = 1.0; walker.w = 0.0; 
    walker.c = 1.0;  walker.r = 0.0; walker.g = 1.0; walker.gam = 0.009; 
    

    %%%% Initial State %%%%%
    q1 = 0.2; u1 = -0.2;
    q2 = 0.4; u2 = -0.3;

    z0 = [q1 u1 q2 u2];
    %%% Root finding will give this stable root 
    %zstar = [0.200161072169750  -0.199906060087682   0.400322144339512  -0.015805473227965];

else 
    %%  More General round feet walker with roots
    %%%% Dimensions %%
    %% c = COM on the leg from hip, w = COM fore-aft offset, r = radius of feet
    %% M = hip mass, m = leg mass, I = leg inertia, l = leg length
    walker.M = 1.0; walker.m = 0.5; walker.I = 0.02; walker.l = 1.0; walker.w = 0.0; 
    walker.c = 0.5; walker.r = 0.2; walker.g = 1.0; walker.gam = 0.01; 
    
    %%%% Initial State %%%%%
    q1 = 0.2; u1 = -0.3;
    q2 = 0.4; u2 = -0.3;
    
    z0 = [q1 u1 q2 u2];
    %% Root finding will give this stable root 
    %zstar = [0.189472782205104  -0.239124222551699   0.378945564410209  -0.053691703909393];
    %%
end

%%%%%%%%%%%%%%%%%%%%%%%%%
steps = 10; %number of steps to animate
fps = 10; %Use low frames per second for low gravity


%%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-12,'TolX',1e-12,'Display','off');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,walker);
if exitflag == 1
    disp('Fixed point:');
    disp(zstar);
else
    error('Root finder not converged, change guess or change system parameters')
end


%%% Stability, using eigenvalues of Poincare map %%%
J=partialder(@onestep,zstar,walker);
disp('EigenValues for linearized map are');
eig(J)
 
%%%% Get data for all the steps %%%
[z,t] = onestep(zstar,walker,steps);

%%% Animate result %%%
disp('Animating...');
animate(t,z,walker,steps,fps);

%%% Plot data %%%
disp('Some plots...')
plot(t,z(:,1),'r',t,z(:,3),'b')
xlabel('time'); ylabel('Angle (rad)');
legend('Stance Angle','Swing Angle');
title('State variables vs time for passive walker');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FUNCTIONS START HERE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================================================================
function zdiff=fixedpt(z0,walker)
%===================================================================
zdiff=onestep(z0,walker)-z0; 

%===================================================================
function J=partialder(FUN,z,walker)
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
    J(:,i)=(feval(FUN,ztemp1,walker)-feval(FUN,ztemp2,walker)) ;
end
J=J/(2*pert);

%===================================================================
function [z,t]=onestep(z0,walker,steps)
%===================================================================

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; w = walker.w;   
r = walker.r;  g = walker.g; gam = walker.gam;

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
end

q1 = z0(1);
u1 = z0(2);
q2 = z0(3);
u2 = z0(4);

    %%%% Derived variables %%%%
    TE = 1/2*m*(((-l*cos(q1)-r)*u1-u1*(-c*cos(q1)+w*sin(q1)))^2+(-l*sin(q1)*u1+u1*(c*sin(q1)+w*cos(q1)))^2)+1/2*m*(((-l*cos(q1)-r)*u1-(u1-u2)*(-c*cos(q1-q2)+w*sin(q1-q2)))^2+(-l*sin(q1)*u1+(u1-u2)*(c*sin(q1-q2)+w*cos(q1-q2)))^2)+1/2*M*((-l*cos(q1)-r)^2*u1^2+l^2*sin(q1)^2*u1^2)+1/2*I*(u1^2+(u1-u2)^2)+2*m*g*cos(gam)*r+2*m*g*l*cos(gam-q1)-m*g*c*cos(gam-q1)-m*g*w*sin(gam-q1)+2*m*g*sin(gam)*r*q1-m*g*c*cos(gam-q1+q2)-m*g*w*sin(gam-q1+q2)+M*g*cos(gam)*r+M*g*l*cos(gam-q1)+M*g*sin(gam)*r*q1; 
    xp1 = 0;
    xh = -l*sin(q1) - r*q1 + xp1;
    vxh = (-l*cos(q1)-r)*u1; 
    yh =  l*cos(q1) + r;
    vyh = -l*sin(q1)*u1; 
    
z0 = [q1 u1 q2 u2 TE xh vxh yh vyh];

t0 = 0; 
dt = 5; %might need to be changed based on time taken for one step
time_stamps = 100;
t_ode = t0;
z_ode = z0;

for i=1:steps
    options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp, z_temp] = ode113(@single_stance,tspan,z0,options,walker);
    
    zplus=heelstrike(t_temp(end),z_temp(end,:),walker); 
    
    z0 = zplus;
    t0 = t_temp(end);
    
    %%%%% Ignore time stamps for heelstrike and first integration point
    t_ode = [t_ode; t_temp(2:end)];
    z_ode = [z_ode; z_temp(2:end,:)];
    
end

z = zplus(1:4);

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=single_stance(t,z,walker)  
%===================================================================

q1 = z(1);   u1 = z(2);                         
q2 = z(3);   u2 = z(4);                         
xh = z(6);  vxh = z(7);                       
yh = z(8);  vyh = z(9);                     

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; w = walker.w;   
r = walker.r;  g = walker.g; gam = walker.gam;

Th=0;   %external hip torque, if needed               

M11 = -2*w^2*m-2*I+2*m*l*c*cos(q2)+2*m*w*l*sin(q2)-2*m*c^2-2*m*l^2-M*l^2+2*m*l*c-2*m*r^2-M*r^2+2*m*r*c*cos(q1-q2)-2*m*r*w*sin(q1-q2)-2*M*r*l*cos(q1)-4*m*r*l*cos(q1)+2*m*r*c*cos(q1)-2*m*r*w*sin(q1); 
M12 = w^2*m+I-m*l*c*cos(q2)-m*w*l*sin(q2)+m*c^2-m*r*c*cos(q1-q2)+m*r*w*sin(q1-q2); 

M21 = m*w*l*sin(q2)+m*l*c*cos(q2)-m*r*w*sin(q1-q2)+m*r*c*cos(q1-q2)-m*c^2-w^2*m-I; 
M22 = w^2*m+m*c^2+I; 

RHS1 = -2*m*r*u1*u2*c*sin(q1-q2)-2*m*r*u1*u2*w*cos(q1-q2)+m*r*u1^2*w*cos(q1)+m*r*u1^2*c*sin(q1)-2*m*r*l*sin(q1)*u1^2+M*g*sin(gam)*r+2*m*g*sin(gam)*r+m*r*u2^2*w*cos(q1-q2)+m*r*u2^2*c*sin(q1-q2)+m*r*u1^2*w*cos(q1-q2)+m*r*u1^2*c*sin(q1-q2)-M*r*l*sin(q1)*u1^2+M*g*l*sin(gam-q1)+2*m*g*l*sin(gam-q1)-m*g*c*sin(gam-q1)+m*g*w*cos(gam-q1)-m*g*c*sin(gam-q1+q2)+m*g*w*cos(gam-q1+q2)-2*m*l*u1*u2*w*cos(q2)-m*l*u2^2*c*sin(q2)+2*m*l*u1*u2*c*sin(q2)+m*l*u2^2*w*cos(q2); 
RHS2 = -m*g*c*sin(gam-q1+q2)+m*g*w*cos(gam-q1+q2)-Th-m*l*u1^2*w*cos(q2)+m*l*u1^2*c*sin(q2); 

MM = [M11 M12;                               
     M21 M22];                               

RHS = [RHS1; RHS2];                       

X = MM \ RHS;                                    

ud1 = X(1);                                       
ud2 = X(2);                                       

DTE = -ud1*I*u2+2*ud1*m*u1*r^2+m*u1*r*u2^2*c*sin(q1-q2)+m*u1*r*u2^2*w*cos(q1-q2)-m*u2^2*l*u1*c*sin(q2)+u2*m*g*c*sin(gam-q1+q2)-u2*m*g*w*cos(gam-q1+q2)+2*ud1*I*u1+ud2*I*u2+m*u2^2*l*u1*w*cos(q2)+2*ud1*m*u1*c^2+ud2*m*u2*c^2-ud2*I*u1+ud1*m*u2*c*l*cos(q2)+ud1*m*u2*w*l*sin(q2)-2*ud1*m*u1*l*c*cos(q2)-2*ud1*m*l*u1*w*sin(q2)-m*u2*u1^2*w*l*cos(q2)+m*u2*u1^2*c*l*sin(q2)+2*ud1*m*u1*w^2+ud2*m*u2*w^2+ud2*m*u1*l*c*cos(q2)+ud1*M*l^2*u1-ud2*m*u1*w^2-ud1*m*u2*c^2-ud2*m*u1*c^2+2*ud1*m*l^2*u1-ud1*m*u2*w^2-2*ud1*m*l*u1*c+2*ud1*M*l*cos(q1)*u1*r+4*ud1*m*l*cos(q1)*u1*r-2*ud1*m*u1*r*c*cos(q1)+2*ud1*m*u1*r*w*sin(q1)-2*ud1*m*u1*r*c*cos(q1-q2)-2*m*u1^3*r*l*sin(q1)+m*u1^3*r*c*sin(q1)+m*u1^3*r*w*cos(q1)+m*u1^3*r*c*sin(q1-q2)+m*u1^3*r*w*cos(q1-q2)-2*m*u1^2*r*u2*c*sin(q1-q2)-2*m*u1^2*r*u2*w*cos(q1-q2)-M*u1^3*r*l*sin(q1)+2*u1*m*g*l*sin(gam-q1)-u1*m*g*c*sin(gam-q1)+u1*m*g*w*cos(gam-q1)+2*u1*m*g*sin(gam)*r+ud2*m*l*u1*w*sin(q2)+ud1*M*u1*r^2-u1*m*g*c*sin(gam-q1+q2)+u1*m*g*w*cos(gam-q1+q2)+u1*M*g*l*sin(gam-q1)+u1*M*g*sin(gam)*r+2*ud1*m*u1*r*w*sin(q1-q2)+ud1*m*u2*c*cos(q1-q2)*r-ud1*m*u2*w*sin(q1-q2)*r+ud2*m*u1*r*c*cos(q1-q2)-ud2*m*u1*r*w*sin(q1-q2); 
axh = l*sin(q1)*u1^2+(-l*cos(q1)-r)*ud1; 
ayh = -l*cos(q1)*u1^2-l*sin(q1)*ud1; 

zdot = [u1 ud1 u2 ud2 ...           
        DTE vxh axh vyh ayh]';  

%===================================================================
function [gstop, isterminal,direction]=collision(t,z,walker)
%===================================================================

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; w = walker.w;   
r = walker.r;  g = walker.g; gam = walker.gam; 

q1 = z(1); q2 = z(3); 

gstop = -q2 + 2*q1;
if (q2>-0.05) %allow legs to pass through for small hip angles (taken care in real walker using stepping stones)
    isterminal = 0;
else
    isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
end
direction=-1; % The t_final can be approached by any direction is indicated by the direction

%===================================================================
function zplus=heelstrike(t,z,walker)      
%===================================================================

r1 = z(1);   v1 = z(2);                         
r2 = z(3);   v2 = z(4);                         
xh = z(6);   yh = z(8);                       

q1 = r1 - r2;                         
q2 = -r2;                                       

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; w = walker.w;   
r = walker.r;  g = walker.g; gam = walker.gam; 

M11 = 2*m*l^2-2*m*l*c+2*m*c^2+2*m*w^2+2*m*r^2+4*m*r*l*cos(q1)-2*m*r*c*cos(q1)+2*m*w*sin(q1)*r-2*m*l*c*cos(q2)-2*m*l*w*sin(q2)-2*m*r*c*cos(q1-q2)+2*m*sin(q1-q2)*w*r+M*l^2+2*M*r*l*cos(q1)+M*r^2+2*I; 
M12 = m*l*c*cos(q2)+m*l*w*sin(q2)-m*c^2-m*w^2+m*r*c*cos(q1-q2)-m*sin(q1-q2)*w*r-I; 

M21 = -m*l*c*cos(q2)-m*l*w*sin(q2)+m*c^2+m*w^2-m*r*c*cos(q1-q2)+m*sin(q1-q2)*w*r+I; 
M22 = -m*w^2-m*c^2-I; 

RHS1 = m*l*v2*c-m*c^2*v2+M*v1*r^2-2*m*c*l*v1+M*v1*l^2*cos(r2)+2*m*l^2*v1*cos(r2)+2*I*v1-I*v2+2*m*v1*r^2-2*m*l*v1*c*cos(r2)+2*m*c^2*v1+2*m*w^2*v1-m*w^2*v2+2*m*r*v1*w*sin(r1)+M*r*v1*l*cos(r1)+M*l*cos(-r1+r2)*v1*r-2*m*r*v1*c*cos(r1)+2*m*l*cos(-r1+r2)*v1*r+2*m*r*v1*l*cos(r1)-2*m*r*v1*c*cos(-r1+r2)-2*m*r*v1*w*sin(-r1+r2)+m*r*v2*c*cos(-r1+r2)+m*r*v2*w*sin(-r1+r2); 
RHS2 = m*r*v1*w*sin(r1)-m*r*v1*c*cos(r1)+I*v1-I*v2+m*w^2*v1-m*c*l*v1+m*c^2*v1; 

MM = [M11 M12;                               
     M21 M22];    

RHS = [RHS1; RHS2];                      

X = MM \ RHS;                                    

u1 = X(1);                                       
u2 = X(2);                                      

TE = 1/2*m*(((-l*cos(q1)-r)*u1-u1*(-c*cos(q1)+w*sin(q1)))^2+(-l*sin(q1)*u1+u1*(c*sin(q1)+w*cos(q1)))^2)+1/2*m*(((-l*cos(q1)-r)*u1-(u1-u2)*(-c*cos(q1-q2)+w*sin(q1-q2)))^2+(-l*sin(q1)*u1+(u1-u2)*(c*sin(q1-q2)+w*cos(q1-q2)))^2)+1/2*M*((-l*cos(q1)-r)^2*u1^2+l^2*sin(q1)^2*u1^2)+1/2*I*(u1^2+(u1-u2)^2)+2*g*m*cos(gam)*r+2*m*g*l*cos(gam-q1)-m*g*c*cos(gam-q1)-m*g*w*sin(gam-q1)+2*g*m*sin(gam)*r*q1-m*g*c*cos(gam-q1+q2)-m*g*w*sin(gam-q1+q2)+g*M*cos(gam)*r+M*g*l*cos(gam-q1)+g*M*sin(gam)*r*q1; 
vxh = (-l*cos(q1)-r)*u1; 
vyh = -l*sin(q1)*u1; 

zplus = [q1 u1 q2 u2 TE xh vxh yh vyh];                     


%===================================================================
function animate(t_all,z_all,walker,steps,fps)
%===================================================================

%%%% Interpolate linearly using fps %%%%%
z_all_plot = [z_all(:,1) z_all(:,3) z_all(:,6) z_all(:,8)];
nn = size(z_all_plot,2);
total_frames = round(t_all(end)*fps);
t = linspace(0,t_all(end),total_frames);
z = zeros(total_frames,nn);
for i=1:nn
    z(:,i) = interp1(t_all,z_all_plot(:,i),t);
end

%%%% Now animate the results %%%%%%%
clf

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; w = walker.w;   
r = walker.r;  g = walker.g; gam = walker.gam; 

mm = size(z,1);

min_xh = min(z(:,3)); max_xh = max(z(:,3)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -1*l; window_xmax = 1*l;
window_ymin = -0.1; window_ymax = 1.1*(l+r);

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%%%%%
lines_for_feet = 4; %no of straight lines to represent a foot.
counter = 2 + 2*lines_for_feet; % total number of segments needed for animation. 2 legs and other for feet
th = 0.25; %%%% angle subtended by each straight line segment of foot

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

%%% feet in blue %%%
for p = 3:counter
    barpic(p)=line('xdata',barref(1,:),'ydata',barref(2,:),'linewidth', 2, 'erase','xor','color','blue');
end

%%%% create ramp %%%%
rampref=[min_xh-1 max_xh+l ; 0 0];

%%%% Draw ramp %%%%%%%%%%
line('xdata',rampref(1,:),'ydata',rampref(2,:), ...
            'linewidth', 1,'color','black');
     

moviescaling = 1;                      % slow down factor
delay =floor(moviescaling); %delay per frame in .001 secs


for i=1:mm
    
%%%% Put some delay if needed %%%%%%%%%%   
%    for j=1:100, log(1:delay*17); end %delay for graphics. 
% 	                                 %the number in this expression
% 									 %is machine dependent.
% 									 %The LOG is just something
% 									 %to keep the machine busy.

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
            bar(:,:,p) = [hinge, hinge] + (l+r)*R(A(p))*barref;
            center(:,:,p) = hinge + l*R(A(p))*y; %%% center of each circle on foot 
        end
        
  %%%% feet coordinates
        
        %%% angle subtended by arc at the center %%%
        %%% This is for lines_for_feet = 4 and need changes if more lines
        %%% are needed to represent a foot. Size of this matrix is lines_for_feet*2
            B = [-th -2*th; 0 -th; th 0; 2*th th];
    
        incr = 3;
        for p=1:2
            for q=1:4
                   C = A(p) + B(q,:);
                   bar(:,:,incr) = [center(:,:,p), center(:,:,p)] + r*R(C(1))*[O,y] + r*R(C(2))*[y, O]; 
                   incr = incr + 1;
            end
        end
               
    %%% animate now    
    set(hingepic,'xdata',hinge(1),'ydata',hinge(2));

    for p=1:counter
        set(barpic(p),'xdata',bar(1,:,p),'ydata',bar(2,:,p));
    end
    
    drawnow  
  
end

%===================================================================
function rotation = R(A)
%===================================================================
rotation = [cos(A) -sin(A); sin(A) cos(A)];

