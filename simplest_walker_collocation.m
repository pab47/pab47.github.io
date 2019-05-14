function simplest_walker_collocation
% run simplest_walker_collocation by Panchajanya Karasani
% needs fmincon and tested on R2015a and R2017b
% Last edit: Nov 19, 2018
% 
% Using collocation method to solve for the passive solution to the simplest walker
% Garcia, M., Chatterjee, A., Ruina, A., & Coleman, M. (1998). 
% The simplest walking model: stability, complexity, and scaling. 
% Journal of biomechanical engineering, 120(2), 281-288.

clc
clear all
close all 

% declaring global variables 
global N; % N is the number of discrete points, not sure if this is need for this problem
global gamma; % the slope angle 


N = 400; %number of collocation points
gamma = 0.009; % slope angle
% declaring bounds of the problem, ever angle should be in radians 
theta_min = -(30/180)*pi; 
theta_max = (30/180)*pi;
phi_min = -(30/180)*pi;
phi_max = (30/180)*pi;

% angular velocity bounds , correct guesss? 
thetaDot_min = -(30/180)*pi;
thetaDot_max = (30/180)*pi; 
phiDot_min = -(30/180)*pi; 
phiDot_max = (30/180)*pi;  

%time bounds 
tf_min = 1;
tf_max = 10; 


% intializing optimization parameters
theta0 = (15/180)*pi*ones(1,N); % ?
phi0 = -(15/180)*pi*ones(1,N); % ?
thetaDot0 = ones(1,N); 
phiDot0 = ones(1,N);
tf0 = 3.7;

x0 = [theta0, phi0, thetaDot0, phiDot0, tf0];

options = optimset('MaxFunEvals',100000,'Display','iter');
%fromating bounds into the required format for fmincon
LB = [theta_min*ones(1,N) phi_min*ones(1,N) thetaDot_min*ones(1,N) phiDot_min*ones(1,N) tf_min ]; % ????
UB = [theta_max*ones(1,N) phi_max*ones(1,N) thetaDot_max*ones(1,N) phiDot_max*ones(1,N) tf_max ]; % ????

% solving using fmincon 
[x_sol,FVAL,EXITFLAG,OUTPUT] = fmincon(@cost,x0,[],[],[],[],LB,UB,@constraints,options); % solving 
EXITFLAG

% outputing solution 

thetaS  = x_sol(1,N)
phiS = x_sol(1,2*N)
thetaDotS = x_sol(1,3*N)
phiDotS = x_sol(1,4*N)
tS = x_sol(1,4*N+1)

k = linspace(0,tS,N);

q = x_sol(1:N) % theta solution 

w = x_sol(N+1:2*N) % phi solution 

% graphical soulution 
figure(1)
plot(k,q);% theta
hold on; 
plot(k,w)% phi
legend('theta','phi');

% animation

figure(2)

h = animatedline;
o= animatedline;
r = animatedline;
l = 1; % length of the legs 

hold on;

    hc = h.Color;
    h.Color = 'red';
    oc = o.Color;
    o.Color = 'blue';
    rc = o.Color;
    r.Color = 'black';
  
    
for i = 1:length(q);
    
    xpos1(i) = -l*sin(q(i));
    
    ypos1(i) = l*cos(q(i));
    
    xpos2(i) = l*(sin(q(i) - w(i))) +xpos1(i); %- l*sin(q(i));
    
    ypos2(i) = -l*(cos(q(i) - w(i))) + ypos1(i);% - l*cos(q(i));
    

    jj = linspace(0,xpos1(i));
    kk = linspace(0,ypos1(i));
    nn = linspace(xpos1(i),xpos2(i));
    ll = linspace(ypos1(i),ypos2(i));
    k = linspace(-1,2,30);
    m = zeros(1,30);
    
    
    
    addpoints(h,jj,kk);   axis('equal');
    addpoints(o,nn,ll);    axis('equal');
    addpoints(r,k,m);
    
    xlim([-1-0.5 1+0.5]);
    ylim([-0.5 2+0.5]);
    drawnow limitrate;
    
    
   if(i<length(q-1))
    clearpoints(o);
    clearpoints(h);
   end

   
end

end

% constraints for walker 
function [c,ceq] = constraints(x)

global N; % N is the number of discrete points, not sure if this is need for this problem
global gamma; % the slope angle 

theta = x(1:N);
phi = x(N+1:2*N);
thetaDot = x(2*N+1:3*N);
phiDot = x(3*N+1:4*N);
tf = x(4*N+1); 

dt = tf/N;

for i=1:N-1
    
 defect_theta(i) =  thetaDot(i)*dt + theta(i) - theta(i+1)  ; % dt is temrs of time, i is in terms of steps; 
 
 defect_thetaDot(i) = sin(theta(i)-gamma)*dt -thetaDot(i+1) + thetaDot(i) ;
 
 defect_phiDot(i) = (sin(theta(i) -gamma) + (thetaDot(i).^2 - cos(theta(i) - gamma))*(sin(phi(i))))*dt - phiDot(i+1) + phiDot(i) ;
 
 defect_phi(i) = phiDot(i)*dt + phi(i) - phi(i+1);
 
end

c = []; %no inequality constraints

ceq = double( [ defect_theta defect_thetaDot defect_phi defect_phiDot theta(1)+theta(end) phi(1)+phi(end) thetaDot(1)-((cos(2*theta(end)))*(thetaDot(end))) phiDot(1)-((1-cos(2*(theta(end))))*(cos(2*(theta(end))))*(thetaDot(end))) (phi(1)-2*theta(1)) (phi(end)-(2*theta(end)))]);

end

% cost walker : no cost for this problem 
function score = cost(x)  
 score = 0;
end
 
 


