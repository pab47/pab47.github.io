function raibert_hopper
% RAIBERT_HOPPER simulates spring mass hopper with Raibert style decoupled
% controller. Code by Chelsea Moussouni cnmoussouni@gmail.com

% Relevant paper: 
% Raibert, Marc H., H. Benjamin Brown, and Michael Chepponis. 
% "Experiments in balance with a 3D one-legged hopping machine." 
% The International Journal of Robotics Research 3.2 (1984): 75-92.
% 
% If you find bugs in this code please mail, 
% Pranav A. Bhounsule, pranav.bhounsule@utsa.edu
% Last updated: 16 December 2016
% Fixed gstop in the function takeoff, 23 June 2017

clc
clear all
close all

y = 1.2;            % initial y position (m)
xdot = 3;           % initial x velocity (m/s)
spring.K= 0.1;      % gain selected to maximize stability 
spring.k= 1.5e4;    % spring constant (N/m)
spring.m = 70;      % mass of 70 kg
spring.Ts=pi*sqrt(spring.m/spring.k); % previous stance time
spring.Vdes=xdot;   % parameter that influences forward velocity
tend = 10;           % end value of t (s)
t0 = 0;              % start value of t (s)
spring.g = 9.81;     % accel. due to gravity (m/s)
spring.fps = 30;     % frames per second
spring.pauseval=0.01; % pause value
spring.L0=1;         % original length 
spring.alpha_td=acosd(xdot*spring.Ts/(2*spring.L0)+spring.K*(xdot-spring.Vdes)/spring.L0); % degrees 
spring.alpha_to=spring.alpha_td; %angle of takeoff is equal to touchdown

q0 = [0 xdot y 0]; % initial conditions [x xdot y ydot]

spring.xtouchdown = q0(1)+spring.L0*cosd(spring.alpha_td); %alpha is angle with the horizontal

t_all = t0; % initialize time variable
q_all = [q0, q0(1)+spring.L0*cosd(spring.alpha_td), q0(3)-spring.L0*sind(spring.alpha_td)]; % initialize state space variables
while(t0<tend)
    t = linspace(t0,tend,(tend-t0)*100); % time variable
    
    % set tolerances, seek ground detection 
    options = odeset('Abstol',1e-6,'Reltol',1e-6,'Events',@ground_detection); 
    [t,q,TE,YE,IE]=ode45(@free_fall,t,q0,options,spring);
    if (IE==1)
        t_td=t(end);
        spring.xtouchdown = YE(1) + spring.L0*cosd(spring.alpha_td);    
    end
   
    bottom=[q(:,1)+spring.L0*cosd(spring.alpha_td),q(:,3)-spring.L0*sind(spring.alpha_td)]; % top is mass, [x y]

    % Storing t and q variables
    t_all = [t_all; t(2:end)]; 
    q_all = [q_all; q(2:end,:),bottom(2:end,:)];

    % Updating t0 and q0
     t0 = t(end);    
     q0 = [q(end,1) q(end,2), q(end,3) q(end,4)];
     
     if (t0>=tend)
        break;
     end
    
    % Seeking takeoff 
    t = linspace(t0,tend,(tend-t0)*100);
    options2 = odeset('Abstol',1e-6,'Reltol',1e-6,'Events',@takeoff);  
    [t,q,TE,YE,IE]=ode45(@touchdown,t,q0,options2,spring);
           
    bottom=[spring.xtouchdown*ones(length(q),1) 0*ones(length(q),1)]; % top is mass, [x y]
    
    t_all = [t_all; t(2:end)]; 
    q_all = [q_all; q(2:end,:),bottom(2:end,:)];
    
    % Updating initial conditions 
      t0 = t(end);
      q0 = [q(end,1) q(end,2), q(end,3) q(end,4)];

       
      % Determing xdot midstance
           xdot = q(end,2);
       
           %xdot
           value = (xdot*spring.Ts/(2*spring.L0))+(spring.K*(xdot-spring.Vdes)/spring.L0);
           if (value>1)
               value = 1;
           end
          spring.alpha_td=acosd(value);%+spring.K*(xdot-spring.Vdes)/spring.L0;
          spring.alpha_td;
          spring.alpha_to=spring.alpha_td;
end

tinterp = linspace(0,t_all(end),spring.fps*t(end));

[m,n] = size(q_all);
for i=1:n;
    qinterp(:,i) = interp1(t_all,q_all(:,i),tinterp);
end

figure(1) % animation of body
for i=1:length(tinterp)
    plot(qinterp(i,1),qinterp(i,3),'ro','MarkerEdgeColor','r',...
                       'MarkerFaceColor','r',...
                       'MarkerSize',10);
                   hold on
                   
    % draw leg
    top=[qinterp(i,1),qinterp(i,3)]; % top is mass, [x y]
    bottom = [qinterp(i,5) qinterp(i,6)];
    line([bottom(1) top(1)],[bottom(2) top(2)])
    
    plot(qinterp(1:i,1), qinterp(1:i,3),'k');
    axis([min(qinterp(:,1))-0.1 max(qinterp(:,1))+0.1 0 max(qinterp(:,3))+0.2]);
    xlabel('x'); ylabel('y');
    grid on;
    pause(spring.pauseval);
    hold off
end

figure(2) % x and y position vs. time
subplot(2,1,1);
plot(t_all,q_all(:,1));
grid on;
ylabel('x');
subplot(2,1,2);
plot(t_all,q_all(:,3),'r');
grid on;
xlabel('t'); ylabel('y');

function dqdt=free_fall(t,q,spring)
x2 = q(2);  % xdot
x4 = q(4);  % ydot

dqdt = [x2; 0; x4; -spring.g]; % xd, xdd, yd, ydd

function dqdt_td=touchdown(t,q,spring) % while body is in contact with ground
x = q(1);  % x
xdot = q(2); % xdot
y = q(3); % y
ydot = q(4); % ydot

L=sqrt((x-spring.xtouchdown)^2+y^2);
%L=sqrt((x-spring.xtouchdown)^2+y^2);
xddot = (spring.k/spring.m)*(spring.L0-L)*((x-spring.xtouchdown)/L);
yddot = ((spring.k/spring.m)*(spring.L0-L)*(y/L))-spring.g;

% q'
dqdt_td = [xdot; xddot; ydot; yddot];


function [value, isterminal, direction] = ground_detection(t,q,spring)
x1 = q(1);  %x
x2 = q(2);
x3 = q(3);  %y
x4 = q(4);

value =  x3 - spring.L0*sind(spring.alpha_td);
isterminal = 1; % 1 = stop integration, 0 = continue integration
direction = -1; % -1, 1, 0

function [value, isterminal, direction] = takeoff(t,q,spring)

x1 = q(1);  %x
x2 = q(2);
x3 = q(3);  %y
x4 = q(4);

%value = x3 - spring.L0*sind(spring.alpha_td); %incorrect
L=sqrt((x1-spring.xtouchdown)^2+x3^2);
value = L-spring.L0;
isterminal = 1; % 1 = stop integration, 0 = continue integration
direction = 1; % -1, 1, 0
