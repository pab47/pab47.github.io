function car_game
clear all
%Modified the pong code by David Buckingham
%https://www.mathworks.com/matlabcentral/fileexchange/31177-dave-s-matlab-pong

%%%%%% main part of the code %%%
global game_over
global t


close all
initData  %first function, initialize the data variables
initFigure %second function, initialize the figure

while ~game_over %runs till game_over = 1
    moveCar; %second function, compute car movement including collision detection
    refreshPlot; %fourth function, refresh plot based on moveCar
t = toc

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initData %first function, initialize the data variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global SPEED_MAX SPEED_MIN
global DIRECTION_MAX DIRECTION_MIN
global speed direction x y theta
global game_over level
global DT DELAY
global SPEED_INIT DIRECTION_INIT
global R r b
global obstacle_R1 obstacle_x1 obstacle_y1
global obstacle_R2 obstacle_x2 obstacle_y2
global obstacle_R3 obstacle_x3 obstacle_y3
global obstacle_R4 obstacle_x4 obstacle_y4
global obstacle_R5 obstacle_x5 obstacle_y5
global obstacle_R6 obstacle_x6 obstacle_y6
global obstacle_R7 obstacle_x7 obstacle_y7
global obstacle_R8 obstacle_x8 obstacle_y8
global obstacle_R9 obstacle_x9 obstacle_y9
global obstacle_R10 obstacle_x10 obstacle_y10
global obstacle_R11 obstacle_x11 obstacle_y11
global obstacle_R12 obstacle_x12 obstacle_y12
global obstacle_R13 obstacle_x13 obstacle_y13
global start_time

level = 0;
game_over = 0;
WALL_X_MIN = -3;
WALL_X_MAX = 3;
WALL_Y_MIN = -3;
WALL_Y_MAX = 3;

DIRECTION_MAX =  0.5;
DIRECTION_MIN = -0.5;
SPEED_MAX = 10;
SPEED_MIN = 5;

DT = 0.1; 
DELAY = 0.001;
SPEED_INIT = SPEED_MAX; 
DIRECTION_INIT = 0; 

%%%%%% initialize car settings %%%%%%
speed = 10;
direction = 0;
x = 1.5; 
y = -1.5; 
theta = pi/2;

%%%%%% initialize obstacle settings %%%%%%
obstacle_R1 = 0.5;
obstacle_x1 = 0;
obstacle_y1 = 0;

obstacle_R2 = 0.65;
obstacle_x2 = -1.2;
obstacle_y2 = 0.8;

obstacle_R3 = 2;
obstacle_x3 = 3;
obstacle_y3 = 3;

obstacle_R4 = 0.5;
obstacle_x4 = 1.3;
obstacle_y4 = 0;

obstacle_R5 = 0.5;
obstacle_x5 = 0.75;
obstacle_y5 = 1.2;

obstacle_R6 = 0.4;
obstacle_x6 = 2.6;
obstacle_y6 = 0;

obstacle_R7 = 1.3;
obstacle_x7 = -1.2;
obstacle_y7 = 3;

obstacle_R8 = 0.5;
obstacle_x8 = 2.2;
obstacle_y8 = -1.2;

obstacle_R9 = 0.55;
obstacle_x9 = 0;
obstacle_y9 = -1.3;

obstacle_R10 = 0.5;
obstacle_x10 = -0.8;
obstacle_y10 = -2.3;

obstacle_R11 = 0.45;
obstacle_x11 = -2.5;
obstacle_y11 = 0;

obstacle_R12 = 0.55;
obstacle_x12 = 1.2;
obstacle_y12 = -2.3;

obstacle_R13 = 0.5;
obstacle_x13 = -1.2;
obstacle_y13 = -0.7;
r = 0.01; %radius of wheels
b = 0.05; %width of the robot
R = 0.1; %foot print of the robot
tic;
start_time;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initFigure %second function, initialize the figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global x y theta
global car_anim dir_anim 
global obstacle_R1 obstacle_x1 obstacle_y1
global obstacle_R2 obstacle_x2 obstacle_y2
global obstacle_R3 obstacle_x3 obstacle_y3
global obstacle_R4 obstacle_x4 obstacle_y4
global obstacle_R5 obstacle_x5 obstacle_y5
global obstacle_R6 obstacle_x6 obstacle_y6
global obstacle_R7 obstacle_x7 obstacle_y7
global obstacle_R8 obstacle_x8 obstacle_y8
global obstacle_R9 obstacle_x9 obstacle_y9
global obstacle_R10 obstacle_x10 obstacle_y10
global obstacle_R11 obstacle_x11 obstacle_y11
global obstacle_R12 obstacle_x12 obstacle_y12
global obstacle_R13 obstacle_x13 obstacle_y13
global t

fig = figure; %initialize the figure
set(fig, 'Resize', 'off'); %do not allow figure to resize
set(fig,'KeyPressFcn',@keyDown); %setkey presses for later
axis('equal');
axis([WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX]); %set the size of the board.
axis manual;
grid on;
hold on;
title(['Survive more than 30 seconds ; Time: ' ,num2str(round(t,2)) ],'Fontsize',15);
set(gca, 'color', 'w', 'YTick', [], 'XTick', []); %remove x and y label
        
[x_robot,y_robot,x_dir,y_dir] = car_coordinates(x,y,theta);

%%%%%%% set the car %%%%%%
light_blue = [255,128,114]/255; %from https://www.rapidtables.com/web/color/blue-color.html
car_anim= patch(x_robot,y_robot,light_blue);  
dir_anim = line(x_dir,y_dir,'Color','black','Linewidth',2);
         
%%%%%%%% set the walls %%%%%%%
line('Xdata',[WALL_X_MIN WALL_X_MIN],'Ydata',[WALL_Y_MIN WALL_Y_MAX],'Color','k','Linewidth',3); %left wall
line('Xdata',[WALL_X_MIN WALL_X_MAX],'Ydata',[WALL_Y_MAX WALL_Y_MAX],'Color','k','Linewidth',3); %top wall
line('Xdata',[WALL_X_MAX WALL_X_MAX],'Ydata',[WALL_Y_MIN WALL_Y_MAX],'Color','k','Linewidth',3); %right wall
line('Xdata',[WALL_X_MIN WALL_X_MAX],'Ydata',[WALL_Y_MIN WALL_Y_MIN],'Color','k','Linewidth',3); %bottom wall

%%%%%% set the obstacles %%%%%
phi = linspace(0,2*pi);
light_red = [135 206 250]/255;%from https://www.rapidtables.com/web/color/RGB_Color.html

patch(obstacle_x1+obstacle_R1*cos(phi),obstacle_y1+obstacle_R1*sin(phi),light_red);
patch(obstacle_x2+obstacle_R2*cos(phi),obstacle_y2+obstacle_R2*sin(phi),light_red);
patch(obstacle_x3+obstacle_R3*cos(phi),obstacle_y3+obstacle_R3*sin(phi),light_red);
patch(obstacle_x4+obstacle_R4*cos(phi),obstacle_y4+obstacle_R4*sin(phi),light_red);
patch(obstacle_x5+obstacle_R5*cos(phi),obstacle_y5+obstacle_R5*sin(phi),light_red);
patch(obstacle_x6+obstacle_R6*cos(phi),obstacle_y6+obstacle_R6*sin(phi),light_red);
patch(obstacle_x7+obstacle_R7*cos(phi),obstacle_y7+obstacle_R7*sin(phi),light_red);
patch(obstacle_x8+obstacle_R8*cos(phi),obstacle_y8+obstacle_R8*sin(phi),light_red);
patch(obstacle_x9+obstacle_R9*cos(phi),obstacle_y9+obstacle_R9*sin(phi),light_red);
patch(obstacle_x10+obstacle_R10*cos(phi),obstacle_y10+obstacle_R10*sin(phi),light_red);
patch(obstacle_x11+obstacle_R11*cos(phi),obstacle_y11+obstacle_R11*sin(phi),light_red);
patch(obstacle_x12+obstacle_R12*cos(phi),obstacle_y12+obstacle_R12*sin(phi),light_red);
patch(obstacle_x13+obstacle_R13*cos(phi),obstacle_y13+obstacle_R13*sin(phi),light_red);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function moveCar %third function, compute car movement including collision detection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global game_over DT
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global b r R
global speed direction x y theta
global obstacle_R1 obstacle_x1 obstacle_y1
global obstacle_R2 obstacle_x2 obstacle_y2
global obstacle_R3 obstacle_x3 obstacle_y3
global obstacle_R4 obstacle_x4 obstacle_y4
global obstacle_R5 obstacle_x5 obstacle_y5
global obstacle_R6 obstacle_x6 obstacle_y6
global obstacle_R7 obstacle_x7 obstacle_y7
global obstacle_R8 obstacle_x8 obstacle_y8
global obstacle_R9 obstacle_x9 obstacle_y9
global obstacle_R10 obstacle_x10 obstacle_y10
global obstacle_R11 obstacle_x11 obstacle_y11
global obstacle_R12 obstacle_x12 obstacle_y12
global obstacle_R13 obstacle_x13 obstacle_y13
global t
rot = [cos(theta) -sin(theta) 0; ...
     sin(theta) cos(theta) 0; ...
     0 0 1];
vel_local = [0.5*r*speed; ...
             0; ...
     0.5*(r/b)*direction];
vel_global = rot*vel_local;

x = x + vel_global(1,1)*DT;
y = y + vel_global(2,1)*DT;
theta = theta + vel_global(3,1)*DT;

%%%%%% set all conditions for game to end %%%%%
dist_robot_obstacle1 = norm([obstacle_x1-x obstacle_y1-y]);
dist_robot_obstacle2 = norm([obstacle_x2-x obstacle_y2-y]);
dist_robot_obstacle3 = norm([obstacle_x3-x obstacle_y3-y]);
dist_robot_obstacle4 = norm([obstacle_x4-x obstacle_y4-y]);
dist_robot_obstacle5 = norm([obstacle_x5-x obstacle_y5-y]);
dist_robot_obstacle6 = norm([obstacle_x6-x obstacle_y6-y]);
dist_robot_obstacle7 = norm([obstacle_x7-x obstacle_y7-y]);
dist_robot_obstacle8 = norm([obstacle_x8-x obstacle_y8-y]);
dist_robot_obstacle9 = norm([obstacle_x9-x obstacle_y9-y]);
dist_robot_obstacle10 = norm([obstacle_x10-x obstacle_y10-y]);
dist_robot_obstacle11 = norm([obstacle_x11-x obstacle_y11-y]);
dist_robot_obstacle12 = norm([obstacle_x12-x obstacle_y12-y]);
dist_robot_obstacle13 = norm([obstacle_x13-x obstacle_y13-y]);
if (x<WALL_X_MIN+R || x>WALL_X_MAX-R || ...
    y<WALL_Y_MIN+R || y>WALL_Y_MAX-R || ...
    dist_robot_obstacle1 <= (obstacle_R1+R)||...
    dist_robot_obstacle2 <= (obstacle_R2+R)||...
    dist_robot_obstacle3 <= (obstacle_R3+R)||...
    dist_robot_obstacle4 <= (obstacle_R4+R)||...
    dist_robot_obstacle5 <= (obstacle_R5+R)||...
    dist_robot_obstacle6 <= (obstacle_R6+R)||...
    dist_robot_obstacle7 <= (obstacle_R7+R)||...
    dist_robot_obstacle8 <= (obstacle_R8+R)||...
    dist_robot_obstacle9 <= (obstacle_R9+R)||...
    dist_robot_obstacle10 <= (obstacle_R10+R)||...
    dist_robot_obstacle11 <= (obstacle_R11+R)||...
    dist_robot_obstacle12 <= (obstacle_R12+R)||...
    dist_robot_obstacle13 <= (obstacle_R13+R))
    game_over = 1;
elseif ( t > 30 )
    game_over = 2;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function refreshPlot %fifth function, refresh plot based on moveCar
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global DELAY
global car_anim dir_anim
global x y theta
global game_over
global t 
[x_robot,y_robot,x_dir,y_dir] = car_coordinates(x,y,theta);

set(car_anim,'XData', x_robot,'YData',y_robot);
set(dir_anim,'XData', x_dir,'YData',y_dir);

drawnow;
pause(DELAY);  

title(['Survive more than 30 seconds ; Time: ' ,num2str(round(t,1)) ],'Fontsize',15);
if (game_over==1)
    text(mean([WALL_X_MIN-4.7 WALL_X_MAX]),mean([WALL_Y_MIN WALL_Y_MAX]),'YOU LOSE ~~~','Fontsize',30);
elseif (game_over==2)
    text(mean([WALL_X_MIN-4.7 WALL_X_MAX]),mean([WALL_Y_MIN WALL_Y_MAX]),'YOU WIN !!!!','Fontsize',30);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function keyDown(src,event) %called from initFigure to take key press to move the paddle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global direction speed
global SPEED_MAX SPEED_MIN
global DIRECTION_MAX DIRECTION_MIN

switch event.Key
  case 'rightarrow'
    direction = direction-0.5;
    if (direction<DIRECTION_MIN)
        direction = DIRECTION_MIN;
    end
       
  case 'leftarrow'
    direction = direction+0.5;
    if (direction>DIRECTION_MAX)
        direction = DIRECTION_MAX;
    end
end

switch event.Key
  case 'uparrow'
    speed = speed+5;
    if (speed>SPEED_MAX)
        speed = SPEED_MAX;
    end

  case 'downarrow'
    speed = speed-2;
    if (speed<SPEED_MIN)
        speed = SPEED_MIN;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_robot,y_robot,x_dir,y_dir] = car_coordinates(x,y,theta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global R 

phi = linspace(0,2*pi);
x_circle = R*cos(phi);
y_circle = R*sin(phi);

x_robot = x + x_circle;
y_robot = y + y_circle;
x_dir = x + [0 R*cos(theta)];
y_dir = y + [0 R*sin(theta)];
        
