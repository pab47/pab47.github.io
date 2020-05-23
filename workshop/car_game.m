function car_game
%Modified the pong code by David Buckingham
%https://www.mathworks.com/matlabcentral/fileexchange/31177-dave-s-matlab-pong

%%%%%% main part of the code %%%
global game_over

close all
initData  %first function, initialize the data variables
initFigure %second function, initialize the figure
while ~game_over %runs till game_over = 1
    moveCar; %second function, compute car movement including collision detection
    refreshPlot; %fourth function, refresh plot based on moveCar
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
global obstacle_R1 obstacle_x1 obstacle_y1 %one obstacle
global obstacle_R2 obstacle_x2 obstacle_y2 %one obstacle
global obstacle1_visited
global clock_time

[clock_time tf] = clock;
obstacle1_visited = 0;
level = 0;
game_over = 0;
WALL_X_MIN = -2;
WALL_X_MAX = 2;
WALL_Y_MIN = -2;
WALL_Y_MAX = 2;

DIRECTION_MAX =  1.0;
DIRECTION_MIN = -1.0;
SPEED_MAX = 35;
SPEED_MIN = -35;

DT = 0.1; 
DELAY = 0.001;
SPEED_INIT = SPEED_MAX; 
DIRECTION_INIT = 0; 

%%%%%% initialize car settings %%%%%%
speed = 30;
direction = 0;
x = 1.5; 
y = -1.5; 
theta = pi/2;

%%%%%% initialize obstacle settings %%%%%%
obstacle_R1 = 0.25;
obstacle_x1 = 1;
obstacle_y1 = 0.5;

obstacle_R2 = 0.5;
obstacle_x2 = -1;
obstacle_y2 = 1;

r = 0.01; %radius of wheels
b = 0.05; %width of the robot
R = 0.1; %foot print of the robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initFigure %second function, initialize the figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global x y theta
global car_anim dir_anim 
global level
global obstacle_R1 obstacle_x1 obstacle_y1
global obstacle_R2 obstacle_x2 obstacle_y2


fig = figure; %initialize the figure
set(fig, 'Resize', 'off'); %do not allow figure to resize
set(fig,'KeyPressFcn',@keyDown); %setkey presses for later
axis('equal');
axis([WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX]); %set the size of the board.
axis manual;
grid on;
hold on;
title(['directon change <left/right>; speed change <up/down> ; score = ',num2str(level)],'Fontsize',14);
%set(gca, 'color', 'w', 'YTick', [], 'XTick', []); %remove x and y label
        
[x_robot,y_robot,x_dir,y_dir] = car_coordinates(x,y,theta);

%%%%%%% set the car %%%%%%
light_blue = [176,224,230]/255; %from https://www.rapidtables.com/web/color/blue-color.html
car_anim= patch(x_robot,y_robot,light_blue);  
dir_anim = line(x_dir,y_dir,'Color','black','Linewidth',2);
         
%%%%%%%% set the walls %%%%%%%
line('Xdata',[WALL_X_MIN WALL_X_MIN],'Ydata',[WALL_Y_MIN WALL_Y_MAX],'Color','k','Linewidth',3); %left wall
line('Xdata',[WALL_X_MIN WALL_X_MAX],'Ydata',[WALL_Y_MAX WALL_Y_MAX],'Color','k','Linewidth',3); %top wall
line('Xdata',[WALL_X_MAX WALL_X_MAX],'Ydata',[WALL_Y_MIN WALL_Y_MAX],'Color','k','Linewidth',3); %right wall
line('Xdata',[WALL_X_MIN WALL_X_MAX],'Ydata',[WALL_Y_MIN WALL_Y_MIN],'Color','k','Linewidth',3); %bottom wall

%%%%%% set the obstacles %%%%%
phi = linspace(0,2*pi);
light_red = [253 197 197]/255; %from https://www.rapidtables.com/web/color/RGB_Color.html
patch(obstacle_x1+obstacle_R1*cos(phi),obstacle_y1+obstacle_R1*sin(phi),light_red);
patch(obstacle_x2+obstacle_R2*cos(phi),obstacle_y2+obstacle_R2*sin(phi),'r');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function moveCar %third function, compute car movement including collision detection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global game_over DT
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global b r R
global speed direction x y theta
global obstacle_R1 obstacle_x1 obstacle_y1
global obstacle_R2 obstacle_x2 obstacle_y2
global level
global obstacle1_visited
global clock_time

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

old_time = clock_time;
[new_time tf] = clock;
time_elapsed = new_time(6) - old_time(6);
disp(time_elapsed)
%%%%%% set all conditions for game to end %%%%%
dist_robot_obstacle1 = norm([obstacle_x1-x obstacle_y1-y]); %distance between 
dist_robot_obstacle2 = norm([obstacle_x2-x obstacle_y2-y]); %distance between 
                                                            %2 points
if ( dist_robot_obstacle1 <= (obstacle_R1+R) && obstacle1_visited==0)
    obstacle1_visited=1;
    level = level+1;
    title(['directon change <left/right>; speed change <up/down> ; score = ',num2str(level)],'Fontsize',14);
end

if (x<WALL_X_MIN+R || x>WALL_X_MAX-R || ...
    y<WALL_Y_MIN+R || y>WALL_Y_MAX-R || ...
    dist_robot_obstacle2 <= (obstacle_R2+R) || ...
    time_elapsed > 20)
    game_over = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function refreshPlot %fifth function, refresh plot based on moveCar
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global WALL_X_MIN WALL_X_MAX WALL_Y_MIN WALL_Y_MAX
global DELAY
global car_anim dir_anim
global x y theta
global game_over

[x_robot,y_robot,x_dir,y_dir] = car_coordinates(x,y,theta);

set(car_anim,'XData', x_robot,'YData',y_robot);
set(dir_anim,'XData', x_dir,'YData',y_dir);

drawnow;
pause(DELAY);  

if (game_over)
    text(mean([WALL_X_MIN WALL_X_MAX]),mean([WALL_Y_MIN WALL_Y_MAX]),'GAME OVER','Fontsize',14);
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
    speed = speed-5;
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
        

