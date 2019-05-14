clc
clear all
close all

figure(1)
y_sun = [linspace(-1,1) linspace(1, -1)];
x_sun = zeros(1,length(y_sun));
for i=1:length(y_sun)
    pause(0.01);
    plot(x_sun(i),y_sun(i),'yo','Markersize',80,'MarkerFaceColor','y');
    x = [-2 -2 2 2];
    y = [0 -2 -2 0];
    patch(x,y,[165/255,42/255,42/255])
    axis([-2 2 -2 2]);
end

