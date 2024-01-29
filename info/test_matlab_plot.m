clc
clear all
close all

PaperPosition = [-0.25 -0.1 8 6]; %location on printed page. rect = [left, bottom, width, height]
PaperSize = [7.25 5.8]; %[width height]
Fontsize = 12;
print_pdf = 1;
path = '/Users/pranav/Desktop/';
Linewidth = 2;

x = linspace(0,pi);
y = sin(x);

hh = figure(1);
plot(x,y,'b-.','LineWidth',Linewidth);
xlabel('x-title','Fontsize',Fontsize)
ylabel('y-title','Fontsize',Fontsize)

string = [path,'test_MATLAB_plot'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 