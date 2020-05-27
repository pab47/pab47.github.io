function SnakeGame
Lan = 20;
[GDX, GDY] = meshgrid(1:Lan,1:Lan);
GDD = [GDX(:) GDY(:)];
set(gcf,'keypressfcn',@KeyPress,'deletefcn',@Deletefunction)
set(gca,'color','b','xtick',[],'ytick',[])
axis('equal',0.5+[0 Lan 0 Lan])
hold on
siz = 250;
press = '';
pos = [5 5];
dic = [1 0];
tal = [];
lon = 2;
hhs = scatter(gca,pos(1),pos(2),siz,'wd','filled');
foo = [10 10];
hha = scatter(gca,foo(1),foo(2),siz,'ro','filled');
fps = 0.2;
gam = timer('ExecutionMode','fixedRate','Period',fps,'timerfcn',@gameplay);
start(gam)
function KeyPress(~,evt)
    press = evt.Key;
end
function gameplay(~,~)
    switch press 
        case 'uparrow'
            dictmp = [0 1];
        case 'downarrow'
            dictmp = [0 -1];
        case 'leftarrow'
            dictmp = [-1 0];
        case 'rightarrow'
            dictmp = [1 0];
        otherwise
            dictmp = nan;
    end
if any(dic + dictmp) 
	dic = dictmp;
end
pos = pos + dic;
pos(pos > Lan) = pos(pos > Lan) - Lan; 
pos(pos < 1) = pos(pos < 1) + Lan;
tal(end+1,:) = pos;
while length(tal)> lon
	tal(1,:) = [];
end
if intersect(tal(1:end-1,:),pos,'rows')
	a=lon-2;
	msgbox({'Points:',num2str(a)})
	close all;
	return
end
if isequal(pos(1,:),foo) 
	lon = lon + 1;
	posTmp = setdiff(GDD, tal, 'rows');
	foo = posTmp(randi(size(posTmp,1)),:); 
end 
if (lon==400)
    msgbox('You Finally Won!!!')
end
set(hha,'xdata',foo(1),'ydata',foo(2)) 
set(hhs,'xdata',tal(:,1),'ydata',tal(:,2));
title(['Points = ',num2str(lon-2)],'Fontsize',14);
end
function Deletefunction(~,~)
	if isvalid(gam)
      stop(gam)
      delete(gam)
	end
end
end
    
