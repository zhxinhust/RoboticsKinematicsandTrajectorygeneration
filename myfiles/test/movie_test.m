% clear all
% clc
% x=0:pi/50:2*pi;
% y=sin(x);
% plot(x,y)
% h=line(0,0,'color','r','marker','.','markersize',40);
% axesValue=axis;
% A(1:length(x))=struct('cdata',[],'colormap',[]);
% for jj=1:length(x)
%     set(h,'xdata',x(jj),'ydata',y(jj));
%     axis(axesValue);
%     drawnow
% %     A(jj)=getframe;
% end
% % movie(A,1)


clear all
clc
t=-10*pi:pi/250:10*pi;
comet3((cos(2*t).^2).*sin(t),(sin(2*t).^2).*cos(t),t);