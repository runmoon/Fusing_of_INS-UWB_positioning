% 绘制基站及定位区域图---------------------

x = [0 2.598 0 0];
y = [0 1.5 3 0];
plot(x,y,'--');
axis([-1 4 -1 4]);
text(x(1),y(1),'o Beacon1(0, 0)','color','r');
text(x(2),y(2),'o Beacon2(2.598, 1.5)','color','r');
text(x(3),y(3),'o Beacon3(0, 3)','color','r');
text(1,2,'* (0, 3)','color','b');
% grid on;
xlabel('x轴');
ylabel('y轴');
