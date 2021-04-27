function Display_axis( a )

figure(2);
a(:,2) = a(:,2) * 0.5;

temp1 = 0.5 * (a(:,1) + a(:,2));  % x-y矩形中心坐标
temp2 =  0.5 * (2*temp1 + a(:,1));  % 三角形顶点坐标

plot3([0 a(1,1)],[0 a(2,1)],[0 a(3,1)],'color','r');  % x轴
text(a(1,1),a(2,1),a(3,1),'x');
hold on;
plot3([0 a(1,2)],[0 a(2,2)],[0 a(3,2)],'color','c');  % y轴
text(a(1,2),a(2,2),a(3,2),'y');

plot3([0 a(1,3)],[0 a(2,3)],[0 a(3,3)],'color','b');  % z轴
text(a(1,3),a(2,3),a(3,3),'z');

plot3([0 temp2(1)],[0 temp2(2)],[0 temp2(3)],'color','green');  % 三角形的边

plot3([a(1,2) temp2(1)],[a(2,2) temp2(2)],[a(3,2) temp2(3)],'color','green');  % 三角形的边
hold off

grid on;
axis([-1 1 -1 1 -1 1]);
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴')  % 标记起始位置grid on
drawnow

end


% function Display_axis(a)
% 
% hold off
% a(:,2) = a(:,2) * 0.5;
% 
% plot3([0 a(1,1)],[0 a(2,1)],[0 a(3,1)],'color','k');  % x轴
% text(a(1,1),a(2,1),a(3,1),'x');
% hold on;
% plot3([0 a(1,2)],[0 a(2,2)],[0 a(3,2)],'color','green');  % y轴
% text(a(1,2),a(2,2),a(3,2),'y');
% hold on;
% plot3([0 a(1,3)],[0 a(2,3)],[0 a(3,3)],'color','red');  % z轴
% text(a(1,3),a(2,3),a(3,3),'z');
% 
% temp = 0.5 * (a(:,1) + a(:,2));  % 矩形中心坐标
% fill3([0 a(1,1) 2*temp(1) a(1,2)],[0 a(2,1) 2*temp(2) a(2,2)],[0 a(3,1) 2*temp(3) a(3,2)],'g');
% grid on;
% axis([-1 1 -1 1 -1 1]);
% xlabel('X轴');
% ylabel('Y轴');
% zlabel('Z轴')  % 标记起始位置
% grid on
% drawnow
% 
% end