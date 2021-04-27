% 4个基站

clear;
clc;

global x_act
global y_act
global Vx  % 实际速度
global Vy
% 提取txt的距离信息，并放入C_txt{4}中
% fid = fopen('2020-4-22-4.txt');
fid = fopen('2018-8-31-18-12-56.txt');

temp = fscanf(fid,'%f');           % 提取实际x轴与y轴的坐标放入temp
x_act = temp(1); y_act = temp(2);  % 实际位置x0,y0
Vx = temp(3); Vy = temp(4);        % 实际位置x0,y0
C_txt = textscan(fid, '%s %s %s %s %s %f %d');
start = ismember(C_txt{4},'F1');   % 'F1'即为每次数据的开始标志,标为1，其他标为0
start_num = find(start==1);        % 记录'F1'的位置（所在行的次序）

fclose(fid);

% ------------------解出坐标位置---------------------
% LX = B ==> X = inv(L'L) * A'B，构造A和B
% 由基站坐标得到矩阵A
x_beacon = [0 6 0 6];   % 分别对应三个基站的横坐标x1,x2,x3
y_beacon = [0 0 3.4 3.4];

L(:,1) = 2*[x_beacon(1)-x_beacon(3) x_beacon(2)-x_beacon(3) x_beacon(4)-x_beacon(3)]';
L(:,2) = 2*[y_beacon(1)-y_beacon(3) y_beacon(2)-y_beacon(3) y_beacon(4)-y_beacon(3)]';
b1 = x_beacon(1)^2 - x_beacon(3)^2 + y_beacon(1)^2 - y_beacon(3)^2;
b2 = x_beacon(2)^2 - x_beacon(3)^2 + y_beacon(2)^2 - y_beacon(3)^2;
b3 = x_beacon(4)^2 - x_beacon(3)^2 + y_beacon(4)^2 - y_beacon(3)^2;
% 由距离得到矩阵B
coordinate = [];
% (length(start_num)-1)为uwb数据的组数，一组数据（三个距离）可以计算得到一个位置点
% i将记录下总的数据组的组数
void = 0;  % 记录无效的数据组的个数
% for i = 1 : 50
for i = 1 : (length(start_num)-1)
    d = C_txt{6}((start_num(i)+1):(start_num(i+1)-1));
    if (length(d) == 4)    % 确定该组距离的数据个数为3
        B(1) = b1 + d(3)^2 - d(1)^2;
        B(2) = b2 + d(3)^2 - d(2)^2;
        B(3) = b3 + d(3)^2 - d(4)^2;
        coordinate(:,i-void) = inv(L'*L) * (L'*B');  % 最小二乘法计算得到标签的位置
        % coordinate(:,i-void) = A\B';  % 计算得到标签的位置
        timetable(i-void) =  str2double( C_txt{1}{start_num(i)+2}(2:10) );  % 记录每个数据组对应的时间
    else
        void = void + 1;  % 记录距离的数据小于3的数据组的组数
    end
end

% 二维图
plot(coordinate(1,:),coordinate(2,:),'*r');  % 原始定位-蓝色；加入KF后的-红色
hold on;
plot(coordinate(1,:),coordinate(2,:),'-r');  % 绘制图形与连线
plot([x_beacon(1:2) x_beacon(4)  x_beacon(3) x_beacon(1)], [y_beacon(1:2) y_beacon(4)  y_beacon(3) y_beacon(1)], 'b');  % 绘制定位区域，蓝色
axis([-5 10 -1 5]);
xlabel('x轴');
ylabel('y轴');
% grid minor;
grid on;
