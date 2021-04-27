clear;
clc;

global x_act
global y_act
global Vx  % 实际速度
global Vy
% 提取txt的距离信息，并放入C_txt{4}中
% fid = fopen('2018-8-28-12-8-55.txt');
fid = fopen('2020-4-26-2_zhuan2.txt');

temp = fscanf(fid,'%f');           % 提取实际x轴与y轴的坐标放入temp
x_act = temp(1); y_act = temp(2);  % 实际位置x0,y0
Vx = temp(3); Vy = temp(4);        % 实际位置x0,y0
eq_tr = temp(5);                   % 基站（等边三角形）的边长
C_txt = textscan(fid, '%s %s %s %s %s %f %d');
start = ismember(C_txt{4},'F1');   % 'F1'即为每次数据的开始标志,标为1，其他标为0
start_num = find(start==1);        % 记录'F1'的位置（所在行的次序）

fclose(fid);

% ------------------解出坐标位置---------------------
% LX = B ==> X = inv(L'L) * A'B，构造A和B
% 由基站坐标得到矩阵A
x_beacon = [0 0.866*eq_tr 0];   % 分别对应三个基站的横坐标x1,x2,x3
y_beacon = [0 0.5*eq_tr eq_tr];
x_beacon = [0 3.2 1.6];
y_beacon = [0 0 4.25];


L(:,1) = 2*[x_beacon(1)-x_beacon(3) x_beacon(2)-x_beacon(3)]';
L(:,2) = 2*[y_beacon(1)-y_beacon(3) y_beacon(2)-y_beacon(3)]';
b1 = x_beacon(1)^2 - x_beacon(3)^2 + y_beacon(1)^2 - y_beacon(3)^2;
b2 = x_beacon(2)^2 - x_beacon(3)^2 + y_beacon(2)^2 - y_beacon(3)^2;
% 由距离得到矩阵B
coordinate = [];
% (length(start_num)-1)为uwb数据的组数，一组数据（三个距离）可以计算得到一个位置点
% i将记录下总的数据组的组数
void = 0;  % 记录无效的数据组的个数
% for i = 1 : 50
for i = 1 : (length(start_num)-1)
    d = C_txt{6}((start_num(i)+1):(start_num(i+1)-1));
    if (length(d) == 3)    % 确定该组距离的数据个数为3
        B(1) = b1 + d(3)^2 - d(1)^2;
        B(2) = b2 + d(3)^2 - d(2)^2;
        coordinate(:,i-void) = inv(L'*L) * (L'*B');  % 最小二乘法计算得到标签的位置
        % coordinate(:,i-void) = A\B';  % 计算得到标签的位置
        timetable(i-void) =  str2double( C_txt{1}{start_num(i)+2}(2:10) );  % 记录每个数据组对应的时间
    else
        void = void + 1;  % 记录距离的数据小于3的数据组的组数
    end
end

g_t = 0.7;  % 设置时间间隔
% Vx = 0;  % 设置速度
% Vy = 0.15;  % 0.14
% ------------------滤波-------------------
Y(1,1) = coordinate(1,1);  % Y为观察值,x和y两个坐标值
Y(2,1) = coordinate(2,1);
X(1,1) = Y(1,1);  % 初始的状态值（X0）？？？？？？？
X(2,1) = Vx;
X(3,1) = Y(2,1);
X(4,1) = Vy;
sum_error = 0;  % 计算误差
sum_error_with_kf = 0;

% x = F*x + Q;  x = [Px Vx Py Vy]'
% y = H*x + P;  y = [Px Py]';
F = [1 g_t 0 0;0 1 0 0;0 0 1 g_t;0 0 0 1];  % t-1时刻到t时刻的状态X的转移矩阵，g_t是时间间隔
H = [1 0 0 0;0 0 1 0];    % H为方
P = [10 0 0 0;0 10 0 0;0 0 10 0;0 0 0 10];
% Q = [1e-6 0 0 0;0 1e-7 0 0;0 0 1e-7 0;0 0 0 1e-6];  % Q为过程噪声
% % R = [1e-6 0;0 1e-6];  % R为观测噪声
% % R = [15e-1 1e-1;1e-1 15e-1];  % R为观测噪声
% R = [15e-1 0;0 15e-1];  % R为观测噪声
Q = [1e-5 0 0 0;0 1e-5 0 0;0 0 1e-5 0;0 0 0 1e-5];  % Q为过程噪声
R = [1e-1 0;0 1e-1];  % R为观测噪声

I = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% X和P的初值可以随意设置，但P的初值不能为0
% 应该根据具体的使用场景收集到的数据来决定Q和R的取值,Q = 1e-7; R = 5e-5; 
% 此处Q和R取值为经验取值，没有对实际噪声做数理统计！！！，不是真实值
turn_point = 22;

for k = 2:(i-void)
% for t = 2:(i-j)
    % 预测 时间更新
    F(1,2) = timetable(k)-timetable(k-1); F(3,4) = F(1,2);  % 校准时间间隔!!!
    X1(:,k) = F * X(:,k-1);  % U为外部输入，固定位置，无外部输入，U取零
    P = F * P * F' + Q; % Q为 预测噪声协方差
    % 修正 测量更新
    Y(:,k) = coordinate(:,k);  % Y为观察值
    K = (P*H') * pinv((H*P*H') + R);    % R为 观测噪声协方差，源于传感器的测量（测量精度及热噪声等的影响）；K为 卡尔曼增益
    X(:,k) = X1(:,k) + K * (Y(:,k) - H * X1(:,k));
    P = (I - K*H) * P;
    sum_error = sum_error + (Y(1,k) - x_act)^2 * (Vx == 0) + (Y(2,k) - y_act)^2 * (Vy == 0);
    if k >= 10
        if k < turn_point
            sum_error_with_kf = sum_error_with_kf + (X(1,k) - x_act)^2 * (Vx == 0) + (X(3,k) - y_act)^2 * (Vy == 0);
        else
            Vx = ;
            Vy = ;
            sum_error_with_kf = sum_error_with_kf + (X(1,k) - x_act)^2 * (Vx == 0) + (X(3,k) - y_act)^2 * (Vy == 0);
        end
    end
    
end
% 计算均方根误差
rmse = (sum_error / (i - void))^0.5;
rmse_kf = (sum_error_with_kf / (i - void - 10))^0.5;
rmse
rmse_kf
str_rmse = num2str(rmse);
str_rmse_kf = num2str(rmse_kf);

% --------------------绘图---------------------
% 二维图
plot(Y(1,:),Y(2,:),'ob',X(1,:),X(3,:),'*r');  % 原始定位-蓝色；加入KF后的-红色
hold on;
plot(Y(1,:),Y(2,:),'b',X(1,:),X(3,:),'r');  % 绘制图形与连线
plot([x_beacon x_beacon(1)], [y_beacon,y_beacon(1)], 'b');  % 绘制定位区域，蓝色
%axis([-2 14 -1 13]);
axis([-2 5 -1 6]);
xlabel('x轴');
ylabel('y轴');
grid minor;
% grid on;
text(Y(1,1),Y(2,1),' s');  % 标记定位的起始位置，
text(x_beacon(1),y_beacon(1),'  b1');  % 标记基站位置，黄色
text(x_beacon(2),y_beacon(2),'  b2');
text(x_beacon(3),y_beacon(3),'  b3');
legend_1 = strcat('原始定位的RMSE: ',str_rmse,'m');  % 均方差误差
legend_2 = strcat('加入KF后的RMSE: ',str_rmse_kf,'m');
legend(legend_1,legend_2);
if Vx == 0
    x_act = [x_act x_act];  % 沿y轴方向运动
    y_act = [0 11];
else
    x_act = [0 11];  % 沿x轴方向运动
    y_act = [y_act y_act];
end
plot(x_act,y_act, 'g');  % 绘制实际路径，绿色
grid on;
hold off

figure(2);
plot(Y(1,:),'-ob');  % 原始定位-蓝色；加入KF后的-红色

% t = 1:i-j;
% plot(t,X(1,:),'ob',t,X(2,:),'*r');  % 原始定位-蓝色；加入KF后的-红
% hold on
% plot(t,X(1,:),'b',t,X(2,:),'r'); 
% grid minor;

% % 三维图，随时间变化
% hold off
% t = 1:(i-j);
% plot3(t,x0,y0,'k',t,Y(1,:),Y(2,:),'b',t,X(1,:),X(2,:),'r');  % 实际值-黑色；原始定位-蓝色；加入KF后-红色
% hold on;
% axis([0 (i-j) -1 2.5 0 3]);
% % axis([0 (i-j) 1 4 1 5.5]);  % 为偏差极大的准备的
% xlabel('t_时间');
% ylabel('x轴');
% zlabel('y轴');
% % grid minor;
% grid on;
% text(1,Y(1,1),Y(2,1),'* s','color','b');  % 标记定位的起始位置，'*'
% text(1,x0(1),y0(1),'o','color','k');  % 标记实际位置，黑
% legend_1 = strcat('原始定位的RMSE: ',str_rmse);
% legend_2 = strcat('加入KF后的RMSE: ',str_rmse_kf);
% legend('实际位置',legend_1,legend_2,1);
% plot3(t,Y(1,:),Y(2,:),'ob',t,X(1,:),X(2,:),'*r'); 
% hold off
