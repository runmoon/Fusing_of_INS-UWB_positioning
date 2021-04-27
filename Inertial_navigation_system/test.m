clear;
clc;
s = serial('COM5');
set(s,'BaudRate',921600,'StopBits',1,'Parity','none','DataBits',8,'FlowControl','none');%设置波特率  停止位  校验位 
fopen(s);

global q  % 四元数q = [q0 q1 q2 q3]
global t
global T1
global vv  % 速度
global integral_e_a
global g  % 重力加速度
global a_cor  % 加速度校正参数
t = 0;
T1 = 0;
vv = [0 0 0]';
integral_e_a = [0 0 0];
g = [0 0 0]';
a_cor = 0;

% 创建txt文档。  
ch=clock;
ch_1=int2str(ch(1));
ch_2=int2str(ch(2));
ch_3=int2str(ch(3));
ch_4=int2str(ch(4));
ch_5=int2str(ch(5));
ch_6=int2str(ch(6));
ch_7='.txt';
FileName = [ch_1,'-',ch_2,'-',ch_3,'-',ch_4,'-',ch_5,'-',ch_6,ch_7];  
FileID = fopen(FileName,'a+');

s.ReadAsyncMode = 'continuous';
i = 5000;
'start_________________________________________'
tic
while(i)
    if(s.BytesAvailable)
%         temp_1 = fscanf(s,'%c');
%         temp_2 = temp_1';
%         fprintf(FileID,'%c',temp_2);
        temp1 = fscanf(s,'%c');
        temp2 = regexp(temp1, ' ', 'split');
        if ismember( 'IMU',temp1)
%             Ins_get_trajectory_accurate(temp2);
temp = temp2;
T2 = hex_dec(temp(2)) * 0.001;
a = hex_dec(temp(3:5));  % 加速度
w = hex_dec(temp(6:8));  % 角速度
a = a * 0.0005981445;
w = w * 0.0076293945 * 0.0175;  % 角速度量程 250，250/32768(7FFF) =  0.0076293945，pi/180 = 0.0175
a = a';

% T = 0.005;  % 时间间隔为5ms
T = T2 - T1;
T1 = T2;
t = t + 1;

DCMg = Quaternion_without_m( T, w, t);

% 初始化,校准
if t <= 200
    X = 0;
    Y = 0;
    v = [0 0 0]';
    % 实际重力加速度
    g = g + 0.005 * a;
    if t==200  % 校准加速度
        a_cor = 9.7966 / (g(1)^2 + g(2)^2 + g(3)^2)^0.5;  % 计算加速度校正参数
        g_norm = (g(1)^2 + g(2)^2 + g(3)^2)^0.5;
        g  = (g / g_norm) * 9.7966;
    end
else
    % 阈值，去除a的噪声？？？？？？？？？？？？？？？？？？
    
    %
    
    % 加速度转化到全局坐标系下
    a_T = DCMg * (a*a_cor) - g;
    
    % 得到速度<-加速度积分
    v = v + a_T * T;  % （实际坐标下的）加速度的积分即为（实际坐标下的）速度
    
    % 得到位移<-速度积分
    x_T = v(1) * T + 0.5 * a_T(1) * T^2;  % T时刻下的位移
    y_T = v(2) * T + 0.5 * a_T(2) * T^2;
    
    X = X + x_T;
    Y = Y + y_T;
    
    vv = [vv a_T];
end


        end
    end
    i = i - 1;
end
toc
'end_________________________________________'
% sizeA = ;
% A = fscanf(fileID,formatSpec,sizeA);

% while(1)
%     if(s.BytesAvailable)
%         out = fscanf(s)
%     end
% end
s
fclose(s);
delete(s);  
clear s