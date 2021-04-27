% clear;
% clc;
% 
% % 提取txt的距离信息，并放入C_txt{4}中
% fid = fopen('2018-8-1-15-40-22.txt');
% C_txt = textscan(fid, '%s %s %s %s %s %s %s %s %s');
% fclose(fid);
% 
% T = 0.005;  % 时间间隔为5ms
% a_x = hex_dec(C_txt{3});
% a_y = hex_dec(C_txt{4});
% a_z = hex_dec(C_txt{5});
% g_x = hex_dec(C_txt{6});  % x轴方向角速度
% g_y = hex_dec(C_txt{7});
% g_z = hex_dec(C_txt{8});
% 
% a_x = a_x * 0.0005981445 + a_x_correction;  % 加速度量程 9.8*2，9.8*2/32768(7FFF) = 0.0005981445
% a_y = a_y * 0.0005981445 + a_y_correction;
% a_z = a_z * 0.0005981445;
% g_x = pi * g_x * 0.0076293945 / 180;  % 角速度量程 250，250/32768(7FFF) =  0.0076293945
% g_y = pi * g_y * 0.0076293945 / 180;
% g_z = pi * g_z * 0.0076293945 / 180 - 0.0124;
% 
% % 初始化DCMg =[ig jg kg] = (DCMb)t，DCMb即为体坐标系下，[I J K]的坐标
% norm_a = (a_x^2 + a_y^2 + a_z^2)^0.5;
% a = [a_x a_y a_x]'/norm;  % 归一化的重力加速度
% Kb0 = -a ;  % 体坐标系下的zenith方向的单位向量的坐标
% Ib0 = cross(Kb0,[0 ay 0]);
% Jb0 = cross(Kb0,Ib0);
% Ib0 = cross(Kb0,Jb0);
% %归一化!!!!!!
% 
% w = [g_x g_y g_z];  % 角速度
% theta_g = T * w;  % 角速度的积分为角度
% Kb1a = -a;  % 新的重力加速度
% theta_a = cross(Kb0,(Kb1a - Kb0));
% theta = weight * theta_a + (1 - weight) * theta_g;
% Kb1 = Kb0 + cross(theta,Kb0);
% Ib1 = Ib0 + cross(theta,Ib0);
% Jb1 = cross(Kb1,Ib1);
% 
% DCMb = [Ib1' Jb1' Kb1'];
% DCMg = [Ib1;Jb1;Kb1];
% ig = DCMg * [1 0 0]';  % 体坐标系的基底在全局坐标系中的坐标
% jg = DCMg * [0 1 0]';
% kg = DCMg * [0 0 1]';



function Ins_get_trajectory(temp)

global t
global X  % 坐标
global Y
global v  % 速度
global vv
global T1
global angle_z

T2 = hex_dec(temp(2)) * 0.001;
a = hex_dec(temp(3:5));  % 加速度
w = hex_dec(temp(6:8));  % 角速度
a = a * 0.0005981445;
w = w * 0.0076293945 * 0.0175;  % 角速度量程 250，250/32768(7FFF) =  0.0076293945，pi/180 = 0.0175

% T = 0.005;  % 时间间隔为5ms
T = T2 - T1;
T1 = T2;
t = t + 1;

% 初始化
if t == 1
    X = 0;
    Y = 0;
    v = [0 0 0];
    angle_z = 0;
else 
    % 更新
    angle_z = angle_z + w(3) * T;
      
    a_x_T = a(1) * cos(angle_z) - a(2) * sin(angle_z);  % 先计算T时间段内，矢量加速度a从imu参考坐标到实际坐标的转换
    a_y_T = a(2) * cos(angle_z) + a(1) * sin(angle_z);  % a：参考坐标下-->实际坐标下
     
    v(1) = v(1) + a_x_T * T;  % （实际坐标下的）加速度的积分即为（实际坐标下的）速度
    v(2) = v(2) + a_y_T * T;
    
    x_T = v(1) * T + 0.5 * a_x_T * T^2;  % T时刻下的位移
    y_T = v(2) * T + 0.5 * a_y_T * T^2;
        
    X = X + x_T;
    Y = Y + y_T;
end
vv = [vv v'];

if ~mod(t-1,20)
    Display_trajectory(X,Y);
end

end


% clear;
% clc;
% 
% % 提取txt的距离信息，并放入C_txt{4}中
% fid = fopen('2018-8-6-10-49-7.txt');
% C_txt = textscan(fid, '%s %s %s %s %s %s %s %s %s');
% fclose(fid)
% 
% a_x_correction = 0; a_y_correction = 0;  % 修正
% x(1) = 0; y(1) = 0; z(1) = 0;  % 初始相对（以imu为参考的）位置
% X(1) = 0; Y(1) = 0; Z(1) = 0;  % 初始绝对（选定的以大地为参考）位置
% T = 0.005;  % 时间间隔为5ms
% v_x(1) = 0; v_y(1) = 0; v_z(1) = 0;  % 初速度为零
% or_z(1) = 0;  % z轴方向初始角度（初始方位，即相对位置与绝对位置）为零
% a_x = hex_dec(C_txt{3});
% a_y = hex_dec(C_txt{4});
% a_z = hex_dec(C_txt{5});
% g_x = hex_dec(C_txt{6});  % x轴方向角速度
% g_y = hex_dec(C_txt{7});
% g_z = hex_dec(C_txt{8});
% 
% a_x = a_x * 0.0005981445 + a_x_correction;  % 加速度量程 9.8*2，9.8*2/32768(7FFF) = 0.0005981445
% a_y = a_y * 0.0005981445 + a_y_correction;
% a_z = a_z * 0.0005981445;
% g_x = pi * g_x * 0.0076293945 / 180;  % 角速度量程 250，250/32768(7FFF) =  0.0076293945
% g_y = pi * g_y * 0.0076293945 / 180;
% g_z = pi * g_z * 0.0076293945 / 180 - 0.0124;
% 
% % 在每个时间段T内：
% % 由角速率（已知），得到两个坐标系的角度 -> 由该角度，对矢量加速度a（已知）坐标转换 -
% % -> 对转换后的a积分，即得速度 -> 由速度，计算该时间段内的位移 -> 对该位移积分，得到实际坐标.
% for t = 2:length(C_txt{1})
%     if ismember(C_txt{1}(t),'$IMU')
%         % 角速度的积分是角度，即方位orientation
%         or_z(t) = or_z(t-1) + g_z(t) * T;
%         
%         a_x_T = a_x(t-1) * cos(or_z(t-1)) - a_y(t-1) * sin(or_z(t-1));  % 先计算T时间段内，矢量加速度a从imu参考坐标到实际坐标的转换
%         a_y_T = a_y(t-1) * cos(or_z(t-1)) + a_x(t-1) * sin(or_z(t-1));  % a：参考坐标下-->实际坐标下
%         
%         v_x(t) = v_x(t-1) + a_x_T * T;  % （实际坐标下的）加速度的积分即为（实际坐标下的）速度
%         v_y(t) = v_y(t-1) + a_y_T * T;
%         
%         x_T = v_x(t-1) * T + 0.5 * a_x_T * T^2;
%         y_T = v_y(t-1) * T + 0.5 * a_y_T * T^2;
%         
%         X(t) = X(t-1) + x_T;
%         Y(t) = Y(t-1) + y_T;
%          
%     end
% end
% plot(X,Y,'g');
% axis equal;
% xlabel('x轴');
% ylabel('y轴');
% text(x(1),y(1),'o','color','b');  % 标记起始位置