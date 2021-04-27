% 初始时对重力进行了校正
% 对陀螺仪漂移进行一下初始校正？？？


function Ins_get_trajectory_accurate(temp)

global t
global coordinate
global displacement_T
global v  % 速度
global vv
global T1
global g  % 重力加速度
global a_cor  % 加速度校正参数
global w_cor  % 角速度校正参数
global N_g_cor % 校正步数
global w_record

global t1

T2 = hex_dec(temp(2)) * 0.001;
if t==1
    t1 = T2;
end
a = hex_dec(temp(3:5));  % 加速度
w = hex_dec(temp(6:8));  % 角速度
a = a * 0.0005981445;
w = w * 0.0076293945 * 0.0175;  % 角速度量程（弧度） 250，250/32768(7FFF) =  0.0076293945，pi/180 = 0.0175
a = a';

% T = 0.005;  % 时间间隔为5ms
T = T2 - T1;
T1 = T2;
t = t + 1;

% 初始化,校准
 if t <= N_g_cor
        coordinate_ins = [0 0 0]';
        v = [0 0 0]';
        a_T = [0 0 0]';
        % 实际重力加速度
        g = g + a/N_g_cor;
        % 校正角速度
        w_record(:,t) = w';
        if t == N_g_cor  % 校准加速度
            norm_g =  (g(1)^2 + g(2)^2 + g(3)^2)^0.5;
            a_cor = 9.7966 / norm_g;  % 计算加速度校正参数
            g = 9.7966 * (g/norm_g);
            % 求w的门限值，！！！其他方法！！！
            for i = 1:3
              w_cor(i) = mean(w_record(i,:));
            end
            'start-------------'    
        end
else
    w = w - w_cor;
    for i = 1:3
        if abs(w(i)) < 0.004
            w(i) = 0;
        end
    end
    w_record(:,t) = w';
    DCMg = Quaternion_without_m( T, a, w, t );
    
    % 加速度转化到全局坐标系下，并除去重力
    a_T = DCMg * (a*a_cor) - g;
    for i = 1:3
        if abs(a_T(i)) < 0.06
            a_T(i) = 0;
        end
    end
    
    % 得到速度<-加速度积分
    v = v + a_T * T;  % （实际坐标下的）加速度的积分即为（实际坐标下的）速度
    
    % 得到位移<-速度积分
    displacement_T = v * T + 0.5 * a_T * T^2;  % T时刻下的位移
    
    coordinate = coordinate + displacement_T;
end
vv = [vv a_T];  % record v or a_T

if ~mod(t-1,15)
    Display_trajectory(coordinate);
end

end


% clear;
% clc;
% 
% % 提取txt的距离信息，并放入C_txt{4}中
% fid = fopen('2018-8-6-10-49-7.plot(vv(2,:))txt');
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