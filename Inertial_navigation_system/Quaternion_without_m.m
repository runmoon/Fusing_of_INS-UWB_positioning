% function Quaternion( temp )
% 
% global q  % 四元数q = [q0 q1 q2 q3]
% global integral_e_a
% global t
% global T1
% 
% Kp = 0.7;  % 比例参数？？？？？？？？
% Ki = 0.001;
% 
% T2 = hex_dec( temp(2) );
% a = hex_dec( temp(3:5) );  % 加速度
% w = hex_dec( temp(6:8) );  % 角速度
% w = w * 0.0076293945 * 0.0175;  % 角速度量程 250，250/32768(7FFF) =  0.0076293945，pi/180 = 0.0175
% t = t + 1;
% % halfT = 0.003;  % 时间间隔，或重新计算
% halfT = 0.5 * (T2 - T1) * 0.001;
% T1 = T2;

function [ DCMg ] = Quaternion_without_m( T, a, w, t)
% 传入时间间隔、加速度、角速度、时间段计数

global q  % 四元数q = [q0 q1 q2 q3]
global N_g_cor % 校正步数
global integral_e_a

Kp = 0.2;  % 比例参数？？？？？？？？
Ki = 0.005;

% halfT = 0.003;  % 时间间隔，或重新计算
halfT = 0.5 * T;

% 初始化
if t <= N_g_cor
    q = [1 0 0 0];
else
%     % 实际重力加速度
%     norm_a = (a(1)^2 + a(2)^2 + a(3)^2)^0.5;
%     a = a / norm_a;  % 单位化
%     % 理论重力加速度，由四元数表示的 DCMb*[0 0 1]'得到，即DCMb的第三列
%     v(1) = 2 * (q(2)*q(4) - q(1)*q(3));
%     v(2) = 2 * (q(1)*q(2) + q(3)*q(4));
%     v(3) = q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4);
% 
%     % 构建PI控制器，求误差补偿
%     error_a = cross(a, v);  % 误差 <- 对实际g与理论g求叉乘
%     integral_e_a = integral_e_a + Ki * error_a;  % 积分项？？？？？？？？？？？？？？？t
%     error_w = Kp * error_a + integral_e_a;  % 补偿值 <- 比例项 + 积分项
% 
%     % 角速度：补偿后的w <- w + 补偿值
%     w = w + error_w;
    
    % 阀值，去除w的噪声？？？？？？？
    
    w = w;
    % 四元数微分方程求解(法)，更新----------
    q(1) = q(1) + (-q(2)*w(1) - q(3)*w(2) - q(4)*w(3)) * halfT;
    q(2) = q(2) + (q(1)*w(1) + q(3)*w(3) - q(4)*w(2)) * halfT;
    q(3) = q(3) + (q(1)*w(2) - q(2)*w(3) + q(4)*w(1)) * halfT;
    q(4) = q(4) + (q(1)*w(3) + q(2)*w(2) - q(3)*w(1)) * halfT;

    % 四元数单位化
    norm_q = (q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2)^0.5;
    q = q / norm_q;
end

% if ~mod(t-1,70)
    % 由四元数求DCMb
    DCMb(1,1) = q(1)*q(1) + q(2)*q(2) - q(3)*q(3) - q(4)*q(4);
    DCMb(2,1) = 2 * (q(2)*q(3) - q(1)*q(4));
    DCMb(3,1) = 2 * (q(2)*q(4) + q(1)*q(3));
    DCMb(1,2) = 2 * (q(2)*q(3) + q(1)*q(4));
    DCMb(2,2) = q(1)*q(1) - q(2)*q(2) + q(3)*q(3) - q(4)*q(4);
    DCMb(3,2) = 2 * (q(4)*q(3) - q(1)*q(2));
    DCMb(1,3) = 2 * (q(2)*q(4) - q(1)*q(3));
    DCMb(2,3) = 2 * (q(4)*q(3) + q(1)*q(2));
    DCMb(3,3) = q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4);
    DCMg = DCMb';
%  if ~mod(t-1,70)
%      Display_axis(DCMg);
%  end

% % 由方程：四元数表示的DCM = 欧拉角表示的DCM，来反解出欧拉角
% pitch = asin( -2*q(2)*q(4) + 2*q(1)*q(3) ) * 57.3; % 俯仰角,pitch
% roll = atan2( 2*(q(1)*q(2) + q(3)*q(4)), (1 - 2*q(2)*q(2) - 2*q(3)*q(3)) )* 57.3; % 横滚角,roll
% yaw = atan2( 2*(q(2)*q(3) + q(1)*q(4)), (q(1)*q(1)+q(2)*q(2)-q(3)*q(3)-q(4)*q(4)) ) * 57.3; % 航向角,yaw

end