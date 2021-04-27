function Quaternion_with_PID2( T, a, w, t)
% ????????????????????
 
global DCMg2
global q2  % ???q = [q0 q1 q2 q3]
global N_g_cor % ????
global roll  % ???
global pitch  % ???
global yaw  % ???
global integral_e_a
global error_w_record
global error_a_record
global state
global Kp2
 
 Ki = 0;  % ?????Ki????????????
 Kd = 0.05; 
 
halfT = 0.5 * T;  % ????
 
if state==0
    Kp2=4;
end
 
% ???
if t <= N_g_cor
    if t < N_g_cor
        q2 = [1 0 0 0]';
%       q2 = [0.9997 0 0 -0.0262]';
    end
else
    norm_a = (a(1)^2 + a(2)^2 + a(3)^2)^0.5;
    a = a / norm_a;  % ???
    % ??????????????? DCMb*[0 0 1]'????DCMb????
    v(1) = 2 * (q2(2)*q2(4) - q2(1)*q2(3));
    v(2) = 2 * (q2(1)*q2(2) + q2(3)*q2(4));
    v(3) = q2(1)*q2(1) - q2(2)*q2(2) - q2(3)*q2(3) + q2(4)*q2(4);
    v = v';
    % ??PI?????????
%     error_a = a - v;
    error_a = cross(a, v);  % ?? <- ???g???g???
    error_a_record(:, t) = error_a;
    integral_e_a = integral_e_a + Ki * error_a * T;  % ???
    error_w = Kp2 * error_a + integral_e_a + Kd * (error_a - error_a_record(:,t-1));  % ??? <- ??? + ???
%     error_w = Kp * error_a + integral_e_a;  % ??? <- ??? + ???
 
    error_w_record = [error_w_record error_w];
    w = w + error_w;
    w = w;
    % ?????????(?)???----------
    q2(1) = q2(1) + (-q2(2)*w(1) - q2(3)*w(2) - q2(4)*w(3)) * halfT;
    q2(2) = q2(2) + (q2(1)*w(1) + q2(3)*w(3) - q2(4)*w(2)) * halfT;
    q2(3) = q2(3) + (q2(1)*w(2) - q2(2)*w(3) + q2(4)*w(1)) * halfT;
    q2(4) = q2(4) + (q2(1)*w(3) + q2(2)*w(2) - q2(3)*w(1)) * halfT;
 
    % ??????
    norm_q = (q2(1)^2 + q2(2)^2 + q2(3)^2 + q2(4)^2)^0.5;
    q2 = q2 / norm_q;
end
 
% if ~mod(t-1,70)
    % ?????DCMb
    % ????????
    DCMb(1,1) = q2(1)*q2(1) + q2(2)*q2(2) - q2(3)*q2(3) - q2(4)*q2(4);
    DCMb(2,1) = 2 * (q2(2)*q2(3) - q2(1)*q2(4));
    DCMb(3,1) = 2 * (q2(2)*q2(4) + q2(1)*q2(3));
    DCMb(1,2) = 2 * (q2(2)*q2(3) + q2(1)*q2(4));
    DCMb(2,2) = q2(1)*q2(1) - q2(2)*q2(2) + q2(3)*q2(3) - q2(4)*q2(4);
    DCMb(3,2) = 2 * (q2(4)*q2(3) - q2(1)*q2(2));
    DCMb(1,3) = 2 * (q2(2)*q2(4) - q2(1)*q2(3));
    DCMb(2,3) = 2 * (q2(4)*q2(3) + q2(1)*q2(2));
    DCMb(3,3) = q2(1)*q2(1) - q2(2)*q2(2) - q2(3)*q2(3) + q2(4)*q2(4);
    DCMg2 = DCMb';
    
%   if ~mod(t-1,30)
%       Display_axis(DCMg);
%   end
 
%   roll = atan( DCMg(3,2)/DCMg(3,3) );
%   pitch = asin( -DCMg(3,1) );
%   yaw = atan( DCMg(2,1)/DCMg(1,1) );
  
% % % ??????????DCM = ??????DCM????????
% roll = atan2( 2*(q(1)*q(2) + q(3)*q(4)), (1 - 2*q(2)*q(2) - 2*q(3)*q(3)) )* 57.3; % ???,roll
% pitch = asin( -2*q(2)*q(4) + 2*q(1)*q(3) ) * 57.3; % ???,pitch
% yaw = atan2( 2*(q(2)*q(3) + q(1)*q(4)), (q(1)*q(1)+q(2)*q(2)-q(3)*q(3)-q(4)*q(4)) ) * 57.3; % ???,yaw
 
end
 
 