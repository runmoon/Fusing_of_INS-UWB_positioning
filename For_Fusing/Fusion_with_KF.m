% This is the main program
% The fusing of INS and UWB positioning
% Direct Kalman filter is used for fusion
% ????????????????????????????????????????????

% ??????w_record_all?a_record_all,uwb_record?g?a_sen?a_zero_drift;w_cor(???????),N_g_cor(????)
% g_local = 9.7966;
% a_record_all??[ a; j; k; T; T2; t;]?uwb_record??[k; j; T; T2; Y(:, k); 4; d'; t;]

  a_scale = [0.9997 0.9987 0.9922]'; a_offset = [0.113 0.156 0.345]';  % 10? ???????????
  a_scale = [ 1.0066 0.9943  0.9837]'; a_offset = [0.156893 -0.098563 0.562545]'; % 5? ???
?
% ????
global X1_fu
global Y_fu  % uwb?????
global F_fu
global H_fu
global P_fu
global Q_fu
global R_fu  % uwb?????
global I_fu
?
global DCMg
global q  % ???q = [q0 q1 q2 q3]
global N_g_cor % ????
global roll  % ???
global pitch  % ???
global yaw
global integral_e_a
global yaw_1
global error_w_record
global a_ave  % ????????
global state
global Kp1
global Kp2
Kp1 = 2;
Kp2 = 0.2;
?
state =0;
?
error_w_record = [0 0 0]';
state_count = 0;
?
global DCMg1
global q1  % ???q = [q0 q1 q2 q3]
global DCMg2
global q2
?
q = [1 0 0 0]';
integral_e_a = [0 0 0]';
integral_e_a_record = [0 0 0]';
q_record_1 = [1 0 0 0]';
a_T_record_1 = [0 0 0]';
a_T_record_2 = [0 0 0]';
h = 1;  % INS??
c = 2;  % UWB??
velocity_ins_1 = [0 0 0]';
velocity_ins_2 = [0 0 0]';
velocity_ins_o = [0 0 0]';
position_ins_1 = [0 0 0]';
position_ins_2 = [0 0 0]';
position_ins_o = [0 0 0]';
velocity_ins_b = [0 0 0]';
position_ins_b = [0 0 0]';
velocity_ins_f = [0 0 0]';
position_ins_f = [0 0 0]';
?
a_record = a_record_all(1:3, :);
w_record = w_record_all(1:3, :);
?
w3_filter(1) = w_record(3, 1);
yaw_1(1) = 0;
yaw_record = 0;
det_acc = 0;
eula_record = [0 0 0]';
?
u1 =5; u2 = 6;
Y = uwb_record(u1:u2, :);
record_a2(1) = 0;
?
P_s = [100 0;0 100];
X_s(:, 1:2) = uwb_record(u1:u2, 1:2);
X1_s = X_s;
uwb_s = 0;  % ????????uwb????????
?
Initial_KF;
Initial_kf_fuse;
?
count_a = 0;
detect_a = 0;
yaw1_t = 0;
X_fu_record = [0 0 0 0 0 0 0]';
yaw1 = 0;
for ii = 1:3  % ??????2??200??????2?????
     a_record_lf(ii, :) = butter1array(a_record(ii,:), 200, 2);  % a1??????2hz
     a_record_lf2(ii, :) = butter1array(a_record(ii,:), 200, 2);  % a2???
end
t = 1;
while t < length(a_record(1,:)-1)
    t = t + 1;
    T = a_record_all(6, t);
    
    a1 = a_record_lf(:, t);
    a2 = a_record_lf2(:,t);
    
%     a1 = a_record_lf(:, t);
%     a2 = a_record_lf2(:,t);
    if t>115
    w = w_record(:, t);
    else
        w= [0 0 0]';
    end
 
    a1 = a1 - a_offset;
%     a1 = a1 .* a_scale;
    a2 = a2 - a_offset;
%     a2 = a2 .* a_scale;
        record_a2 = [record_a2 (a2(1)^2+ a2(2)^2+ a2(3)^2)^0.5];
    
    % ****???,??
    if t <= N_g_cor
        a_T_1 = [0 0 0]';
        for i = 1:3
            a_initial(i) = mean(a_record(i, 2:N_g_cor));
        end
%         Quaternion_with_gyr( T, a_initial, w, t);
          Quaternion_with_PID1( T, a_initial, w, t);
          Quaternion_with_PID2( T, a_initial, w, t);
%         q_record_1 = [q_record_1 q];
        a_ave = a_record(1:2, t);
        yaw1(t) =  0;
    else
        h = h + 1;
        w = w - w_cor;
        if abs(w) < 0.001
            w = [0 0 0]';
        end
       
        % ????????
        if t > (N_g_cor + 20)  % ????
            if ~mod(t,50)
                for i = 1: 2
                    a_ave(i) = mean( a_record(i, (t-20):t) );  % ???
                end
            end
            a_stand_threshold = 0.15;
            if (a_ave(1) - a_stand_threshold)<a_record(1, t) & (a_ave(1) + a_stand_threshold)>a_record(1, t)...
                    & (a_ave(2) - a_stand_threshold)<a_record(2, t) & (a_ave(2) + a_stand_threshold)>a_record(2, t)
               state_count = state_count + 1;
            else
                state_count = 0;
            end
            if state_count > 19
                state = 0;
            else
                state = 1;
                Kp1 = 2;
                Kp2 = 0.2;
            end
        end
        
        if w_record(3, t) > 0.4  % ?????
            Kp1 = 0;
            Kp2 = 0;
        end
        
        Quaternion_with_PID1(T, a1, w, t);
        Quaternion_with_PID2(T, a1, w, t);
        
?
        
        yaw1(t) = yaw1(t-1) + w(3)*T; % ???,yaw
        yaw1_t = yaw1(t);
?
        eula = [pitch  roll yaw]';
        eula_record = [eula_record eula];
        
      % ??????????????????
        a_T_1 = DCMg1 * (a1) - g;
        a_T_2 = DCMg2 * (a1) - g;
        
        a_T_11 = (DCMg1 * (a2) - g);
        a_T_22 = (DCMg2 * (a2) - g);
        a_T_record_1 = [a_T_record_1 a_T_11];  % record v or a_T
        a_T_record_2 = [a_T_record_2 a_T_22];  % record v or a_T
        
        if (abs(a_T_22(1)) < 0.1) && (abs(a_T_22(2)) < 0.1)
            a_T = a_T_1;
        else
            a_T = a_T_2;
            if abs(a_T_22(1)) > 0.15
                detect_a = 1;
            elseif abs(a_T_22(2)) > 0.15
                detect_a = 2;
            end
        end
        a_T_record = [a_T_record a_T];
        
        % ????<-?????
        velocity_ins_1(:,h) = velocity_ins_1(:,h-1) + a_T_1 * T;
        velocity_ins_2(:,h) = velocity_ins_2(:,h-1) + a_T_2 * T;
        velocity_ins_b(:,h) = velocity_ins_b(:,h-1) + a_T * T;
        velocity_ins_f(:,h) = velocity_ins_f(:,h-1) + a_T * T;
        if detect_a == 1
            if  ( a_T_record_2(1,h) * a_T_record_2(1,h-1) ) < 0  % ????????
                count_a = 700;
                detect_a = 0;
                velocity_ins_b(:,h-1) = velocity_ins_b(:,h-50);
                velocity_ins_f(:,h) = velocity_ins_b(:,h);
            end
        elseif detect_a == 2
            if  ( a_T_record_2(2,h) * a_T_record_2(2,h-1) ) < 0  % ????????
                count_a =700;
                detect_a = 0;
                velocity_ins_b(:,h-1) = velocity_ins_b(:,h-50);
                velocity_ins_f(:,h) = velocity_ins_b(:,h) * 1.2;
            end
        end
        if count_a > 0
            count_a = count_a-1;
            velocity_ins_b(:,h) = velocity_ins_b(:,h-1);
        end
        
        velocity_ins_o(1,h) = cos(yaw1_t) * ( velocity_ins_f(1,h)*cos(yaw1_t) + velocity_ins_f(2,h)*sin(yaw1_t));
        velocity_ins_o(2,h) = sin(yaw1_t) * ( velocity_ins_f(1,h)*cos(yaw1_t) + velocity_ins_f(2,h)*sin(yaw1_t));
        % %  ????????
        if t < 20000
            velocity_ins_f(: ,h) = velocity_ins_o(:,h);
        end
    
        if state == 0
            velocity_ins_1(:,h) = [0 0 0]';
            velocity_ins_2(:,h) = [0 0 0]';
            velocity_ins_b(:,h) = [0 0 0]';
            velocity_ins_o(:,h) = [0 0 0]';
            velocity_ins_f(:,h) = [0 0 0]';
        end
        % ????<-????
        displacement_T_1 = velocity_ins_1(:,h) * T; 
        displacement_T_2 = velocity_ins_2(:,h) * T;
        displacement_T_b = velocity_ins_b(:,h) * T;
        displacement_T_o = velocity_ins_o(:,h) * T;
        displacement_T_f = velocity_ins_f(:,h) * T;
        
        % ??
        position_ins_1(:,h) = position_ins_1(:,h-1) + displacement_T_1;
        position_ins_2(:,h) = position_ins_2(:,h-1) + displacement_T_2;
        position_ins_b(:,h) = position_ins_b(:,h-1) + displacement_T_b;
        position_ins_o(:,h) = position_ins_o(:,h-1) + displacement_T_o;
        position_ins_f(:,h) = position_ins_f(:,h-1) + displacement_T_f;
   
        % ****INS?????
        
%       %  ??  % ?? ????
%         F_fu1 = [eye(3) diag([T T T]); zeros(3) eye(3)];
%         P_fu1 = F_fu1 * P_fu1 * F_fu1' + Q_fu1;  % ??????
      
%       %  ??  % ?? ????
%         F_fu = [eye(3) diag([T T T]); zeros(3) eye(3)];
%         P_fu = F_fu * P_fu * F_fu' + Q_fu;  % ??????
        
        % [P V]' = [1 T;0 1] * [P V]' + [T^2 T] * a;
?
    end
?
% UWB*****************
   if ( uwb_record(12, c) == a_record_all(8, t) ) && (c < length(uwb_record(1,:)))
        % % uwb_record??[k; j; T; T2; Y(:, k); 4; d'; t;];  a_record_all??[ a; j; k; T; T2; t;]?
        %  ??UWB?INS?????????
?
        % T = 0.005;  % ?????0.556s
        T = uwb_record(3, c);
        Y(:,c) =  uwb_record(u1:u2, c);  % Y????
?
        position_error = Y(:, c) - position_ins_f(1:2, h);
        
        if ( (t <= N_g_cor)  || (state == 0) )
?
          % ?????????????
             % ?? ????
            F_s = [1 0; 0 1]; H_s = [1 0;0 1]; R_s = [10 0;0 10]; Q_s = [0 0;0 0]; I_s = [1 0;0 1]; % R_s = [5e-5 0;0 5e-5]; 
            X1_s(:,c) = F_s * X_s(:,c-1);
            P_s = F_s * P_s * F_s' + Q_s; % Q? ???????
            % ?? ????
            Y_s(:,c) = Y(:,c);  % Y????
            K_s = (P_s*H_s') * pinv((H_s*P_s*H_s') + R_s);    % R? ???????????????????????????????K? ?????
            X_s(:,c) = X1_s(:,c) + K_s * ( Y_s(:,c) - H_s * X1_s(:,c) );
            P_s = (I_s - K_s*H_s) * P_s;
            
            if uwb_s < 1
                X_s(:, c) = uwb_record(u1:u2, c);
                X1_s(:, c) = X_s(:, c);
            end
            uwb_s = uwb_s + 1;  % ????????uwb????????
            
            position_ins_f(1:2, h) = X_s(:,c);  % ??????
            position_ins_f(3,h) = 0;
            velocity_ins_f(:,h) = [0 0 0]';
            X_fu(1:3) = position_ins_f(:, h);  % ??
            X_fu(4:6) = velocity_ins_f(:, h);  % ??
            Y_fu(:,c) = [Y(:,c); 0;];
%             position_ins_f(:,h) = Y_fu(:,c);
        elseif  ( position_error(1)^2 + position_error(2)^2 < (1^2)) % ??uwb????????
             % ????  ??c
            X_s(:, c) = uwb_record(u1:u2, c);
            X1_s(:, c) = X_s(:, c);
            uwb_s = 0;
            
?
        % ??---------------------------------------------------------
              % ???UWB?????????
?
            % ????(f:fuse) F_fu = [eye(3) diag([T T T]); zeros(3) eye(3)];  H_fu = [eye(3) zeros(3)];
            % ?? ????
            Y_fu(:,c) = [Y(:,c); 0];
         
            T_f = T;
            % ?? ????
            F_fu = [eye(3) diag([T T T]); zeros(3) eye(3)];
            P_fu = F_fu * P_fu * F_fu' + Q_fu;  % ??????
            % ?? ????
            X1_fu(:,c) = [position_ins_f(:,h); velocity_ins_f(:,h)];
            K_fu = P_fu * H_fu' * pinv(H_fu * P_fu * H_fu' + R_fu);  % ?????
            X_fu = X1_fu(:, c) + K_fu * ( Y_fu(:,c) - H_fu * X1_fu(:,c) );  % ????
            P_fu = ( I_fu - K_fu * H_fu ) * P_fu;  % ???????
            X_fu_record = [X_fu_record [X_fu; c]];
            
            % ???????????INS? ?????
            position_ins_f(:,h) = X_fu(1:3);  % ??
            velocity_ins_f(:,h) = X_fu(4:6);  % ??
            %--------------------------------------------------------------
        else  % ??-??uwb????????
            
            Y_fu(:,c) = [Y(:,c); 0];
            X_fu(1:3) = position_ins_f(:, h);  % ??
            X_fu(4:6) = velocity_ins_f(:, h);  % ??
            X_fu_record = [X_fu_record [X_fu; c]];
            
            % ????  ??c
            X_s(:, c) = uwb_record(u1:u2, c);
            X1_s(:, c) = X_s(:, c);
            uwb_s = 0;
        end
        
        y2 = [y2 [t state]'];
        Display_trajectory(uwb_record(u1:u2, c),2);
        Display_trajectory(position_ins_f(1:2,h),1);
        c = c + 1;
   end
    % **************************
    
    % **************************
End
?
?


function Quaternion_with