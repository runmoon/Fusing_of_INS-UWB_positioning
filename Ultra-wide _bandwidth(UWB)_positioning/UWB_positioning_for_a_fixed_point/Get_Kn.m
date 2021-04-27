function [ Kn ] = Get_Kn( n )
% n是常量，代表horizon

g_t = 0.556;  % 时间间隔，实际的间隔是不稳定的！！！！！！！！！！！！！？？？？？
% 有时候0.79，有时候0.55

% x = F*x + Q;  x = [Px Vx Py Vy]'
% y = C*x + P;  y = [Px Py]';
A = [1 g_t 0 0;0 1 0 0;0 0 1 g_t;0 0 0 1];  % t-1时刻到t时刻的状态X的转移矩阵，gt是时间间隔
d_A = 4;  % A的行（或列）数
C = [1 0 0 0;0 0 1 0];    % C为观测转换矩阵
% Q = [1e-5 0 0 0;0 1e-5 0 0;0 0 1e-5 0;0 0 0 1e-5];  % Q为过程噪声
% R = [0.95 0;0 0.95];   % R为观测噪声
Q = [1e-5 0 0 0;0 1e-5 0 0;0 0 1e-5 0;0 0 0 1e-5];  % Q为过程噪声
R = [0.97 0;0 0.97];   % R为观测噪声
Q = [1e-5 1e-5 0 0;1e-5 2e-4 0 0;0 0 1e-5 1e-5;0 0 1e-5 2e-4];  % Q为过程噪声
R = [0.97 0;0 0.97];   % R为观测噪声

% 曲线
% Q = [1e-5 0 0 0;0 1e-4 0 0;0 0 1e-5 0;0 0 0 1e-4];  % Q为过程噪声
% R = [1e-1 0;0 1e-1];  % R为观测噪声
% Q = [1e-2 1e-3 0 0;1e-3 1e-1 0 0;0 0 1e-2 1e-3;0 0 1e-3 1e-1];  % Q为过程噪声
% R = [2e-1 0;0 2e-1];  % R为观测噪声

I = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
I_m = eye(2 * n);  % 2为y的元素数（即行数）
%  应该Q和R的取值不必要很准确,Q = 1e-7; R = 5e-5; 

Qd = Q;  % 构造 n*某 的多维对角矩阵
Rd = R;
Cd = C;
for temp1 = 1:n-1
    Qd = blkdiag(Qd,Q);
    Rd = blkdiag(Rd,R);
    Cd = blkdiag(Cd,C);
end
An = I;  % 构造An,Bn
Bn = I;
for temp1 = 1:n-1
    An = [(A^temp1)' An];
    Bn = blkdiag(Bn,I);
    Bn(:,(d_A*temp1+1):d_A*(temp1+1)) = An';
end
An = An';

Cn = Cd * An;
Hn = Cd * Bn;
Zpq = Hn * Qd * Hn' + Rd;
Zn = Cn' * pinv(Zpq) * Cn;

K1 = A^(n-1) * pinv(Zn) * Cn';
K2 = Bn(1:d_A,:) * Qd * Hn' * pinv(Zpq) * (I_m - Cn * pinv(Zn) * Cn' * pinv(Zpq));
Kn = K1 + K2;

end