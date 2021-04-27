clear;
clc;
s = serial('COM5');
set(s,'BaudRate',921600,'StopBits',1,'Parity','none','DataBits',8,'FlowControl','none');%设置波特率  停止位  校验位 
fopen(s);

global q  % 四元数q = [q0 q1 q2 q3]
global t
global T1
global vv  % record v or a_T
global coordinate  % actual coordinate of the moving object
global integral_e_a
global g  % 重力加速度
global N_g_cor % 校正步数
global a_cor  % 加速度校正参数
global w_cor  % 角速度校正参数
global w_record
q = [1 0 0 0];
t = 0;
T1 = 0;
vv = [0 0 0]';
integral_e_a = [0 0 0];
g = [0 0 0]';
a_cor = 0;
N_g_cor = 2000;
% w_cor = [-0.0184 -0.0135 -0.0117];  % 漂移由5s,3m降到1m；还能进一步校正？？？？？？？？
coordinate = [0 0 0]';

global t1

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
i = 50/0.01;
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
            Ins_get_trajectory_accurate(temp2);
            % Complementary_filter(temp2);
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