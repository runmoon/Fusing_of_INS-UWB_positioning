clear;
clc;
global x_act  % 记录实际路径
global y_act
global Vx  % 实际速度
global Vy
global eq_tr  % 基站（等边三角形）的边长

Vx = -0.15^0.5; Vy = -0.15^0.5;  % 沿x/y方向运动
x_act = 6; y_act = 1.78;  % 
% Vx = 0; Vy = 0.15;  % 沿y方向运动
% x_act = 1.25; y_act = 0;
eq_tr = 12;

s = serial('COM5');
set(s,'BaudRate',921600,'StopBits',1,'Parity','none','DataBits',8,'FlowControl','none');%设置波特率  停止位  校验位 
fopen(s);

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
fprintf(FileID,'%f',x_act); fprintf(FileID,'%c',' ');
fprintf(FileID,'%f',y_act); fprintf(FileID,'%c',' ');
fprintf(FileID,'%f',Vx); fprintf(FileID,'%c',' ');
fprintf(FileID,'%f',Vy); fprintf(FileID,'%c',' ');
fprintf(FileID,'%f',eq_tr); fprintf(FileID,'\r\n');

s.ReadAsyncMode = 'continuous';
i = 50 / (0.556/4);
tic
'start___________________________________________'
while(i>0)
    if(s.BytesAvailable)
        Temp_1 = fscanf(s,'%c');
        Temp_2 = Temp_1';
        fprintf(FileID,'%c',Temp_2);
        i = i - 1;
    end
end
'end--------------------------------------------'
toc
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