function [ time_record ] = Get_time

time_record = datestr(now,'mmmm dd,yyyy HH:MM:SS.FFF AM');
length_t = length(time_record);
second = time_record(length_t-8:length_t-3);    % second
minute = time_record(length_t-11:length_t-10);  % minute
hour = time_record(length_t-15:length_t-13);    % hour

second = str2double(second);
minute = str2double(minute);
hour = str2double(hour);

end