% 二维字符串元胞{'a5' 'f5';'34' '6d'}
% 十六进制转十进制
function dec = hex_dec(hex)
size_of = size(hex);
dec = [];
for i = 1:size_of(1)
    for j = 1:size_of(2)
        dec(i,j) = sscanf(hex{i,j},'%X');
        if dec(i,j) >= 32768
            dec(i,j) = -(32767 - (dec(i,j) - 32768));
        end
    end
end
end