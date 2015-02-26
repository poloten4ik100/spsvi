clc,clear,close all;
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end;
% s=serial('COM3');%Соединение с ком портом
% set(s,'BaudRate',9600,'DataBits',8,'StopBits',1,'Timeout',1,'InputBufferSize',3,'OutputBufferSize',2,'Terminator','CR');
% fopen(s);%Подключение к устройству, подключенному к ком порту
% %Инициирование переменных
% fprintf(s,'');
while(1)
%     out=fread(s);
%     out_byte_1 = out(1);
%     out_byte_2 = out(2);
%     tmp=uint8([out_byte_1 out_byte_2]);
%     tmp=typecast(tmp,'uint16');
    result = '1111111000001101';
    if (length(result)>=10)
        otr = result;
        for j=1:16
        if (otr(1,j)=='1')
            otr(1,j)='0';
        else
            otr(1,j)='1'; 
        end;
        end;
        otr = dec2bin(bin2dec(otr)+bin2dec('1'));
        otv = (-1)*bin2dec(otr)
    else 
        otv = bin2dec(result)
    end;
%     fprintf(s,'');
end;
% fclose(s);
% delete(s);
% clear s;