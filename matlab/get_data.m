clc,clear,close all;
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end;

s=serial('COM3');%Соединение с ком портом
set(s,'BaudRate',9600,'DataBits',8,'StopBits',1,'Timeout',1,'InputBufferSize',13,'OutputBufferSize',2,'Terminator','CR');
fopen(s);%Подключение к устройству, подключенному к ком порту
%Инициирование переменных
%t=1;
figure;
hold on;

t=0;
x_data = 0;
y_data = 0;
z_data = 0;
x_data_gyro = 0;
y_data_gyro = 0;
z_data_gyro = 0;
pred_tmp_x = 0;
pred_tmp_y = 0;
pred_tmp_z = 0;
pred_tmp_x_gyro =0;
pred_tmp_y_gyro =0;
pred_tmp_z_gyro =0;
fprintf(s,'');
while(1)
    t=t+1;
    out=fread(s);
    
    subplot(221);
    plot([t-1 t], [pred_tmp_x x_data],'b',[t-1 t],[pred_tmp_x_gyro x_data_gyro],'g');
    title('Ось X');
    hold on;
    drawnow;
    axis([t-100 t -512 512]);
    
    
    subplot(222);
    plot([t-1 t], [pred_tmp_y y_data], 'b',[t-1 t],[pred_tmp_y_gyro y_data_gyro],'g');
    title('Ось Y');
    hold on;
    axis([t-100 t -512 512]);
    %drawnow;
    
    subplot(223);
    plot([t-1 t], [pred_tmp_z z_data], 'b',[t-1 t],[pred_tmp_z_gyro z_data_gyro],'g');
    title('Ось Z');
    hold on;
    axis([t-100 t -512 512]);
    %drawnow;
    
    subplot(224);
    plot([t-1 t], [pred_tmp_x x_data], 'r', [t-1 t], [pred_tmp_y y_data], 'g', [t-1 t], [pred_tmp_z z_data], 'b');
    title('Все вместе (акселерометр)');
    hold on;
    axis([t-100 t -512 512]);
    %drawnow;
    
    pred_tmp_x = x_data;
    pred_tmp_y = y_data;
    pred_tmp_z = z_data;
    pred_tmp_x_gyro = x_data_gyro;
    pred_tmp_y_gyro = y_data_gyro;
    pred_tmp_z_gyro = z_data_gyro;
    out_byte_1 = out(1);
    out_byte_2 = out(2);
    out_byte_3 = out(3);
    out_byte_4 = out(4);
    out_byte_5 = out(5);
    out_byte_6 = out(6);
    out_byte_7 = out(7);
    out_byte_8 = out(8);
    out_byte_9 = out(9);
    out_byte_10 = out(10);
    out_byte_11 = out(11);
    out_byte_12 = out(12);
    
    tmp=uint8([out_byte_1 out_byte_2]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    x_data = myfunction(result); 
    
    tmp=uint8([out_byte_3 out_byte_4]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    y_data = myfunction(result);
    
    tmp=uint8([out_byte_5 out_byte_6]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    z_data = myfunction(result);
    
    tmp=uint8([out_byte_7 out_byte_8]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    x_data_gyro = myfunction_gyro(result)*0.07
    
    tmp=uint8([out_byte_9 out_byte_10]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    y_data_gyro = myfunction_gyro(result)*0.07
    
    tmp=uint8([out_byte_11 out_byte_12]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    z_data_gyro = myfunction_gyro(result)*0.07
    
    fprintf(s,'');
end;

fclose(s);
delete(s);
clear s;