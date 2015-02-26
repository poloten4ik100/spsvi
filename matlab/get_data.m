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
figure(1);
hold on;

K=0.1;
t=0;
roll = 0;
pitch =0;
roll_g= 0;
pitch_g = 0;
filter_roll = 0;
filter_pitch = 0;
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
pred_tmp_roll = 0;
pred_tmp_pitch = 0;
pred_tmp_rollg = 0;
pred_tmp_pitchg = 0;
filter_roll_tmp = 0;
filter_pitch_tmp =0;
fprintf(s,'');
while(1)
    tic;
    t=t+1;
    out=fread(s);
    
    subplot(231);
    plot([t-1 t], [pred_tmp_x x_data],'b',[t-1 t],[pred_tmp_x_gyro x_data_gyro],'g');
    title('Ось X');
    hold on;
    drawnow;
    axis([t-100 t -256 256]);
    
    
    subplot(232);
    plot([t-1 t], [pred_tmp_y y_data], 'b',[t-1 t],[pred_tmp_y_gyro y_data_gyro],'g');
    title('Ось Y');
    hold on;
    axis([t-100 t -512 512]);
    %drawnow;
    
    subplot(233);
    plot([t-1 t], [pred_tmp_z z_data], 'b',[t-1 t],[pred_tmp_z_gyro z_data_gyro],'g');
    title('Ось Z');
    hold on;
    axis([t-100 t -512 512]);
    %drawnow;
    
    subplot(234);
    plot([t-1 t], [pred_tmp_x x_data], 'm', [t-1 t], [pred_tmp_y y_data], 'c', [t-1 t], [pred_tmp_z z_data], 'b');
    title('Все вместе (акселерометр)');
    hold on;
    axis([t-100 t -512 512]);
    %drawnow;
    
    subplot(235);
    plot([t-1 t], [pred_tmp_roll roll], 'r', [t-1 t], [pred_tmp_rollg roll_g], 'black', [t-1 t], [filter_roll_tmp filter_roll], 'b');
    title('roll (крен)');
    xlabel('t');
    ylabel('a,°');
    hold on;
    axis([t-100 t -180 180]);
    %drawnow;
    
    subplot(236);
    plot([t-1 t], [pred_tmp_pitch pitch], 'r',[t-1 t], [pred_tmp_pitchg pitch_g], 'black',[t-1 t], [filter_pitch_tmp filter_pitch], 'b');
    title('pitch (тангаж)');
    xlabel('t');
    ylabel('a,°');
    hold on;
    axis([t-100 t -180 180]);
    
    
    pred_tmp_x = x_data;
    pred_tmp_y = y_data;
    pred_tmp_z = z_data;
    pred_tmp_x_gyro = x_data_gyro;
    pred_tmp_y_gyro = y_data_gyro;
    pred_tmp_z_gyro = z_data_gyro;
    pred_tmp_roll = roll;
    pred_tmp_pitch = pitch;
    pred_tmp_rollg = roll_g;
    pred_tmp_pitchg = pitch_g;
    filter_roll_tmp = filter_roll;
    filter_pitch_tmp = filter_pitch;
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
    x_a = x_data*0.03125;
    
    tmp=uint8([out_byte_3 out_byte_4]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    y_data = myfunction(result);
    y_a = y_data*0.03125;    
    
    tmp=uint8([out_byte_5 out_byte_6]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    z_data = myfunction(result);
    z_a = z_data*0.03125;
    
    tmp=uint8([out_byte_7 out_byte_8]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    x_data_gyro = myfunction_gyro(result)*0.07;
    
    tmp=uint8([out_byte_9 out_byte_10]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    y_data_gyro = myfunction_gyro(result)*0.07;
    
    tmp=uint8([out_byte_11 out_byte_12]);
    tmp=typecast(tmp,'uint16');
    result = dec2bin(tmp);
    z_data_gyro = myfunction_gyro(result)*0.07;
    
    %roll [-180, 180] range
    roll = (atan2(y_a ,z_a) * 180) / pi;
    %pitch [-90, 90] range
    pitch = (atan2(x_a ,sqrt(z_a*z_a + y_a*y_a)) * 180) / pi
    
    ti = toc;
    roll_g = roll_g + x_data_gyro*ti;
    pitch_g = pitch_g + y_data_gyro*ti
    ti = 0;
    filter_roll = (1-K)*(roll_g)+(K*roll);   
    filter_pitch = (1-K)*(pitch_g)+(K*pitch);
    
    fprintf(s,'');
    
end;

fclose(s);
delete(s);
clear s;