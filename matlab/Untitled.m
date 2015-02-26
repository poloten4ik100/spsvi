clc,clear,close all;
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end;
s=serial('COM3');%Соединение с ком портом
set(s,'BaudRate',9600,'DataBits',8,...
'StopBits',1,'Timeout',1,'InputBufferSize',7,...
'OutputBufferSize',7,'Terminator','z');%Настройка параметров()
fopen(s);%Подключение к устройству, подключенному к ком порту
%Инициирование переменных
j=0;
l=0;
t=0;
Data=[];
TranslateInGaus=1;
PastData(1)=0;
PastData(2)=0;
PastData(3)=0;
DataT=['' '';'' '';'' '';'' '';'' '';'' ''];
DataTT=['' '' '' '';'' '' '' '';'' '' '' ''];
DataTTT=zeros(3,1);
temp1='0000000000000000';
temp='000000000000';
while(1)
%% ReciveData
          %out=fscanf(s,'%s');%Считать символьные с устройства в переменную out
          out=fread(s);
          OUT=abs(out);%Получение ASCII кода символов
          %OUT=dec2hex(uint8(out));
%           for i=1:length(OUT)%Обработка символов ASCII больше 127
%               if OUT(i)>=976
%                   OUT(i)=OUT(i)-848;
%               end;
%           end;
          OUT=dec2hex(OUT);
          Data=[Data;OUT];
%% ChekOnStop
          for i=1:length(out);
                    if ((out(i)=='s')...
                      (out(i+1)=='t')...
                      &&(out(i+2)=='o')...
                      &&(out(i+3)=='p'));
                    l=1;
                    end;
          end;
          if (l==1)
                    break;
          end;
%% TransformAndVisualizationData
          if (length(out)>6)
%место для обработки данных с датчиков
                    for i=1:1:6
                              DataT(i,1)=OUT(i,1);
                              DataT(i,2)=OUT(i,2);
                    end;
                    j=1;
%Перевод в 16 битное число
                    for i=1:1:3
                              DataTT(i,1)=DataT(j,1);
                              DataTT(i,2)=DataT(j,2);
                              DataTT(i,3)=DataT(j+1,1);
                              DataTT(i,4)=DataT(j+1,2);
                              j=j+2;
                    end;
                    AntiDopCode='000000000001';
%Проверка на дополнительный код и перевод в десятичное число
                    for i=1:1:3
                                %if (hex2dec([DataTT(i,1) DataTT(i,2) DataTT(i,3) DataTT(i,4)])>2047)%((DataTT(i,1)>='0' && DataTT(i,2)>'8' && DataTT(i,3)>'0' && DataTT(i,4)>'0'))
                                        temp1=dec2bin(hex2dec([DataTT(i,1) DataTT(i,2) DataTT(i,3) DataTT(i,4)]),16);
                                        for k=1:12
                                                    temp(k)=temp1(1,k);
                                        end;
                                if (temp(1,1)=='1')
                                        for j=1:12
                                                    if (temp(1,j)=='1')
                                                            temp(1,j)='0';
                                                    else
                                                            temp(1,j)='1'; 
                                                    end;
                                        end;
                                        temp(1,1)='0';
                                        temp2=temp;
                                        DataTTT(i)=(-1).*(bin2dec(temp)+bin2dec(AntiDopCode));   
                                else
                                        for k=1:12%Разобраться, тк цикл заканчивается только последним числом
                                            temp(k)=temp1(1,k);
                                        end;
                                        DataTTT(i)=bin2dec(temp);
                                end;
                    end;
% %Визуализация
%                     figure(1)
%                     %plot3(0,0,0,'-o','MarkerEdgeColor','k','MarkerFaceColor','k');
%                                 %if (( (PastData(1)~=DataTTT(1)) && (PastData(2)~=DataTTT(2)) && (PastData(3)~=DataTTT(3))))
%                                         plot3([0 (DataTTT(1)./TranslateInGaus)],[0 (DataTTT(3)./TranslateInGaus)],[0 (DataTTT(2)./TranslateInGaus)],'-rx');
%                                         hold on
%                                         plot3(0,0,0,'-o','MarkerEdgeColor','k','MarkerFaceColor','k');
%                                         hold off
%                                         grid on
%                                         xlabel('X');
%                                         ylabel('Y');
%                                         zlabel('Z');
%                                         axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
%                                         %axis([-100 100 -100 100 -100 100]);
%                                         axis([-2048 2048 -2048 2048 -2048 2048]);
%                                         %PastData(1)=DataTTT(1);
%                                         %PastData(2)=DataTTT(2);
%                                         %PastData(3)=DataTTT(3);
%                                 %end;
%                                 %refresh(1);
figure(1);
t=t+t;
d = DataTTT(1)
plot(DataTTT(1));
          end;
          fprintf(s,'');
end;
fclose(s);
delete(s);
clear s;