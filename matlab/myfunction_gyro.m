function [res] = myfunction_gyro(data)
if (length(data)==16)
        otr = data;
        otr = dec2bin(bin2dec(otr)-bin2dec('1'));
    for j=1:16
        if (otr(1,j)=='1')
            otr(1,j)='0';
        else
            otr(1,j)='1'; 
        end;
    end;
    %otr
    res=(-1)*bin2dec(otr);
    else
        res=bin2dec(data);
end;