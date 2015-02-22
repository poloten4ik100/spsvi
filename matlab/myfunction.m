function [dec] = myfunction(data)
if (length(data)>=10)
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
    dec=(-1)*bin2dec(otr);
    else
        dec=bin2dec(data);
    end;