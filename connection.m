clc,clear,close
com = serial('COM5','BaudRate',115200);
% fclose(com);
fopen(com);
relN=[];
relE=[];
time=[];
vers=[];
id=[];
reserv=[];
sec=[];
while (length(relN)~=40)                                  
    if fread(com,1,'uint8') == 0xB5 
        if fread(com,1,'uint8') == 0x62
            if fread(com,1,'uint8') == 0x01
                if fread(com,1,'uint8') == 0x3C
                    fread(com,2,'uint8'); %len etc
                    fread(com,4,'uint8'); % vers etc
                    time=[time round(fread(com,1,'uint32')/1000)]; %itow
                    relN=[relN fread(com,1,'int32')];
                    relE=[relE fread(com,1,'int32')];
%                     plot(relN,relE)
%                     drawnow
                end
            end
        end
    end
end
fclose(com);