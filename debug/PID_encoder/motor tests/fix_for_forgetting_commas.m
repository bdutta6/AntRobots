% forgot to place commas in serial print. this takes those raw csv numbers 
% and places them into an excel file where the values are under separate
% columns

filenameStep = 'ramp_test_';
for n = 1:5
    charN = num2str(n);
    [~,~,raw] = xlsread([filenameStep charN '.csv']);
    tArr = zeros(length(raw),1);
    set_posArr = zeros(length(raw),1);
    act_posArr = zeros(length(raw),1);
    for x = 1:length(raw)
        cellVal = raw{x};
        [tStr,rest] = strtok(cellVal);
        t = str2double(tStr);
        [setStr,actStr] = strtok(rest);
        set_pos = str2double(setStr);
        act_pos = str2double(actStr);
        tArr(x) = t;
        set_posArr(x) = set_pos;
        act_posArr(x) = act_pos;
    end
    matrix = [tArr set_posArr act_posArr];
    xlswrite([filenameStep charN '.xls'],matrix);
end
    
    