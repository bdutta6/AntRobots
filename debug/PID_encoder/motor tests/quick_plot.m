% data = csvread('changing_ramp_test_4.csv');
% time = data(:,1);
% set_pos = data(:,2);
% act_pos = data(:,3);
% dT = diff(time);
% dset_pos = diff(set_pos)./dT;
% hold on
% plot(time,set_pos,'--b',time,act_pos,'-r')


caChangeTime = cell(1,5);
for p = 6:10
    charM = num2str(p);
    data = csvread(['changing_ramp_test_' charM '.csv']);
    caChangeTime{p-5} = data;
end

f1 = figure('Name','Changing Ramp by Time Tests','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caChangeTime{a}(:,1)./1000;
    setPos = caChangeTime{a}(:,2);
    actualPos = caChangeTime{a}(:,3);
    switch a
        case 1
            color = 'c';
        case 2
            color = 'm';
        case 3
            color = 'g';
        case 4
            color = 'b';
        case 5
            color = 'r';
        case 6
            color = 'k';
    end
    plot(time,setPos,['--' color],time,actualPos,color);
end
xlabel('time (sec)')
ylabel('position (rev)')
hold off

caChangePos = cell(1,5);
for p = 1:5
    charM = num2str(p);
    data = csvread(['change_by_pos_ramp_test_' charM '.csv']);
    caChangePos{p} = data;
end

f2 = figure('Name','Changing Ramp By Position Tests (position vs time)','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caChangePos{a}(:,1)./1000;
    setPos = caChangePos{a}(:,2);
    actualPos = caChangePos{a}(:,3);
    switch a
        case 1
            color = 'c';
        case 2
            color = 'm';
        case 3
            color = 'g';
        case 4
            color = 'b';
        case 5
            color = 'r';
        case 6
            color = 'k';
    end
    plot(time,setPos,['--' color],time,actualPos,color);
end
xlabel('time (sec)')
ylabel('position (rev)')
hold off


f3 = figure('Name','Changing Ramp By Position Tests (set position vs encoder position)','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caChangePos{a}(:,1)./1000;
    setPos = caChangePos{a}(:,2);
    actualPos = caChangePos{a}(:,3);
    switch a
        case 1
            color = 'c';
        case 2
            color = 'm';
        case 3
            color = 'g';
        case 4
            color = 'b';
        case 5
            color = 'r';
        case 6
            color = 'k';
    end
    plot(actualPos,setPos,['-' color]);
end
xlabel('encoder position (rev)')
ylabel('set position (rev)')
hold off