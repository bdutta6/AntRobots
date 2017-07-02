
caStep4 = cell(1,5);
for a = 1:5
    str = num2str(a);
    dataRead = csvread(['4_motor_step_test_' str '.csv']);
    for b = 1:4
        switch b
            case 1
                data1 = dataRead(:,1:3);
            case 2
                data1 = dataRead(:,4:6);
            case 3
                data1 = dataRead(:,7:9);
            case 4
                data1 = dataRead(:,10:12);
        end
        caStep4{a}{b} = data1;
    end
end

f1 = figure('Name','4 Motors Set Positions = 10,15,20,25','NumberTitle','off');
hold on
for x = 1:5
    switch x
        case 1
            color = 'k';
        case 2
            color = 'm';
        case 3
            color = 'g';
        case 4
            color = 'b';
        case 5
            color = 'r';
    end
    for y = 1:4
        switch y
            case 1
                marker = '-';
            case 2
                marker = '-';
            case 3
                marker = '-';
            case 4
                marker = '-';
        end
        data2 = caStep4{x}{y};
        time = data2(:,1);
        set_pos = data2(:,2);
        act_pos = data2(:,3);
        plot(time,set_pos,[color marker '-'],time,act_pos,[color marker]);
    end     
end
xlabel('time (ms)')
ylabel('position (rev)')
hold off

caRamp4 = cell(1,5);
for a = 1:5
    str = num2str(a);
    dataRead = csvread(['4_motor_ramp_test_' str '.csv']);
    for b = 1:4
        switch b
            case 1
                data1 = dataRead(:,1:3);
            case 2
                data1 = dataRead(:,4:6);
            case 3
                data1 = dataRead(:,7:9);
            case 4
                data1 = dataRead(:,10:12);
        end
        caRamp4{a}{b} = data1;
    end
end

f1 = figure('Name','4 Motors Ramp Slopes = 0.5,1.0,1.5,2.0','NumberTitle','off');
hold on
for x = 1:5
    switch x
        case 1
            color = 'k';
        case 2
            color = 'm';
        case 3
            color = 'g';
        case 4
            color = 'b';
        case 5
            color = 'r';
    end
    for y = 1:4
        switch y
            case 1
                marker = '-';
            case 2
                marker = '-';
            case 3
                marker = '-';
            case 4
                marker = '-';
        end
        data2 = caRamp4{x}{y};
        time = data2(:,1);
        set_pos = data2(:,2);
        act_pos = data2(:,3);
        plot(time,set_pos,[color marker '-'],time,act_pos,[color marker]);
    end     
end
xlabel('time (ms)')
ylabel('position (rev)')
hold off