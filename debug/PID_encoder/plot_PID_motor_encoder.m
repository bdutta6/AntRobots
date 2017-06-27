caStep = cell(1,3);
for n = 1:15
    charN = num2str(n);
    [data,~,~] = xlsread(['step_test_' charN '.xls']);
    if n <= 5
        % these trials are with a setpoint of 5 revolutions
        caStep{1}{n} = data;
    elseif n <= 10
        % these trials are with a setpoint of 10 revolutions
        caStep{2}{n-5} = data;
    else
        % these trials are with a setpoint of 15 revolutions
        caStep{3}{n-10} = data;
    end
end

f1 = figure('Name','Set Position = 5 rev','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caStep{1}{a}(:,1);
    setPos = caStep{1}{a}(:,2);
    actualPos = caStep{1}{a}(:,3);
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
    end
    plot(time,setPos,['--' color],time,actualPos,color);
end
xlabel('time (ms)')
ylabel('position (rev)')
hold off

f2 = figure('Name','Set Position = 10 rev','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caStep{2}{a}(:,1);
    setPos = caStep{2}{a}(:,2);
    actualPos = caStep{2}{a}(:,3);
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
    end
    plot(time,setPos,['--' color],time,actualPos,color);
end
xlabel('time (ms)')
ylabel('position (rev)')
hold off


f3 = figure('Name','Set Position = 15 rev','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caStep{3}{a}(:,1);
    setPos = caStep{3}{a}(:,2);
    actualPos = caStep{3}{a}(:,3);
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
    end
    plot(time,setPos,['--' color],time,actualPos,color);
end
xlabel('time (ms)')
ylabel('position (rev)')
hold off

caRamp = cell(1,5);
for m = 1:5
    charM = num2str(m);
    [data,~,~] = xlsread(['ramp_test_' charM '.xls']);
    caRamp{m} = data;
end

f4 = figure('Name','Ramp Test','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:5
    time = caRamp{a}(:,1);
    setPos = caRamp{a}(:,2);
    actualPos = caRamp{a}(:,3);
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
    end
    plot(time,setPos,['--' color],time,actualPos,color);
end
xlabel('time (ms)')
ylabel('position (rev)')
hold off