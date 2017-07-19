caStep = cell(1,13);
for a = 1:13
    charM = num2str(a);
    data = csvread(['new_gains_step_test_' charM '.csv']);
    caStep{a} = data;
end

f2 = figure('Name','Different gains (focused on tuning kp)','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 1:8
    time = caStep{a}(:,1)./1000;
    setPos = caStep{a}(:,2);
    actualPos = caStep{a}(:,3);
    switch a
        case 1
            color = 'b:'; %kp = 16, kd = 300
        case 2
            color = 'k:'; %kp = 16, kd = 200
        case 3
            color = 'g'; %kp = 20, kd = 400
        case 4
            color = 'b'; %kp = 24, kd = 400
        case 5
            color = 'r'; %kp = 28, kd = 400
        case 6
            color = 'k'; %kp = 16, kd = 400
        case 7
            color = 'c'; %kp = 12, kd = 400
        case 8
            color = 'm'; %kp = 10, kd = 400
%         case 9
%             color = 'g'; %kp = 11, kd = 400
%         case 10
%             color = 'b'; %kp = 11, kd = 500
    end
    plot(time,setPos,['--' color(1)],time,actualPos,color);
end
xlabel('time (sec)')
ylabel('position (rev)')
hold off

f3 = figure('Name','Different gains (focused on tuning kd)','NumberTitle','off'); %, 'Color', [0.75 0.75 0.75]);
hold on
for a = 9:13
    time = caStep{a}(:,1)./1000;
    setPos = caStep{a}(:,2);
    actualPos = caStep{a}(:,3);
    switch a
%         case 1
%             color = 'w'; %kp = 16, kd = 300
%         case 2
%             color = 'w'; %kp = 16, kd = 200
%         case 3
%             color = 'w'; %kp = 20, kd = 400
%         case 4
%             color = 'b'; %kp = 24, kd = 400
%         case 5
%             color = 'r'; %kp = 28, kd = 400
%         case 6
%             color = 'k'; %kp = 16, kd = 400
%         case 7
%             color = 'c'; %kp = 12, kd = 400
%         case 8
%             color = 'm'; %kp = 10, kd = 400
        case 9
            color = 'c'; %kp = 11, kd = 400
        case 10
            color = 'm'; %kp = 11, kd = 500
        case 11
            color = 'g'; %kp = 11, kd = 425
        case 12
            color = 'b'; %kp = 13, kd = 425
        case 13
            color = 'r'; %kp = 13, kd = 435
    end
    plot(time,setPos,['--' color(1)],time,actualPos,color);
end
xlabel('time (sec)')
ylabel('position (rev)')
hold off