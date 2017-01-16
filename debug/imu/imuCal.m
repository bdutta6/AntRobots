clear; clc; close all;

m = csvread('echoIMU.csv', 1);

xMin = min(m(:,1));
yMin = min(m(:,2));

xMax = max(m(:,1));
yMax = max(m(:,2));

% Compute the mapping
xNew = (m(:,1)-xMin)*(2/(xMax-xMin))-1;
yNew = (m(:,2)-yMin)*(2/(yMax-yMin))-1;

% Compute the unit circle points
x = 0;
y = 0;
r = 1;

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;

% figure
% hold on
% h = plot(xunit, yunit);
% grid on
% hold off


%% Plot unit circle mapping
figure
plot(xunit, yunit)
grid on
hold on
% hold off

% figure
plot(m(:,1), m(:,2), '.b')
grid on
hold on
plot(xNew, yNew, '.r')


%% Compute Headings stuff -- not used
hVals = zeros(length(xunit), 1);
for i = 1:length(xunit)
    hVals(i) = getHeading(xunit(i), yunit(i));
end


headings = zeros(length(m),1 );
headings2 = zeros(length(m),1 );
for i = 1:length(m)
    headings(i) = getHeading(m(i,1), m(i,2));
    headings2(i) = getHeading(xNew(i), yNew(i));
end

figure
plot(xunit, hVals)
xlabel('hx')
ylabel('Heading')
hold on
plot(xNew, headings, '.')
plot(xNew, headings2, '.')
legend('Unit Circle', 'Uncalibrated' , 'Calibrated')






%% Generate output for contants files

fprintf('////////////////////////////////////////////// IMU Bias Parameters //////////////////////////////////////////////\n',xMin);

fprintf('#define HX_MIN %6.4f\n',xMin);
fprintf('#define HX_MAX %6.4f\n\n',xMax);

fprintf('#define HY_MIN %6.4f\n',yMin);
fprintf('#define HY_MAX %6.4f\n',yMax);




