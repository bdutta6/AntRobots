clear; clc; close all;

a = csvread('alphaIMU.csv', 1);
b = csvread('bravoIMU.csv', 1);
c = csvread('charlieIMU.csv', 1);
d = csvread('deltaIMU.csv', 1);
e = csvread('echoIMU.csv', 1);

%% Get the mins and maxes for a
xMinA = min(a(:,1));
yMinA = min(a(:,2));

xMaxA = max(a(:,1));
yMaxA = max(a(:,2));

% Compute the mapping
xNewA = (a(:,1)-xMinA)*(2/(xMaxA-xMinA))-1;
yNewA = (a(:,2)-yMinA)*(2/(yMaxA-yMinA))-1;

%% Get the mins and maxes for b
xMinB = min(b(:,1));
yMinB = min(b(:,2));

xMaxB = max(b(:,1));
yMaxB = max(b(:,2));

% Compute the mapping
xNewB = (b(:,1)-xMinB)*(2/(xMaxB-xMinB))-1;
yNewB = (b(:,2)-yMinB)*(2/(yMaxB-yMinB))-1;

%% Get the mins and maxes for c
xMinC = min(c(:,1));
yMinC = min(c(:,2));

xMaxC = max(c(:,1));
yMaxC = max(c(:,2));

% Compute the mapping
xNewC = (c(:,1)-xMinC)*(2/(xMaxC-xMinC))-1;
yNewC = (c(:,2)-yMinC)*(2/(yMaxC-yMinC))-1;

%% Get the mins and maxes for d
xMinD = min(d(:,1));
yMinD = min(d(:,2));

xMaxD = max(d(:,1));
yMaxD = max(d(:,2));

% Compute the mapping
xNewD = (d(:,1)-xMinD)*(2/(xMaxD-xMinD))-1;
yNewD = (d(:,2)-yMinD)*(2/(yMaxD-yMinD))-1;

%% Get the mins and maxes for e
xMinE = min(e(:,1));
yMinE = min(e(:,2));

xMaxE = max(e(:,1));
yMaxE = max(e(:,2));

% Compute the mapping
xNewE = (e(:,1)-xMinE)*(2/(xMaxE-xMinE))-1;
yNewE = (e(:,2)-yMinE)*(2/(yMaxE-yMinE))-1;

%% Compute the unit circle points
x = 0;
y = 0;
r = 1;

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;

%% Plot unit circle mapping
figure
plot(xunit, yunit, a(:,1), a(:,2), '+', b(:,1), b(:,2), 'o', c(:,1), c(:,2), '*', d(:,1), d(:,2), '.', e(:,1), e(:,2), 'x')
hold on
title('Raw IMU Output')
xlabel('hx')
ylabel('hy')
grid on
legend('Unit Circle', 'Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo')
hold off

figure
hold on
title('Raw IMU Output + Calibration Mapping')
plot(xunit, yunit, xNewA, yNewA, '+', xNewB, yNewB, 'o', xNewC, yNewC, '*', xNewD, yNewD, '.', xNewE, yNewE, 'x')
xlabel('hx')
ylabel('hy')
legend('Unit Circle', 'Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo')
grid on
hold off

%% Compute Headings stuff

% Get the heading values for a unit circle
hVals = zeros(length(xunit), 1);
for i = 1:length(xunit)
    hVals(i) = getHeading(xunit(i), yunit(i));
end

% Set up original and new heading matrices
headingsA = zeros(length(a),1);
headingsAnew = zeros(length(a),1);

headingsB = zeros(length(b),1);
headingsBnew = zeros(length(b),1);

headingsC = zeros(length(c),1);
headingsCnew = zeros(length(c),1);

headingsD = zeros(length(d),1);
headingsDnew = zeros(length(d),1);

headingsE = zeros(length(e),1);
headingsEnew = zeros(length(e),1);


% Compute them
for i = 1:length(a)

    headingsA(i) = getHeading(a(i,1), a(i,2));
    headingsAnew(i) = getHeading(xNewA(i), yNewA(i));

end

for i = 1:length(b)
    
    headingsB(i) = getHeading(b(i,1), b(i,2));
    headingsBnew(i) = getHeading(xNewB(i), yNewB(i));

end

for i = 1:length(c)
    
    headingsC(i) = getHeading(c(i,1), c(i,2));
    headingsCnew(i) = getHeading(xNewC(i), yNewC(i));

end

for i = 1:length(d)
    
    headingsD(i) = getHeading(d(i,1), d(i,2));
    headingsDnew(i) = getHeading(xNewD(i), yNewD(i));

end

for i = 1:length(e)
    headingsE(i) = getHeading(e(i,1), e(i,2));
    headingsEnew(i) = getHeading(xNewE(i), yNewE(i));
end



% Plot the original heading values as a function of the hx reading - There
% is a scale factor difference only in the horizatonal axis. The vertical
% axis differences are what we care about, which display the errors in the
% hy values returned by the sensor
figure
hold on
plot(a(:,1), headingsA, '.', b(:,1), headingsB, '.', c(:,1), headingsC, '.', d(:,1), headingsD, '.', e(:,1), headingsE, '.')
grid on
title('Raw IMU Output')
xlabel('hx')
ylabel('Heading')
legend('Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo')
hold off


figure
plot(xNewA, headingsAnew, '.', xNewB, headingsBnew, '.', xNewC, headingsCnew, '.', xNewD, headingsDnew, '.', xNewE, headingsEnew, '.', xunit, hVals, '-')
grid on
title('Raw IMU Output + Calibration Mapping')
xlabel('hx')
ylabel('Heading')
legend('Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo', 'Unit Circle')



