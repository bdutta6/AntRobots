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
title('Raw IMU Output')
grid on
hold on
plot(xunit, yunit, a(:,1), a(:,2), '+', b(:,1), b(:,2), 'o', c(:,1), c(:,2), '*', d(:,1), d(:,2), '.', e(:,1), e(:,2), 'x')
hold off

figure
title('Raw IMU Output + Calibration Mapping')
grid on
hold on
plot(xunit, yunit, xNewA, yNewA, '+', xNewB, yNewB, 'o', xNewC, yNewC, '*', xNewD, yNewD, '.', xNewE, yNewE, 'x')
hold off


