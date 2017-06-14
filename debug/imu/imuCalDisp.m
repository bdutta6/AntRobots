clear; clc; close all;

a = csvread('alphaIMU.csv', 1);
b = csvread('bravoIMU.csv', 1);
c = csvread('charlieIMU.csv', 1);
d = csvread('deltaIMU.csv', 1);
e = csvread('echoIMU.csv', 1);

%% Map to the unit circle
[xNewA, yNewA] = map2unit(a);
[xNewB, yNewB] = map2unit(b);
[xNewC, yNewC] = map2unit(c);
[xNewD, yNewD] = map2unit(d);
[xNewE, yNewE] = map2unit(e);

%% Scale to the unit circle, but do not shift
[xNewSA, yNewSA] = scale2unit(a);
[xNewSB, yNewSB] = scale2unit(b);
[xNewSC, yNewSC] = scale2unit(c);
[xNewSD, yNewSD] = scale2unit(d);
[xNewSE, yNewSE] = scale2unit(e);


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
print -dpng rawIMUOutput

figure
hold on
title('Raw IMU Output + Calibration Mapping')
plot(xunit, yunit, xNewA, yNewA, '+', xNewB, yNewB, 'o', xNewC, yNewC, '*', xNewD, yNewD, '.', xNewE, yNewE, 'x')
xlabel('hx')
ylabel('hy')
legend('Unit Circle', 'Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo')
grid on
hold off
% print -dpng calibratedIMUOutput

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
    A(i,:) = [a(i,1), a(i,2), headingsA(i)];
    Anew(i,:) = [xNewA(i), yNewA(i), headingsAnew(i)];

end

for i = 1:length(b)
    
    headingsB(i) = getHeading(b(i,1), b(i,2));
    headingsBnew(i) = getHeading(xNewB(i), yNewB(i));
    B(i,:) = [b(i,1), b(i,2), headingsB(i)];
    Bnew(i,:) = [xNewB(i), yNewB(i), headingsBnew(i)];
end

for i = 1:length(c)
    
    headingsC(i) = getHeading(c(i,1), c(i,2));
    headingsCnew(i) = getHeading(xNewC(i), yNewC(i));
    C(i,:) = [c(i,1), c(i,2), headingsC(i)];
    Cnew(i,:) = [xNewC(i), yNewC(i), headingsCnew(i)];
end

for i = 1:length(d)
    
    headingsD(i) = getHeading(d(i,1), d(i,2));
    headingsDnew(i) = getHeading(xNewD(i), yNewD(i));
    D(i,:) = [d(i,1), d(i,2), headingsD(i)];
    Dnew(i,:) = [xNewD(i), yNewD(i), headingsDnew(i)];
end

for i = 1:length(e)
    headingsE(i) = getHeading(e(i,1), e(i,2));
    headingsEnew(i) = getHeading(xNewE(i), yNewE(i));    
    E(i,:) = [e(i,1), e(i,2), headingsE(i)];
    Enew(i,:) = [xNewE(i), yNewE(i), headingsEnew(i)];
    
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
% print -dpng rawHeadingOutput


figure
plot(xNewA, headingsAnew, '.', xNewB, headingsBnew, '.', xNewC, headingsCnew, '.', xNewD, headingsDnew, '.', xNewE, headingsEnew, '.', xunit, hVals, '-')
grid on
title('Raw IMU Output + Calibration Mapping')
xlabel('hx')
ylabel('Heading')
legend('Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo', 'Unit Circle')
% print -dpng calibratedHeadingOutput


figa = plotCorrections( A, Anew, xNewSA, yNewSA, 'IMU A');
print -dpng figa
figb = plotCorrections( B, Bnew, xNewSB, yNewSB, 'IMU B');
print -dpng figb

figc = plotCorrections( C, Cnew, xNewSC, yNewSC, 'IMU C');
print -dpng figc

figd = plotCorrections( D, Dnew, xNewSD, yNewSD, 'IMU D');
print -dpng figd

fige = plotCorrections( E, Enew, xNewSE, yNewSE, 'IMU E');
print -dpng fige
