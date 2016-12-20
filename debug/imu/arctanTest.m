%% Circle Arctan Stuff
clear; clc; close all;

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

hVals = zeros(length(xunit), 1);
for i = 1:length(xunit)
    hVals(i) = getHeading(xunit(i), yunit(i));
end

figure
plot(xunit, yunit)
grid on
hold off
