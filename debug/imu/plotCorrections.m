function [fig] = plotCorrections( E, Enew, xNewSE, yNewSE, figureTitle)

x = 0;
y = 0;
r = 1;

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;

hVals = zeros(length(xunit), 1);
for i = 1:length(xunit)
    hVals(i) = getHeading(xunit(i), yunit(i));
end

figure
suptitle(figureTitle)
subplot(2,2,1)
view([0 0]);
hold on
scatter3(xNewSE, yNewSE, E(:,3), 10, E(:,3), 'filled')
scatter3(xunit, yunit, hVals, '.', 'k')
xlabel('hx')
ylabel('hy')
zlabel('Heading')
ax = gca;
ax.ZTick = [0:90:360];
zlim([0 360])
axis square
grid on
hold off

subplot(2,2,2)
hold on
view([0 0]);

scatter3(Enew(:,1), Enew(:,2), Enew(:,3), 10, Enew(:,3), 'filled')
scatter3(xunit, yunit, hVals, '.', 'k')
xlabel('hx')
ylabel('hy')
zlabel('Heading')
zlim([0 360])
ax = gca;
ax.ZTick = [0:90:360];
axis square
grid on
hold off

subplot(2,2,3)
hold on
view([90 0]);
scatter3(xNewSE, yNewSE, E(:,3), 5, E(:,3))
scatter3(xunit, yunit, hVals, '.', 'k')
xlabel('hx')
ylabel('hy')
zlabel('Heading')
zlim([0 360])
ax = gca;
ax.ZTick = [0:90:360];
axis square
grid on
hold off


subplot(2,2,4)
hold on
view([90 0]);

scatter3(Enew(:,1), Enew(:,2), Enew(:,3), 5, Enew(:,3))
scatter3(xunit, yunit, hVals, '.', 'k')
xlabel('hx')
ylabel('hy')
zlabel('Heading')
zlim([0 360])
ax = gca;
ax.ZTick = [0:90:360];
axis square
grid on
hold off

fig = gcf;
end

