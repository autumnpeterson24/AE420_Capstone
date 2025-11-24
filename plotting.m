clear;close;clc
mainCarpetPlot
hold on
xlim([0, 5])
ylim([0, .4])
plot(WeightTOFL./SurfTOFL,ThrustTOFL./WeightTOFL,'-r', 'LineWidth',1.5)
plot(WeightClimb./SurfClimb,ThrustClimb./WeightClimb,'-b', 'LineWidth',1.5)
yl = ylim;
ymax = yl(2);
% x and y of the red curve
xRed = WeightTOFL./SurfTOFL;
yRed = ThrustTOFL./WeightTOFL;

% make sure x is sorted so the patch doesn't cross itself
[xRedSorted, idx] = sort(xRed);
yRedSorted = yRed(idx);

% build polygon: along the curve, then straight across at ymax
Xpatch = [xRedSorted, fliplr(xRedSorted)];
Ypatch = [yRedSorted, ymax*ones(size(yRedSorted))];

% draw light-green, transparent region above red line
patch(Xpatch, Ypatch, [0 1 0], ...
      'FaceAlpha', 0.15, ...   % transparency
      'EdgeColor', 'none');

[DesignThrust,DesignWeight,DesignWingArea] = DesignPoint();
plot(DesignWeight/DesignWingArea,DesignThrust/DesignWeight,'ro', 'MarkerSize', 10, 'LineWidth', 2)
%xline(30/ConstantWeightS(1),'--b','Color',[0.5 0.1 0.6])
%xline(35/ConstantWeightS(2),'--','Color', [0.9 0.4 0.1])

xline(40/ConstantWeightS(1),'--b')
xline(45/ConstantWeightS(2),'--k')
xline(50/ConstantWeightS(3),'--','Color', [0.9 0.4 0.1])
xline(55/ConstantWeightS(4),'--r')


legend({'Takeoff Constraint','Climb Rate Constraint','Feasible Design Space','Remora Design Point','40 lbs','45 lbs','50 lbs','55 lbs'},'Location', 'southeast','NumColumns', 2)
xlabel('Wing Loading W/S (lb/ft^2)'); ylabel('Thrust-to-Weight T/W');
set(findall(gcf,'-property','FontSize'),'FontSize',18)

% title("Figure 5.XXX Design Space plot - Thrust-to-Weight vs Wing Loading");