Cfig = 2;
Alt = 5000; %ft
AirfoilMaxThickness = .12; %Fraction of chord
V = 101; %ft/s
%--------------------------------------------------------------------------
% INPUTS
%--------------------------------------------------------------------------
WingTipThickness = 1.5; %inches
WingArea = 11; %ft^2
TaperRatio = .67;
%--------------------------------------------------------------------------
% Wing Sizing Calculations
%--------------------------------------------------------------------------
TipChord = (WingTipThickness/12)/AirfoilMaxThickness; %ft
RootChord = TipChord/TaperRatio; %ft
AverageWingChord = (TipChord + RootChord) / 2;
SingleWingSpan = (WingArea/2)/(AverageWingChord); %ft
TotalWingSpan = SingleWingSpan * 2; %ft
AR = (TotalWingSpan^2)/WingArea;
cmac = (2/3)*RootChord*(1+TaperRatio+TaperRatio^2)/(1+TaperRatio);
%--------------------------------------------------------------------------
% Aircraft Design Code
%--------------------------------------------------------------------------
b = TotalWingSpan;
S = WingArea;
t = TaperRatio;
% Call Controls function
[Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S , cmac , t, Cfig);
% Call Structures function
[W, CG] = structures_modified(S , AR , t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, Cfig,0,0);
% Call Aerodynamics function
[D, CDp, CDi, alpha] = aerodynamics(W, S , AR , t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);
% Call Stability function
[SM, I] = stability(CG, AR , t, Vh, ARh, alpha, cmac , hn, Cfig);
% Call Propulsion function
%[P] = propulsion(V, D, Alt);
% Call Performance function
T = .3*W;
[TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, D);
% Weight Sensitivity
TOFL = 0;
ValidWeights = [];
i=1;
Weight = W;
while TOFL <= 350
    [TOFL, Climb, MaxAlt, Time] = performance(Weight, S , T ,V, Alt,AR, D);
    ValidWeights(i)= Weight;
    Weight = Weight + .1*i;
    if i >200
        break
    end
    i=i+1;
end

%Drag Sensitivity
i = 1;
Drag = D;
ValidDrag = [];
TOFL = 0;
while TOFL <= 350
    [TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, Drag);
    ValidDrag(i)= Drag;
    Drag = Drag + .1*i;
    if i >200
        break
    end
    i=i+1;
end

%Thrust Sensitivity
i = 1;
Thrust = T;
ValidThrust = [];
TOFL = 0;
while TOFL <= 350
    [TOFL, Climb, MaxAlt, Time] = performance(W, S , Thrust ,V, Alt,AR, D);
    ValidThrust(i)= Thrust;
    Thrust = Thrust - .1*i;
    if i >200
        break
    end
    i=i+1;
end

%Plotting it all
% clear;close;clc
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
x_left=(DesignWeight-(ValidWeights(end)-DesignWeight))/DesignWingArea;
y_bottom=DesignThrust/(DesignWeight+(ValidWeights(end)-DesignWeight));
width=(DesignWeight+(ValidWeights(end)-DesignWeight))/DesignWingArea - x_left;
height=DesignThrust/(DesignWeight-(ValidWeights(end)-DesignWeight))-y_bottom;
rectangle('Position',[x_left y_bottom width height], ...
        'FaceAlpha', .50, ...
        'FaceColor',[0 0.6 0 0.35], ...
        'EdgeColor','none');

plot(DesignWeight/DesignWingArea,DesignThrust/DesignWeight,'ro', 'MarkerSize', 10, 'LineWidth', 2)
plot((DesignWeight-(ValidWeights(end)-DesignWeight))/DesignWingArea,DesignThrust/(DesignWeight-(ValidWeights(end)-DesignWeight)),'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','')
text((DesignWeight-(ValidWeights(end)-DesignWeight))/DesignWingArea,DesignThrust/(DesignWeight-(ValidWeights(end)-DesignWeight))+.005,'-5%', 'VerticalAlignment','bottom','HorizontalAlignment','center');
plot(DesignWeight/DesignWingArea,(DesignThrust+(DesignThrust-ValidThrust(end)))/DesignWeight,'ko', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','')
text(DesignWeight/DesignWingArea,(DesignThrust+(DesignThrust-ValidThrust(end)))/DesignWeight+.005,'+5%', 'VerticalAlignment','bottom');
plot((DesignWeight+(ValidWeights(end)-DesignWeight))/DesignWingArea,DesignThrust/(DesignWeight+(ValidWeights(end)-DesignWeight)),'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','')
text((DesignWeight+(ValidWeights(end)-DesignWeight))/DesignWingArea,DesignThrust/(DesignWeight+(ValidWeights(end)-DesignWeight))-.005,'+5%', 'VerticalAlignment','top');
plot(DesignWeight/DesignWingArea,(DesignThrust-(DesignThrust-ValidThrust(end)))/DesignWeight,'ko', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','')
text(DesignWeight/DesignWingArea,(DesignThrust-(DesignThrust-ValidThrust(end)))/DesignWeight-.005,'-5%', 'VerticalAlignment','top','HorizontalAlignment','center');
% xline(30/ConstantWeightS(1),'--b','Color',[0.5 0.1 0.6])
% xline(35/ConstantWeightS(2),'--','Color', [0.9 0.4 0.1])
% xline(40/ConstantWeightS(1),'--b')
% xline(45/ConstantWeightS(2),'--k')
% xline(50/ConstantWeightS(3),'--','Color', [0.9 0.4 0.1])
% xline(55/ConstantWeightS(4),'--r')


legend({'Takeoff Constraint','Climb Rate Constraint','Feasible Design Space','Remora Design Point','Weight Variation','Thrust Variation'},'Location', 'southeast','NumColumns', 2)
xlabel('Wing Loading W/S (lb/ft^2)'); ylabel('Thrust-to-Weight T/W');
set(findall(gcf,'-property','FontSize'),'FontSize',18)

% title("Figure 5.XXX Design Space plot - Thrust-to-Weight vs Wing Loading");