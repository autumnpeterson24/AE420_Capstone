clc;
clear;
close all;

%% === Concept Selection ===
choice = input('What concept do you want to simulate? (Enter 2 or 4): ');

switch choice
    case 2
        disp('Concept 2 selected.');
        W = 27.9;   % [lbf]
    case 4
        disp('Concept 4 selected.');
        W = 30.8;   % [lbf]
    otherwise
        warning('Invalid selection. Please restart and enter 2 or 4.');
        return;
end

%% === Base Geometry ===
b = 5;                             % Semi-span (ft or in, consistent)
k = W / (pi * b);                  % Constant for elliptical distribution
xc = (4*b) / (3*pi);               % Centroid of half-ellipse (moment arm)
M_ref = (W * xc) / 2;              % Reference moment about root

%% === 1. Elliptical Lift Distribution ===
x = linspace(-b, b, 400);
y = k .* sqrt(1 - (x./b).^2);

figure(1);
plot(x, y, 'LineWidth', 2);
hold on;
fill([x fliplr(x)], [y zeros(size(y))], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Spanwise position, x');
ylabel('Lift per unit span, y');
title('Elliptical Lift Distribution');
grid on;
xlim([-b b]);
ylim([0 max(y)*1.1]);

%% === 2. Roll Condition ===
theta = -30:0.5:30;        % bank angle (deg)
L_total = W ./ cosd(theta); % required total lift
L_right = 0.5 * L_total .* (1 + 0.5 * sind(theta));
L_left  = 0.5 * L_total .* (1 - 0.5 * sind(theta));

% Moments for each wing
M_right = L_right * xc;
M_left  = L_left  * xc;
M_net   = L_total/2 *xc;

% (Plot 2) Lift vs Bank Angle
figure(2);
plot(theta, L_total, 'k-', 'LineWidth', 1.8); hold on;
xlabel('Bank Angle (deg)');
ylabel('Lift (lbf)');
title('Lift Distribution vs Bank Angle');
legend('Total Lift','Location','best');
grid on;

% (Plot 3) Moment vs Bank Angle
figure(3);
plot(theta, M_net, 'm-', 'LineWidth', 2);
xlabel('Bank Angle (deg)');
ylabel('Banked Moment (lbf·ft)');
title('Banked Moment vs Bank Angle');
grid on;

%% === 3. Pitch Condition ===
pitch = -30:0.5:30;       
L_total = W ./ cosd(pitch);
L_vertical = L_total .* cosd(pitch);
L_horizontal = L_total .* sind(pitch);

% Moment due to total lift (pitching)
M_pitch = L_total/2 .* xc;

% (Plot 4) Lift Components vs Pitch Angle
figure(4);
plot(pitch, L_total, 'k-', 'LineWidth', 1.8); hold on;
plot(pitch, L_vertical, 'b--', 'LineWidth', 1.5);
plot(pitch, L_horizontal, 'r--', 'LineWidth', 1.5);
xlabel('Pitch Angle (deg)');
ylabel('Lift Components (lbf)');
title('Lift Components vs Pitch Angle');
legend('Total Lift', 'Vertical Component', 'Horizontal Component', 'Location', 'northwest');
grid on;

% (Plot 5) Moment vs Pitch Angle
figure(5);
plot(pitch, M_pitch, 'm-', 'LineWidth', 2);
xlabel('Pitch Angle (deg)');
ylabel('Pitching Moment (lbf·ft)');
title('Pitching Moment vs Pitch Angle');
grid on;

%% === 4. Descent Condition ===
Vx_knots = 60;
Vx = Vx_knots * 1.68781;   % ft/s
Vy = -30:0.5:0;            % ft/s (negative = downward)
gamma = atan(abs(Vy) ./ Vx);
L = W .* cos(gamma);       % lift required (steady descent)
M_descent = L/2 .* xc;       % moment from lift

% (Plot 6) Lift vs Descent Rate
figure(6);
plot(Vy, L, 'b-', 'LineWidth', 2); hold on;
xlabel('Vertical Velocity, V_y (ft/s)');
ylabel('Required Lift (lbf)');
title(['Required Lift vs Descent Rate at ' num2str(Vx_knots) ' knots']);
legend('Required Lift (steady descent)', 'Weight (W)', 'Location', 'best');
grid on;
text(-28, W-0.5, 'L = W (level flight)', 'FontSize', 9, 'Color', 'k');
text(-28, W-5, 'L < W → steady descent', 'FontSize', 9, 'Color', 'b');

% (Plot 7) Moment vs Descent Rate
figure(7);
plot(Vy, M_descent, 'm-', 'LineWidth', 2); hold on;
xlabel('Vertical Velocity, V_y (ft/s)');
ylabel('Moment (lbf·ft)');
title(['Moment vs Descent Rate at ' num2str(Vx_knots) ' knots']);
legend('Moment (steady descent)',  'Location', 'best');
grid on;

%% === Impact Condition ===
[v, dt] = meshgrid(linspace(0, 5, 50), linspace(0.005, 0.05, 50));

% --- Mass Properties ---
m_wing  = 7.76/2/32.2;     % Wing half mass [slug]
m_motor = 3.5/2/32.2;      % Motor half mass [slug]

% --- Impulse and Force Calculations ---
J_wing  = m_wing  .* v;
J_motor = m_motor .* v;

F_wing  = J_wing  ./ dt;
F_motor = J_motor ./ dt;

% --- Compute global max for both concepts ---
F_combined_max = max( F_wing(:) + F_motor(:) );  % for Concept 4 (largest case)
F_combined_min = min( F_wing(:) );               % lower bound for consistency

% --- Moments ---
M_wing  = F_wing  * 2.5;
M_motor = F_motor * (5/3);

% --- Combined Values Based on Concept ---
if choice == 2
    F_total = F_wing;
    M_total = M_wing;
    conceptLabel = 'Concept 2';
else
    F_total = F_wing + F_motor;
    M_total = M_wing + M_motor;
    conceptLabel = 'Concept 4';
end

% --- Plot: Force Surface ---
figure(8);
surf(v, dt, F_total, 'EdgeColor', 'none', 'FaceColor', 'interp');
xlabel('Descent Rate (ft/s)');
ylabel('Impact Time (s)');
zlabel('Force (lbf)');
title([conceptLabel ' - Force Experienced by Wing']);
view([232 28]);
colorbar;
caxis([F_combined_min F_combined_max]);  % 🔹 fixed scale for both concepts

% --- Plot: Moment Surface ---
figure(9);
surf(v, dt, M_total, 'EdgeColor', 'none', 'FaceColor', 'interp');
xlabel('Descent Rate (ft/s)');
ylabel('Impact Time (s)');
zlabel('Moment (lbf·ft)');
title([conceptLabel ' - Moment Experienced by Wing Root']);
view([232 28]);
colorbar;


%2D Contour Plot: Force vs Descent Rate and Impact Time
figure(13);
contourf(v, dt, F_total, 100, 'LineColor', 'none');
colorbar;
xlabel('Descent Rate (ft/s)');
ylabel('Impact Time (s)');
title([conceptLabel ' - Force Contour (Force vs Descent Rate & Impact Time)']);
colormap('turbo');
grid on;
caxis([F_combined_min F_combined_max]);