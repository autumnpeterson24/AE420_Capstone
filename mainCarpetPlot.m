clear;clc
tic
% Design variable choices
V = 101;      % flight velocity (ft/s)
Alt = 5000; % flight altitude (ft)
b = 10;      % wing span (ft)
cavg = 1;  % average wing chord length (ft)
t = 0.7;     % wing taper ratio
Cfig = 2;    % Config Selection
% Calculated properties
% S = b*cavg;     % wing area (in^2)
% AR_bl = b^2/S;     % wing aspect ratio
% c_root_bl = 2*S/(b*(1+t)); % wing root chord
% c_tip_bl = t*c_root_bl;       % wing tip chord
% cmac_bl = (2/3)*c_root_bl*(1+t+t^2)/(1+t);  % wing mean aerodynamic chord

% S = 1;
% T = 5;
counterT = 1;
counterS = 1;
TOFL = 1000;
ThrustTOFL = [0];
SurfTOFL = [0];
WeightTOFL = [0];
ThrustClimb = [0];
SurfClimb = [0];
WeightClimb = [0];

targetW = 25:5:50;
tolW    = 0.5;   % widen if hits are sparse

% For TOFL
cw_TOFL_S = cell(size(targetW));   % S at which W≈target
cw_TOFL_T = cell(size(targetW));   % T at that S

for S = 5:.1:50
    %Calculating Min Thrust for TOFL
    T = 50;
    TOFL = 0;
    AR = b^2/S;     % wing aspect ratio
    c_root = 2*S/(b*(1+t)); % wing root chord
    c_tip = t*c_root;       % wing tip chord
    cmac = (2/3)*c_root*(1+t+t^2)/(1+t);  % wing mean aerodynamic chord
    while TOFL<350
        % Call Controls function
        [Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S , cmac , t, Cfig);
        % Call Structures function
        [W, CG] = structures(S , AR , t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, Cfig);
        % Call Aerodynamics function
        [D, CDp, CDi, alpha] = aerodynamics(W, S , AR , t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);
        % Call Stability function
        [SM, I] = stability(CG, AR , t, Vh, ARh, alpha, cmac , Cfig);
        % Call Propulsion function
        %[P] = propulsion(V, D, Alt);
        % Call Performance function
        [TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, D);
           
        % ---- Record (S,T) whenever W is close to a target (TOFL loop) ----
        for k = 1:numel(targetW)
            if abs(W - targetW(k)) <= tolW
                % keep at most one point per S for this weight
                if isempty(cw_TOFL_S{k}) || cw_TOFL_S{k}(end) ~= S
                    cw_TOFL_S{k}(end+1) = S;
                    cw_TOFL_T{k}(end+1) = T;
                end
            end
        end

        T = T - .1;
        counterT = counterT +1;
        counterT;
    end
    ThrustTOFL(counterS) = T;
    SurfTOFL(counterS) = S;
    WeightTOFL(counterS) = W;

    %Calculating Min Thrust for Climb
    T = 50;
    Climb = 10000;
    while Climb>400
        % Call Controls function
        [Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S , cmac , t, Cfig);
        % Call Structures function
        [W, CG] = structures(S , AR , t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, Cfig);
        % Call Aerodynamics function
        [D, CDp, CDi, alpha] = aerodynamics(W, S , AR , t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);
        % Call Stability function
        [SM, I] = stability(CG, AR , t, Vh, ARh, alpha, cmac , Cfig);
        % Call Propulsion function
        %[P] = propulsion(V, D, Alt);
        % Call Performance function
        [TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, D);

        T = T - .1;
        counterT = counterT +1;
        counterT;
    end
    ThrustClimb(counterS) = T;
    SurfClimb(counterS) = S;
    WeightClimb(counterS) = W;

    counterS = counterS+1;
    counterS
end
%plot(Thrust)
%plot(Surf)
%plot(Weight)
hold on
xlim([0, 10])
plot(WeightTOFL./SurfTOFL,ThrustTOFL./WeightTOFL)
plot(WeightClimb./SurfClimb,ThrustClimb./WeightClimb)
xlabel('Wing Loading W/S'); ylabel('Thrust-to-Weight T/W')

for k = 1:numel(targetW)
    Wk = targetW(k);

    % TOFL constant-W line (connect across S values)
    if ~isempty(cw_TOFL_S{k})
        x_to = Wk ./ cw_TOFL_S{k};   % W/S
        y_to = cw_TOFL_T{k} ./ Wk;   % T/W
        [x_to, ord] = sort(x_to);    % sort for a clean line
        y_to = y_to(ord);
        plot(x_to, y_to, '-', 'LineWidth', 1.1, 'DisplayName', sprintf('W=%g (TOFL)', Wk));
    end
end
legend('Location','best'); grid on

toc

