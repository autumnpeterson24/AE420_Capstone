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

for S = 5:0.1:50
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
toc

