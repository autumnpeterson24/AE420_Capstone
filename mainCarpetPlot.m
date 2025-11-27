% clear;clc
tic
% Design variable choices
V = 101;      % flight velocity (ft/s)
Alt = 5000; % flight altitude (ft)
b = 10;      % wing span (ft)
cavg = 1;  % average wing chord length (ft)
t = 0.7;     % wing taper ratio
Cfig = 2;    % Config Selection

% S = 1;
% T = 5;
counterS = 1;
TOFL = 1000;
start_s_limit = 5;
end_s_limit = 51;
s_increment = 0.1;
inc = (end_s_limit-start_s_limit)/s_increment +1;
ThrustTOFL = zeros(1,inc);
SurfTOFL = zeros(1,inc);
WeightTOFL = zeros(1,inc);
ThrustClimb = zeros(1,inc);
SurfClimb = zeros(1,inc);
WeightClimb = zeros(1,inc);
weights = [40 45 50 55];
ConstantWeightS = [0 0 0 0];

for S = start_s_limit:s_increment:end_s_limit
    %Calculating Min Thrust for TOFL
    T = 50;
    TOFL = 0;
    AR = b^2/S;     % wing aspect ratio
    c_root = 2*S/(b*(1+t)); % wing root chord
    c_tip = t*c_root;       % wing tip chord
    cmac = (2/3)*c_root*(1+t+t^2)/(1+t);  % wing mean aerodynamic chord
    % Call Controls function
    [Sh, ARh, th, Lh, Vh, Sv, ARv, tv, Lv, hn] = controls(b, S , cmac , t, Cfig);
    % Call Structures function
    [W, CG] = structures_modified(S , AR , t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, Cfig,0,0);
    % Call Aerodynamics function
    [D, CDp, CDi, alpha] = aerodynamics(W, S , AR , t, Sh, ARh, th, Sv, ARv, tv, V, Alt, Cfig);
    % Call Stability function
    %[SM, I] = stability(CG, AR , t, Vh, ARh, alpha, cmac , hn, Cfig);
    % Call Propulsion function
    %[P] = propulsion(V, D, Alt);
    for i = 1:length(weights)
        % find where W is within ±0.5 of target
        idx = abs(W - weights(i)) <= 0.5;
    
        if any(idx)
            % Store the corresponding S (take first if multiple)
            ConstantWeightS(i) = S;
        end
    end
    while TOFL<350
        % Call Performance function
        [TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, D,CDp);

        T = T - .1;
    end
    ThrustTOFL(counterS) = T;
    SurfTOFL(counterS) = S;
    WeightTOFL(counterS) = W;

    while Climb>500
        % Call Performance function
        [TOFL, Climb, MaxAlt, Time] = performance(W, S , T ,V, Alt,AR, D,CDp);

        T = T - .1;
    end
    ThrustClimb(counterS) = T;
    SurfClimb(counterS) = S;
    WeightClimb(counterS) = W;

    counterS = counterS+1;
    if W >= 55
        break
    end
end
ThrustTOFL = ThrustTOFL(1:counterS-1);
SurfTOFL = SurfTOFL(1:counterS-1);
WeightTOFL = WeightTOFL(1:counterS-1);
ThrustClimb = ThrustClimb(1:counterS-1);
SurfClimb = SurfClimb(1:counterS-1);
WeightClimb = WeightClimb(1:counterS-1);
toc