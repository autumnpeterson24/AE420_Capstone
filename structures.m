function [totalW, cgX] = structures(S, AR, t, Sh, ARh, th, Lh, Sv, ARv, tv, Lv, ConfigNum, PrintOutput)
% calculates the total weight and the Cg location in the X and Y in
% reference to the datum of the nose in ft
% Inputs include:
% S  = reference area (h = horizonal stabilizer, v = vertical stabilizer)
% Ar = aspect ratio   (h = horizonal stabilizer, v = vertical stabilizer)
% t  = taper ratio    (h = horizonal stabilizer, v = vertical stabilizer)
% Lw = distance of the wing from the nose
% Lh = distance of the hor. stabilizer from the LE of the wing (positive aft)
% Lv = distance of the vert. stabilizer from the LE of the wing (positive aft)
% Lfuselage = length of the fuselage
% rfuselage = radius of a cylindrial fuselage
% ConfigNum = type of configuration (1-4) based on the initial concepts)
% PrintOutput = booleen 0 or 1 for if the code should output the plot and pie chard breakdown
% lbs and ft are the base units out for W and CG. CG datum is from the nose

%% Placeholder default values for Lh, Lv, Lw
rfuselage = [10.74887755/2 9/2 25.26/2 10.08750493/2]/12;
rfuselage = rfuselage(ConfigNum);
Lfuselage = [98 54 86.01 48]/12;
Lfuselage = Lfuselage(ConfigNum);
Lw = [59.49 34.788 32.22 15.3]/12;
Lw = Lw(ConfigNum);

%% calculation of secondary distances
% transfer of Lv and Lw to be diatnce from the nose for calculations
Lv = Lw + Lv;
Lh = Lw + Lh;

density = 0.025*12^3;         % Carbon Fiber lb/ft^3
surf_den = density/(16*12);  % Carbon Fiber lb/ft^2
lin_den = 0.2625;            % Carbon Fiber 1x1 inch 1/8 thickness lb/ft
foam_den = 2 % lb/ft^3

%% Characterization
[b, ~, ~, MAC] = characteristics(S, AR, t);        % ft
[bh, ~, ~, MACh] = characteristics(Sh, ARh, th); % ft
[bv, ~, ~, MACv] = characteristics(Sv, ARv, tv); % ft

%% Calculation of Surface areas and lengths
% Aerodynamic surfaces
Wing_surf = WingSurf("NACA2412.dat", MAC, b);    % ft^2 per wing
HStab_surf = WingSurf("NACA2412.dat", MACh, bh);     % ft^2
VStab_surf = WingSurf("NACA2412.dat", MACv, bv);     % ft^2

% Structural parts
Fuselage_surf = cylinderSurfaceArea(rfuselage, Lfuselage);  % ft^2 per config
Empenage_len = max([Lh, Lv])-Lw;                                % ft
SparWing_len = b;                                           % ft
SparHStab_len = bh;                                         % ft
SparVStab_len = bh;                                         % ft

%% Calculation of structural weights
WingSkin = Wing_surf*surf_den;       % lb
WingCore = Wing_Vol*foam_den;
HStabSkin = HStab_surf*surf_den;     % lb
VStabSkin = VStab_surf*surf_den;     % lb
Fuselage = Fuselage_surf*surf_den;   % lb
Empenage = Empenage_len*lin_den;     % lb
WingSpar = SparWing_len*lin_den;     % lb
HSpar = SparHStab_len*lin_den;       % lb
VSpar = SparVStab_len*lin_den;       % lb

%% CG and weight of aircraft
%     [Name,     Weight,    X_lead,    Y_lead, LengthX,         LengthY]
Cfig1 = {
    % structural
 'FuselSkin'     Fuselage   0.0        0       Lfuselage        rfuselage*2;
 'WingSkin'      WingSkin   Lw         0       MAC              b;
 'WingSpar'      WingSpar   Lw+MAC/3   0       0.1              b;
 'HStabSkin'     HStabSkin/2  Lh         rfuselage+bh/4       MACh             bh/2;
 'HSpar'         HSpar/2      Lh+MACh/3  rfuselage+bh/4       0.1              bh/2;
 'HStabSkin'     HStabSkin/2  Lh        -rfuselage-bh/4       MACh             bh/2;
 'HSpar'         HSpar/2      Lh+MACh/3 -rfuselage-bh/4       0.1              bh/2;
 'VStabSkin'     VStabSkin  Lw         b/2     MACv             0.1;
 'VSpar'         VSpar      Lw+MACh/3  b/2     0.1              0.1;   
 'VStabSkin'     VStabSkin  Lw        -b/2     MACv             0.1;
 'VSpar'         VSpar      Lw+MACh/3 -b/2     0.1              0.1; 
 % propulsion
 'Motor'         2          Lfuselage         0       0.25            0.5;
 'Prop'          0.25       Lfuselage+0.25    0       0.08            1.6;
    % electrical
 'Battery'       3.4        2            0      0.8             0.6;
 'Electronics'   0.4        3.5          0      0.5             0.5;
 'Actuator'      0.15       Lw+MAC/2     b/4    0.2             0.2;
 'Actuator'      0.15       Lw+MAC/2    -b/4    0.2             0.2;
 'Actuator'      0.15       Lh+(MACh/2)  0.5    0.2             0.2;
 'Actuator'      0.15       Lh+(MACh/2) -0.5    0.2             0.2;
 'Payload'       3          0.5          0      1               0.7;
    % misc
 'LandGear'      1.2        3            0      0.2             0.1;
 'LandGear'      1.2        Lw+MAC/2     1.6    0.2             0.1;
 'LandGear'      1.2        Lw+MAC/2    -1.6    0.2             0.1;
 'CaptrMechn'    1.5        Lw-0.7       0.9    0.5             1;
 };

%[Name,                 Weight,                  X_lead,        Y_lead, LengthX,     LengthY    ]
Cfig2 = {
    % structural
 'Fusel Skin          ' Fuselage+2               0.0            0       Lfuselage    rfuselage*2;
 'Wing Skin           ' WingSkin+WingCore                  Lw             0       MAC          b          ;
 'Wing Spar           ' WingSpar                 Lw+MAC/3       0       0.1          b          ;
 'Empennage           ' Empenage                 Lw+MAC/3      -bh/2    Empenage_len 0.1        ;
 'Empennage           ' Empenage                 Lw+MAC/3       bh/2    Empenage_len 0.1        ;
 'Horizontal Stab Skin' HStabSkin/cosd(50)*1.366 Lh             0       MACh         bh         ;
 'Horizontal Spar     ' HSpar/cosd(50)*1.366     Lh+MACh/3      0       0.1          bh         ;
    % propulsion
 'Motor               ' 2.83                     Lfuselage      0       0.25         0.3        ;
 'Prop                ' 0.125                    Lfuselage+0.25 0       0.08         2.1        ;
    % electrical
 'Battery             ' 4                        5/12           0       0.2          0.7        ;
 'Radio               ' 0.0625                   5/12           0       0.2          0.43       ;
 'Autopilot           ' 0.52                     1              0       0.24         0.325      ;
 'Receiver            ' 0.07                     5/12           0       0.24         0.15       ;
 'BEC                 ' 0.3                      1              0       0.1          0.17       ;
 'Payload             ' 3                        1              0       0.7          0.5        ;

 'Actuator            ' 0.25                     Lw+MAC/2       b/4     0.2          0.2        ;
 'Actuator            ' 0.25                     Lw+MAC/2      -b/4     0.2          0.2        ;
 'Actuator            ' 0.25                     Lh+(MACh/2)    0.5     0.2          0.2        ;
 'Actuator            ' 0.25                     Lh+(MACh/2)   -0.5     0.2          0.2        ;

    % misc
 'Landing Gear        ' 1.5                      1              0       0.5          0.1        ;
 'Landing Gear        ' 1.5                      Lw+MAC/2       1.6     0.5          0.1        ;
 'Landing Gear        ' 1.5                      Lw+MAC/2      -1.6     0.5          0.1        ;
 'Capture Mechanism   ' 2.0                      Lw-0.7         0.9     0.5          1.2        ;
 };

%     [Name,     Weight,    X_lead,    Y_lead, LengthX,         LengthY]
Cfig3 = {
    % structural
 'FuselSkin'     Fuselage   0.0        0       Lfuselage        rfuselage*2;
 'WingSkin'      WingSkin   Lw         0       MAC              b;
 'WingSpar'      WingSpar   Lw+MAC/3   0       0.1              b;
 'HStabSkin'     HStabSkin/2  Lh         rfuselage+bh/4       MACh             bh/2;
 'HSpar'         HSpar/2      Lh+MACh/3  rfuselage+bh/4       0.1              bh/2;
 'HStabSkin'     HStabSkin/2  Lh        -rfuselage-bh/4       MACh             bh/2;
 'HSpar'         HSpar/2      Lh+MACh/3 -rfuselage-bh/4       0.1              bh/2;
 'VStabSkin'     VStabSkin    Lv        0         MACv             0.1;
 'VSpar'         VSpar        Lv+MACv/3  0   0.1              0.1; 
    % propulsion
 'Motor'         2          Lfuselage           0       0.25            0.5;
 'Prop'          0.25       Lfuselage+0.25      0       0.08            1.6;
    % electrical
 'Battery'       3.4        2            0      0.8             0.6;
 'Electronics'   0.4        3.5          0      0.5             0.5;
 'Actuator'      0.15       Lw+MAC/2     b/4    0.2             0.2;
 'Actuator'      0.15       Lw+MAC/2    -b/4    0.2             0.2;
 'Actuator'      0.15       Lh+(MACh/2)  rfuselage+bh/8    0.2             0.2;
 'Actuator'      0.15       Lh+(MACh/2) -rfuselage-bh/4    0.2             0.2;
 'Payload'       3          0.5          0      1               0.7;
    % misc
 'LandGear'      1.2        3            0      0.2             0.1;
 'LandGear'      1.2        Lw+MAC/2     1.6    0.2             0.1;
 'LandGear'      1.2        Lw+MAC/2    -1.6    0.2             0.1;
 'CaptrMechn'    1.5        Lw-0.7       0.9    0.5             1;
 };

%     [Name,     Weight,    X_lead,    Y_lead, LengthX,         LengthY]
Cfig4 = {
    % structural
 'FuselSkin'     Fuselage   0.0        0       Lfuselage        rfuselage*2;
 'WingSkin'      WingSkin   Lw         0       MAC              b;
 'WingSpar'      WingSpar   Lw+MAC/3   0       0.1              b;
 'HStabSkin'     HStabSkin/2  Lh         0       MACh             bh;
 'HSpar'         HSpar/2      Lh+MACh/3  0       0.1              bh;
 'VStabSkin'     VStabSkin    Lv        0         MACv             0.1;
 'VSpar'         VSpar        Lv+MACv/3  0   0.1              0.1;
 'Empennage'      Empenage     Lfuselage  0        (Lh+MACh)-Lfuselage     0.1;
    % propulsion
 'Motor'         0.5        Lw+MAC           1.5       0.25            0.5;
 'Motor'         0.5        Lw+MAC          -1.5       0.25            0.5;
 'Prop'          0.125       Lw+MAC+0.25      1.5       0.08            1.2;
 'Prop'          0.125       Lw+MAC+0.25     -1.5       0.08            1.2;
    % electrical
 'Battery'       5.2        0.2          0      0.2             0.7;
 'Radio'         0.0625     0.4          0      0.2             0.43;
 'Autopilot'     0.52       0.6          0      0.24            0.325;
 'Receiver'      0.07       0.85         0      0.24            0.15;
 'BEC'           0.3        1            0      0.1             0.17;
 'Payload'       3          2            0      0.7             0.5;
 
 'Actuator'      0.25       Lw+MAC/2     b/4    0.2             0.2;
 'Actuator'      0.25       Lw+MAC/2    -b/4    0.2             0.2;
 'Actuator'      0.25       Lh+(MACh/2)  rfuselage+bh/8    0.2             0.2;
 'Actuator'      0.25       Lh+(MACh/2) -rfuselage-bh/4    0.2             0.2;
    % misc
 'Landing Gear'      2.0        1            0      0.5             0.1;
 'Landing Gear'      2.0        Lw+MAC/2     1.6    0.5             0.1;
 'Landing Gear'      2.0        Lw+MAC/2    -1.6    0.5             0.1;
 'Capture Mechanism'    2.0        Lw-0.7       0.9    0.5             1;
 };

% selec the config table based on the input
Cell = {Cfig1, Cfig2, Cfig3, Cfig4};
parts = Cell{ConfigNum};

% read the given values from the given table based on the config
names   = parts(:,1);
W       = cell2mat(parts(:,2));
X_lead  = cell2mat(parts(:,3));
Y_lead  = cell2mat(parts(:,4))-cell2mat(parts(:,6))/2;
LenX    = cell2mat(parts(:,5));
LenY    = cell2mat(parts(:,6));

% shift the CG of the part to an assumed center of the shape
Xc = X_lead + LenX/2;
Yc = Y_lead + LenY/2;

% sum the weights and do moment analysis from the nose for CG
totalW = sum(W);
cgX    = sum(W .* Xc) / totalW;
cgY    = sum(W .* Yc) / totalW;

% % output CG from given configuration
% fprintf('Total Weight: %.2f lb\n', totalW);
% fprintf('CG X-location: %.2f ft\n', cgX);
% fprintf('CG Y-location: %.2f ft\n', cgY);

%% plotting
if PrintOutput == 1
% setup plot the view with no distortion
figure; hold on; axis equal;
xlabel('X (ft)'); ylabel('Y (ft)');
% title('Configuration 1: Twin Boom CG');

% loop through all the parts and colour them in
% for random colour: max(randi(numel(W))/W)
for i = 1:numel(W)
    if W(i) > 0
        rectX = [X_lead(i), X_lead(i)+LenX(i), X_lead(i)+LenX(i), X_lead(i)];
        rectY = [Y_lead(i), Y_lead(i), Y_lead(i)+LenY(i), Y_lead(i)+LenY(i)];
        patch(rectX, rectY, [0.7 0.7 0.9], 'EdgeColor','k');
%         text(Xc(i), Yc(i), names{i}, 'FontSize',7, ...
%              'HorizontalAlignment','center');
    end
end

plot(cgX, cgY, 'kx', 'MarkerSize',8, 'LineWidth',2);
text(cgX, cgY, '  CG','Color','k','FontWeight','bold');
grid on;
hold off

%% Bill of Materials
% converts the data of the chosen configuration to a bill of materials in the form of a dictionary
% part data can be used for tradeoff analysis and have the same name as the config part definitions
    d = dictionary(names, 0); 
    for ii = 1:numel(names)
        d(names(ii)) = d(names(ii))+W(ii);
    end
    disp(d)
    E = entries(d);
    piechart(E.Value, E.Key)
end
end

function [A] = trap(Array_X, Array_Y)
% -Discrete trapesoidal integrator given a list of values in vector form
% -Array_X/Y corresponds to the X and Y coordinates given to the integrator
% -Returns the area A bound by the coordinates in Array_X/Y
%
% Array_X and Array_Y must be of the same size/numbe of elements

A = 0;
for ii = 1:numel(Array_X)-1
    A = (Array_X(ii+1)-Array_X(ii)) * ((Array_Y(ii)+Array_Y(ii+1))/2) + A;
end
end

function [D] = dist2(Array_X, Array_Y)
% -Discrete distance calculator given a list of values in vector form
% -Array_X/Y corresponds to the X and Y coordinates given to the function
% -Returns the distance D covered by follwoing the curve defined 
%  by the coordinates in Array_X/Y
%
% Array_X and Array_Y must be of the same size/numbe of elements
D = 0.0;
for ii = 1:numel(Array_X)-1
  D = D + sqrt( (Array_X(ii+1)-Array_X(ii))^2 + (Array_Y(ii+1)-Array_Y(ii))^2 );
end
end

function [b, c_root, c_tip, MAC] = characteristics(S, AR, t)
% calculates the relavent properties of an aerodynamic surface given
% the non-dimentionalized values
b = sqrt(AR * S);                              % Wingspan
c_root = 2 * S / (b * (1 + t));                % Root chord
c_tip = t * c_root;                            % Tip chord
MAC = (2/3) * c_root * (1 + t + t^2)/(1 + t);  % Mean Aerodynamic Chord
end

function A = cylinderSurfaceArea(r, L)
% Calculates the surface area of a cylinder given its length and radius
    A = 2 * pi * r^2 + 2 * pi * r * L;
end

function [WingSurfArea, WingVolume] = WingSurf(airfoil, AvgChord, b)
% calculation of the square inch and cubic inch volume of a surface given
% an airfoil and physical characteristics

dat = [1.000084  0.001257
  0.999106  0.001461
  0.996177  0.002070
  0.991307  0.003077
  0.984515  0.004469
  0.975825  0.006231
  0.965269  0.008342
  0.952888  0.010778
  0.938727  0.013512
  0.922841  0.016514
  0.905287  0.019752
  0.886134  0.023192
  0.865454  0.026799
  0.843325  0.030537
  0.819834  0.034369
  0.795069  0.038260
  0.769127  0.042172
  0.742109  0.046069
  0.714119  0.049913
  0.685266  0.053670
  0.655665  0.057302
  0.625431  0.060775
  0.594684  0.064054
  0.563545  0.067103
  0.532138  0.069890
  0.500588  0.072381
  0.469023  0.074547
  0.437567  0.076358
  0.406350  0.077787
  0.375297  0.078768
  0.344680  0.079198
  0.314678  0.079063
  0.285418  0.078363
  0.257025  0.077102
  0.229618  0.075290
  0.203313  0.072947
  0.178222  0.070097
  0.154449  0.066771
  0.132094  0.063006
  0.111248  0.058842
  0.091996  0.054325
  0.074415  0.049504
  0.058573  0.044427
  0.044532  0.039144
  0.032343  0.033704
  0.022051  0.028152
  0.013692  0.022531
  0.007292  0.016878
  0.002870  0.011224
  0.000439  0.005592
  0.000000  0.000000
  0.001535 -0.005395
  0.005015 -0.010439
  0.010421 -0.015126
  0.017725 -0.019451
  0.026892 -0.023408
  0.037880 -0.026990
  0.050641 -0.030193
  0.065120 -0.033014
  0.081257 -0.035451
  0.098987 -0.037507
  0.118239 -0.039185
  0.138937 -0.040494
  0.161004 -0.041445
  0.184354 -0.042056
  0.208902 -0.042346
  0.234556 -0.042339
  0.261222 -0.042063
  0.288802 -0.041549
  0.317198 -0.040830
  0.346303 -0.039941
  0.376013 -0.038917
  0.406269 -0.037791
  0.437099 -0.036513
  0.468187 -0.035070
  0.499412 -0.033493
  0.530653 -0.031808
  0.561789 -0.030043
  0.592698 -0.028222
  0.623259 -0.026368
  0.653352 -0.024500
  0.682858 -0.022636
  0.711661 -0.020791
  0.739645 -0.018979
  0.766700 -0.017212
  0.792716 -0.015499
  0.817590 -0.013849
  0.841222 -0.012271
  0.863515 -0.010770
  0.884379 -0.009356
  0.903730 -0.008033
  0.921487 -0.006809
  0.937579 -0.005691
  0.951939 -0.004685
  0.964507 -0.003798
  0.975232 -0.003035
  0.984069 -0.002402
  0.990980 -0.001904
  0.995938 -0.001546
  0.998920 -0.001330
  0.999916 -0.001257];

%input from dat file of airfoil
NACA2412_Coord = dat*AvgChord;

% calculation of curve length and area enclosed of airfoil
ArcLength = dist2(NACA2412_Coord(:, 1), NACA2412_Coord(:, 2));
AirfoilArea = -trap(NACA2412_Coord(:, 1), NACA2412_Coord(:, 2));

% Wing surface area and volume
WingSurfArea = (ArcLength*b+2*AirfoilArea); % in^2
WingVolume = AirfoilArea*b;                 % in^3
end







