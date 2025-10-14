function [D, CDp, CDi, alpha] = aerodynamics(W, S_w,AR_w,taper_w,S_h,AR_h,taper_h,S_v,AR_v,taper_v,V,h,Cfig);

%% Initialize

% Call atmosphere function
[~, ~, ~, ~, ~, ~, mach, REL, q] = atmosphere(h, V);  %	Note: REL is the Reynolds number per unit length

%% Aircraft Geometries - Input Driven

% Aircraft Weight
weight = W;     % max takeoff weight (lb)

% Wing Geometry
b_w = sqrt(AR_w * S_w); % wing span
c_w_root = 2*S_w/(b_w*(1+taper_w)); % wing root chord
c_w = (2/3)*c_w_root*(1+taper_w+taper_w^2)/(1+taper_w); % mean aero chord
tc_w = 0.1;  % thickness-to-chord ratio
Cl_a_w = 0.0972;
alpha_0 = -4.055;

% Horizontal Tail Geometry
b_h = sqrt(AR_h * S_h); % span
c_h_root = 2*S_h/(b_h*(1+taper_h)); % root chord
c_h = (2/3)*c_h_root*(1+taper_h+taper_h^2)/(1+taper_h); % mean aero chord
tc_h= 0.15; % thickness-to-chord ratio

% Vertical Tail Geometry
b_v = sqrt(AR_v * S_v); % span
c_v_root = 2*S_v/(b_v*(1+taper_v)); % root chord
c_v = (2/3)*c_v_root*(1+taper_v+taper_v^2)/(1+taper_v); % mean aero chord
tc_v = 0.15;% thickness-to-chord ratio

% Misc Inputs
fmarkup = 1.20;   % parasite drag correction for roughness

%% Aircraft Geometries - Config Dependant
if Cfig == 2

    % Wing Geometry
    sweep_w = 0;    % sweep    
    
    % Horizontal Tail Geometry
    sweep_h = 0; % sweep
    
    % Vertical Tail Geometry
    sweep_v = 0; % sweep
    
    % Fuselage Geometry
    L_fuse = 4; % total length of fuselage (ft)
    D_fuse = 1;   % fuselage diameter (ft)
    L_nose_cone = 1.5*D_fuse; % nose cone length (based on fineness ratio)
    L_tail_cone = 1.5*D_fuse; % tail cone length (based on fineness ratio)

    % Form Factors and efficiencies
    k_w = 1.2; % Form Factor  (from sweep and t/c)
    u = 0.99; % estimated
    s_w = .98; % determined from graph and D_fuse/Wingspan
    k_F = 1.2; % Form factor (from graph)
    k_h = 1.35; % Form Factor  (from sweep and t/c)
    s_h = .83; % determined from graph and D_fuse/stabspan
    k_v = 1.35; % Form Factor  (from sweep and t/c)


elseif Cfig == 4

    % Wing Geometry
    sweep_w = 0;    % sweep    
    
    % Horizontal Tail Geometry
    sweep_h = 10; % sweep
    
    % Vertical Tail Geometry
    sweep_v = 10; % sweep
    
    % Fuselage Geometry
    L_fuse = 4; % total length of fuselage (ft)
    D_fuse = 1;   % fuselage diameter (ft)
    L_nose_cone = 1.5*D_fuse; % nose cone length (based on fineness ratio)
    L_tail_cone = 1.5*D_fuse; % tail cone length (based on fineness ratio)

    % Form Factors and efficiencies
    k_w = 1.2; % Form Factor  (from sweep and t/c)
    u = 0.99; % estimated
    s_w = .98; % determined from graph and D_fuse/Wingspan
    k_F = 1.25; % Form factor (from graph)
    k_h = 1.35; % Form Factor  (from sweep and t/c)
    s_h = .83; % determined from graph and D_fuse/stabspan
    k_v = 1.35; % Form Factor  (from sweep and t/c)

end


%% Drag of Aircraft Components

% Wing Parasite Drag
c_f_int_w = 0.455./(log10(REL.*c_v)).^2.58; % Coefficent of Friction for Wing
c_f_inc_w = c_f_int_w * fmarkup; % Correction for Roughness
Tw_Tinf_w = 1+(0.178.*mach.^2);   % Mach Number Corrections
Tp_Tinf_w = (1+(0.035.*mach.^2))+(0.45.*(Tw_Tinf_w-1));
Tinf_Tp_w = 1./Tp_Tinf_w;
Rp_Rinf_w = ((Tinf_Tp_w + 216)./(217)).*Tinf_Tp_w.^1.5;
C_f_w = c_f_inc_w .*(Tinf_Tp_w).*(1./Rp_Rinf_w).^0.2; % Coefficent of friction
S_wett_w = 2*(1+(0.2*(tc_w)))*(S_w-(c_w_root*D_fuse)); % Wetted area
S_cs_w = 0.3 * S_w; % Control surface sizing
f_gap_w = 0.0002*((cos(sweep_w))^2)*S_cs_w; % Control surface gap drag
C_D_p_w = k_w*(C_f_w+(f_gap_w/S_w))*(S_wett_w/S_w)+ 0.0015; % Coefficent of parasite drag

% Wing Induced Drag
C_L = (weight)/(q*S_w); % Coefficent of lift of the plane
C_L_w = C_L +(C_L*0.05*(S_h/S_w)); % Wing CL
C_D_inv_w = (C_L_w^2)/(pi*AR_w*u*s_w); % Inviscid Drag
C_D_vis_w = 0.15*C_D_p_w*C_L_w^2; % Viscous drag
C_D_i_w = C_D_inv_w + C_D_vis_w; % Induced Drag

C_D_w = C_D_p_w + C_D_i_w ; % Total Coeficent of drag of the wing

% Fuselage Parasite Drag
c_f_int_F = 0.455./(log10(REL.*L_fuse)).^2.58; % Coefficent of Friction for Fuse
c_f_inc_F = c_f_int_F * fmarkup; % Correction for Roughness
Tw_Tinf_F = 1+(0.178.*mach.^2);   % Mach Number Corrections
Tp_Tinf_F = (1+(0.035.*mach.^2))+(0.45.*(Tw_Tinf_F-1));
Tinf_Tp_F = 1./Tp_Tinf_F;
Rp_Rinf_F = ((Tinf_Tp_F + 216)./(217)).*Tinf_Tp_F.^1.5;
C_f_F = c_f_inc_F .*(Tinf_Tp_F).*(1./Rp_Rinf_F).^0.2; % Coefficent of friction
S_wett_F = (pi*D_fuse*(L_fuse-L_nose_cone-L_tail_cone))+(0.75*pi*D_fuse*L_nose_cone)+(0.72*pi*D_fuse*L_tail_cone); % Wetted area
C_D_F = k_F*(C_f_F)*(S_wett_F/S_w); % Coefficent of parasite drag

% Horizontal tail Parasite Drag
c_f_int_h = 0.455./(log10(REL.*c_h)).^2.58; % Coefficent of Friction for stab
c_f_inc_h = c_f_int_h .* fmarkup; % Correction for Roughness
Tw_Tinf_h = 1+(0.178.*mach.^2);   % Mach Number Corrections
Tp_Tinf_h = (1+(0.035.*mach.^2))+(0.45*(Tw_Tinf_h-1));
Tinf_Tp_h = 1./Tp_Tinf_h;
Rp_Rinf_h = ((Tinf_Tp_h + 216)./(217)).*Tinf_Tp_h.^1.5;
C_f_h = c_f_inc_h .*(Tinf_Tp_h).*(1./Rp_Rinf_h).^0.2; % Coefficent of friction
S_wett_h = 2*(1+(0.2*(tc_h)))*S_h; % Wetted area
S_cs_h = S_h; % Control surface sizing
f_gap_h = 0.0002*((cos(sweep_h))^2)*S_cs_h; % Control surface gap drag
C_D_p_h = k_h*(C_f_h+(f_gap_h/S_h))*(S_wett_h/S_w); % Coefficent of parasite drag

% Horizontal Tail Induced Drag
C_L_h = 0.15*C_L; % Coefficent of lift of the stab
C_D_inv_h = (C_L_h^2)/(pi*AR_h*u*s_h); % Inviscid Drag
C_D_vis_h = 0.15*C_D_p_h*C_L_h^2; % Viscous drag
C_D_i_h = C_D_inv_h + C_D_vis_h; % Induced Drag

C_D_h = C_D_p_h + C_D_i_h;  % Total Coeficent of drag of the horz stab

% Vertical tail Parasite Drag
c_f_int_v = 0.455./(log10(REL.*c_v)).^2.58; % Coefficent of Friction for stab
c_f_inc_v = c_f_int_v .* fmarkup; % Correction for Roughness
Tw_Tinf_v = 1+(0.178.*mach.^2);   % Mach Number Corrections
Tp_Tinf_v = (1+(0.035.*mach.^2))+(0.45.*(Tw_Tinf_v-1));
Tinf_Tp_v = 1./Tp_Tinf_v;
Rp_Rinf_v = ((Tinf_Tp_v + 216)./(217)).*Tinf_Tp_v.^1.5;
C_f_v = c_f_inc_v .*(Tinf_Tp_v).*(1./Rp_Rinf_v).^0.2; % Coefficent of friction
S_wett_v = 2*(1+(0.2*(tc_v)))*S_v; % Wetted area
S_cs_v = S_v; % Control surface sizing
f_gap_v = 0.0002*((cos(sweep_v))^2)*S_cs_v; % Control surface gap drag
C_D_v = k_v*(C_f_v+(f_gap_v/S_v))*(S_wett_v/S_w); % Coefficent of  drag

%% Finalize Drag Totals

C_D_p_m = (C_D_p_w+C_D_F+C_D_p_h+C_D_v)*.2;
C_D_m = C_D_p_m;
C_D_i= C_D_i_h+C_D_i_w;
C_D_p = C_D_p_w+C_D_F+C_D_p_h+C_D_v+C_D_p_m;
C_D = C_D_w+C_D_F+C_D_h+C_D_v+C_D_m;
D_tot =C_D*q*S_w; 

%% Alpha

e = 2/(2-AR_w+sqrt(4+AR_w^2*(1+tand(taper_w)))); % eq1, Span Efficiency
CL_a_w =  Cl_a_w./((1+(57.3.*Cl_a_w)./(pi.*e.*AR_w))); % eq2, 3D Lift Curve Slope
alpha = weight / (q * S_w *CL_a_w) + alpha_0;

%% Output

CDdisp = [ C_D_p_w  C_D_i_w             C_D_w;
            C_D_F   0                     C_D_F;
            C_D_p_h C_D_i_h             C_D_h;
            C_D_v   0                     C_D_v;
            C_D_m   C_D_vis_h+C_D_vis_w   C_D_m+C_D_vis_h+C_D_vis_w;
            C_D_p   C_D_i                 C_D];

CDp = C_D_p;   % parasite drag coefficient
CDi = C_D_i;   % induced drag coefficient
D = D_tot;        % total drag (in lbs)
alpha = alpha;      % angle of attack at the CL needed for steady, level flight

end
