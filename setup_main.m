clear; clc;

% =========================================================================
% MASTER SCRIPT : CONFIGURATION AÉRO (Geometry + Defects + Dynamics)
% Structure : Lineaire (Style Script 1)
% =========================================================================

%% --- 1. CONFIGURATION DES DIMENSIONS (en mètres) ---

% Paramètre global
D_tube = 0.10;     % Diamètre fusée
R_tube = D_tube / 2;
S_ref  = pi * R_tube^2; % Surface de référence

% --- DEFINITION DES SETS ---

% SET 1 (bas, fixe)
Set1_Cr   = 0.15;    % Root chord
Set1_Ct   = 0.15;    % Tip chord
Set1_Span = 0.08;    % Envergure (semi-span!)
Set1_Kflap = 0.15;   % Ratio de surface active
Set1_Nb   = 4;       % Nombre d'ailerons dans le set

% SET 2 (cannards)
Set2_Cr   = 0.39;    
Set2_Ct   = 0.25;    
Set2_Span = 0.028;
Set2_Kflap = 0;      % 0 car pas set qui controle
Set2_Nb   = 4;

% SET 3 (haut, pour camera)
Set3_Cr   = 0.068;    
Set3_Ct   = 0.0279;    
Set3_Span = 0.01992;
Set3_Kflap = 0;      
Set3_Nb   = 4;


%% --- 2. CALCULS ET CRÉATION DES VARIABLES ---

% =========================================================================
% === SET 1 (Calculs complets) ===
% =========================================================================
% A. Variables pour Simulink (Bruit/Défauts)
S_aileron_Set1 = Set1_Span * (Set1_Cr + Set1_Ct) / 2;

AR1 = (2 * Set1_Span)^2 / (2 * S_aileron_Set1);
Cla_iso1 = (2 * pi * AR1) / (2 + sqrt(4 + AR1^2));
K_fb1 = 1 + (R_tube / (Set1_Span + R_tube));
Cla_Set1 = Cla_iso1 * K_fb1;

% Calcul du Bras de Levier (Distance Axe Fusée <-> Centre Pression Aileron)
Ycp_local_1 = (Set1_Span / 3) * ((Set1_Cr + 2*Set1_Ct) / (Set1_Cr + Set1_Ct));
BrasLevier_Set1 = R_tube + Ycp_local_1;

% B. Contribution à la Dynamique de Vol (Cld / Clp)
% Contribution Amortissement (Clp) pour ce set (4 ailerons)
% Formule : -Nb * Cla * S * R^2 / (Sref * D^2)
Clp_Contrib1 = -Set1_Nb * Cla_Set1 * (S_aileron_Set1 * BrasLevier_Set1^2) / (S_ref * D_tube^2);

% Contribution Contrôle (Cld) si actif (1 aileron actif + Kflap)
% Formule : Cla * S * R / (Sref * D) * Kflap
Cld_Contrib1 = (Cla_Set1 * S_aileron_Set1 * BrasLevier_Set1) / (S_ref * D_tube) * Set1_Kflap;


% =========================================================================
% === SET 2 (Calculs complets) ===
% =========================================================================
% A. Variables pour Simulink (Bruit/Défauts)
S_aileron_Set2 = Set2_Span * (Set2_Cr + Set2_Ct) / 2;

AR2 = (2 * Set2_Span)^2 / (2 * S_aileron_Set2);
Cla_iso2 = (2 * pi * AR2) / (2 + sqrt(4 + AR2^2));
K_fb2 = 1 + (R_tube / (Set2_Span + R_tube));
Cla_Set2 = Cla_iso2 * K_fb2;

% Calcul du Bras de Levier
Ycp_local_2 = (Set2_Span / 3) * ((Set2_Cr + 2*Set2_Ct) / (Set2_Cr + Set2_Ct));
BrasLevier_Set2 = R_tube + Ycp_local_2;

% B. Contribution à la Dynamique de Vol
Clp_Contrib2 = -Set2_Nb * Cla_Set2 * (S_aileron_Set2 * BrasLevier_Set2^2) / (S_ref * D_tube^2);
Cld_Contrib2 = (Cla_Set2 * S_aileron_Set2 * BrasLevier_Set2) / (S_ref * D_tube) * Set2_Kflap;


% =========================================================================
% === SET 3 (Calculs complets) ===
% =========================================================================
% A. Variables pour Simulink (Bruit/Défauts)
S_aileron_Set3 = Set3_Span * (Set3_Cr + Set3_Ct) / 2;

AR3 = (2 * Set3_Span)^2 / (2 * S_aileron_Set3);
Cla_iso3 = (2 * pi * AR3) / (2 + sqrt(4 + AR3^2));
K_fb3 = 1 + (R_tube / (Set3_Span + R_tube));
Cla_Set3 = Cla_iso3 * K_fb3;

% Calcul du Bras de Levier
Ycp_local_3 = (Set3_Span / 3) * ((Set3_Cr + 2*Set3_Ct) / (Set3_Cr + Set3_Ct));
BrasLevier_Set3 = R_tube + Ycp_local_3;

% B. Contribution à la Dynamique de Vol
Clp_Contrib3 = -Set3_Nb * Cla_Set3 * (S_aileron_Set3 * BrasLevier_Set3^2) / (S_ref * D_tube^2);
Cld_Contrib3 = (Cla_Set3 * S_aileron_Set3 * BrasLevier_Set3) / (S_ref * D_tube) * Set3_Kflap;


%% --- 3. AGRÉGATION FINALE (DYNAMIQUE GLOBALE) ---

% Somme des amortissements (Tous les sets freinent)
Clp_Total_Adim = Clp_Contrib1 + Clp_Contrib2 + Clp_Contrib3;

% Somme du contrôle (Seul celui avec Kflap > 0 compte)
Cld_Total_Adim = Cld_Contrib1 + Cld_Contrib2 + Cld_Contrib3;

% Mise à l'échelle pour Simulink (Group variables)
% On redonne les dimensions physiques [m^3] pour tes blocs de gain globaux
Clp_group = Clp_Total_Adim * S_ref * D_tube;
Cld_group = Cld_Total_Adim * S_ref * D_tube;

% Moment d'inertie en roulis (kg*m^2)
I_x = 0.0072; 

% Densité de l'air (kg/m^3) - supposée constante pour un vol < 600m
% et pour un vol a h(0)= 440m
rho = 1.18; 

%% --- Gains ---

% Points de rupture pour 'q' (en Pa)
% Basé sur  Vmax de ~131 m/s
q_breakpoints = [ 0, 1000, 3000, 6000, 12000 ];

% Données pour Kp (Gain Proportionnel)
Kp_data = [6.0, 5.0, 3.5, 2.0, 1.0];

% Données pour Ki (Gain Intégral)
Ki_data = [2.0, 1.5, 1.0, 0.5, 0.2];

% Données pour Kd (Gain Dérivé)
Kd_data = [0.05, 0.06, 0.06, 0.05, 0.05];



%% --- 5. AFFICHAGE POUR VÉRIFICATION ---
fprintf('\n--- 1. VARIABLES POUR BLOCS "DEFAUTS / BRUIT" ---\n');
fprintf('Set 1 : S_aileron=%.5f | Cla=%.3f | BrasLevier=%.4f\n', S_aileron_Set1, Cla_Set1, BrasLevier_Set1);
fprintf('Set 2 : S_aileron=%.5f | Cla=%.3f | BrasLevier=%.4f\n', S_aileron_Set2, Cla_Set2, BrasLevier_Set2);
fprintf('Set 3 : S_aileron=%.5f | Cla=%.3f | BrasLevier=%.4f\n', S_aileron_Set3, Cla_Set3, BrasLevier_Set3);

fprintf('\n--- 2. VARIABLES POUR BLOCS "DYNAMIQUE DE VOL" ---\n');
fprintf('Cld_group (Puissance Controle)     : %.8f m^3/rad\n', Cld_group);
fprintf('Clp_group (Amortissement)          : %.8f m^3/rad\n', Clp_group);
fprintf('I_x (Moment d inertie en roulis)   : %.8f kg*m^2\n', I_x);
fprintf('rho (densite de l air)             : %.8f kg/m^3\n', rho);

fprintf('\n--- 3. Gains ---\n');
fprintf('Kp_data : %s\n', mat2str(Kp_data));
fprintf('Ki_data : %s\n', mat2str(Ki_data));
fprintf('Kd_data : %s\n', mat2str(Kd_data));





%concerns and ameliorations:
%q_breakpoints plus large (avoir un nombre persque infini, 
% un q_breakpoints (v) et donc des lookup tables 
% ---> look into the Dynamic lookup table 

%check if leaving the "N" input blank in the PID is OK 

%to do list:
% - Clp and Cld (RASaero/openrocket) DONE
% - size of the control fin to know S_ref and b_ref DONE (as it is)
% - add wind gust perturbations (1-cos) and 
%   torque from the defaults of manufatcuring on the fins 
% - rate limiter for correct representation of the cervo




