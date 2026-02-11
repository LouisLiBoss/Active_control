clear; clc;

% =========================================================================
% MASTER SCRIPT : CONFIGURATION AÉRO (MÉTHODE DYNAMIQUE 2D)
% =========================================================================

%% --- 1. CONFIGURATION DES DIMENSIONS ET DU VOL ---
% Paramètres globaux
D_tube = 0.10;      % Diamètre fusée [m]
R_tube = D_tube / 2;
S_ref  = pi * R_tube^2; % Surface de référence [m^2]

% Conditions de vol nominales (Pour calculs statiques Sets 2 & 3)
Mach_design = 0.385;        % Vitesse critique (Max Q)
beta_PG = sqrt(1 - Mach_design^2); % Facteur Prandtl-Glauert nominal
rho = 1.18;                 % Densité air moyenne [kg/m^3]

% --- DEFINITION DES SETS D'AILERONS ---
% SET 1 (Ailerons Bas - CONTRÔLE ACTIF)
Set1_Cr   = 0.15;    % Corde emplanture
Set1_Ct   = 0.15;    % Corde saumon
Set1_Span = 0.08;    % Envergure (Semi-span)
Set1_Kflap = 0.1342; % Ratio de surface active (Partie mobile)
Set1_Nb   = 4;       % Nombre d'ailerons

% SET 2 (Canards - Fixes/Passifs)
Set2_Cr   = 0.39;    
Set2_Ct   = 0.25;    
Set2_Span = 0.028;
Set2_Kflap = 0;      
Set2_Nb   = 4;

% SET 3 (Haut - Camera - Fixes/Passifs)
Set3_Cr   = 0.068;    
Set3_Ct   = 0.0279;    
Set3_Span = 0.01992;
Set3_Kflap = 0;      
Set3_Nb   = 4;


%% --- 2. CALCULS AÉRODYNAMIQUES DÉTAILLÉS ---

% =========================================================================
% === SET 1 : AILERONS DE CONTRÔLE (MÉTHODE DYNAMIQUE 2D) ===
% =========================================================================
% Ce bloc génère la MAP 2D (Cl = f(Angle, Mach)) pour le bloc "2-D Lookup Table" Simulink.
% Optimisation : Plage Mach 0 -> 0.50 (Précision max pour le vol subsonique)

% A. Géométrie Fixe du Set 1
S_aileron_Set1 = Set1_Span * (Set1_Cr + Set1_Ct) / 2;
AR1 = (2 * Set1_Span)^2 / (2 * S_aileron_Set1); 
Ycp_local_1 = (Set1_Span / 3) * ((Set1_Cr + 2*Set1_Ct) / (Set1_Cr + Set1_Ct));
BrasLevier_Set1 = R_tube + Ycp_local_1;

% B. Définition des axes de la Map 2D (Breakpoints Simulink)
Mach_vector = 0 : 0.01 : 0.50;      % Axe X : Vitesse (Mach)
delta_deg_vector = -30 : 0.5 : 30;  % Axe Y : Angle commande (Degrés)
delta_rad_vector = deg2rad(delta_deg_vector); % Axe Y : Angle commande (Radians)

% Initialisation de la Matrice (Lignes=Angles, Colonnes=Mach)
Cl_2D_Map = zeros(length(delta_rad_vector), length(Mach_vector));

% Facteurs constants Set 1
K_fb1 = 1 + (R_tube / (Set1_Span + R_tube)); % Interférence Corps
Efficiency_Factor = 0.8;   % Pertes de fente (rendement volet)
Alpha_Stall_Deg = 18.0;    % Angle de décrochage estimé (Degrés)
Alpha_Stall = deg2rad(Alpha_Stall_Deg); % Angle de décrochage (Radians)

fprintf('\n--- GÉNÉRATION DE LA MAP DYNAMIQUE SET 1 (0 -> Mach 0.5) ---\n');

% BOUCLE DE CALCUL (Angle x Mach)
for m = 1:length(Mach_vector)
    M_curr = Mach_vector(m);
    
    % 1. Correction de Compressibilité (Prandtl-Glauert)
    beta = sqrt(1 - M_curr^2);
    
    % 2. Pente de Portance (Cla) DYNAMIQUE pour ce Mach
    % Diederich corrigé par Beta
    Cla_iso1 = (2 * pi * AR1) / (2 + sqrt(4 + (AR1 * beta)^2));
    Cla_Set1_Dyn = Cla_iso1 * K_fb1; % Pente efficace totale
    
    % 3. Modèle Polhamus pour ce Mach
    Kp = Cla_Set1_Dyn;
    Kv = 3.2; % Composante Vortex (approx constante)
    
    % Facteur d'échelle pour ce Mach
    % Transforme le Cn local (volet) en Cl global (fusée)
    % C'est ICI qu'on applique Kflap (Set1_Kflap) car on parle de CONTRÔLE
    Scale_Factor = (S_aileron_Set1 * BrasLevier_Set1 * Set1_Kflap * Efficiency_Factor) / (S_ref * D_tube);
    
    for i = 1:length(delta_rad_vector)
        inc = abs(delta_rad_vector(i));
        signe = sign(delta_rad_vector(i));
        
        if inc <= Alpha_Stall
            % Zone Linéaire + Vortex
            Cn_local = Kp*sin(inc)*(cos(inc)^2) + Kv*(sin(inc)^2)*cos(inc);
        else
            % Zone Saturation (Plateau après décrochage)
            Cn_max = Kp*sin(Alpha_Stall)*(cos(Alpha_Stall)^2) + Kv*(sin(Alpha_Stall)^2)*cos(Alpha_Stall);
            Cn_local = Cn_max;
        end
        
        % Remplissage de la matrice
        Cl_2D_Map(i, m) = signe * Cn_local * Scale_Factor;
    end
end

% C. Calcul de l'AMORTISSEMENT (Damping Clp) - MÉTHODE STATIQUE
% Pour le damping, on utilise la surface TOTALE (pas Kflap) et on linéarise à Mach 0.
% Cela donne une estimation conservative (minimum de freinage).
Cla_iso_static = (2 * pi * AR1) / (2 + sqrt(4 + AR1^2));
Cla_Set1_Static = Cla_iso_static * K_fb1;

%pour calcul defauts
Cla_Set1 = Cla_Set1_Static;

Clp_Contrib1 = -Set1_Nb * Cla_Set1_Static * (S_aileron_Set1 * BrasLevier_Set1^2) / (S_ref * D_tube^2);
Cld_Contrib1 = Cla_Set1_Static * Scale_Factor; % Juste pour info

% Affichage graphique de contrôle
fprintf('Map générée avec succès.\n');
figure(10); clf;
surf(Mach_vector, delta_deg_vector, Cl_2D_Map);
title('Map Aéro Dynamique Set 1 (Contrôle)');
xlabel('Mach'); ylabel('Angle (deg)'); zlabel('Coeff Roulis (Cl)');
shading interp; colorbar; view(45, 30);


% =========================================================================
% === SET 2 : CANARDS (MÉTHODE LINEAIRE BARROWMAN) ===
% =========================================================================
S_aileron_Set2 = Set2_Span * (Set2_Cr + Set2_Ct) / 2;
AR2 = (2 * Set2_Span)^2 / (2 * S_aileron_Set2);
Cla_iso2 = (2 * pi * AR2) / (2 + sqrt(4 + (AR2 * beta_PG)^2)); 
K_fb2 = 1 + (R_tube / (Set2_Span + R_tube));
Cla_Set2 = Cla_iso2 * K_fb2;
Ycp_local_2 = (Set2_Span / 3) * ((Set2_Cr + 2*Set2_Ct) / (Set2_Cr + Set2_Ct));
BrasLevier_Set2 = R_tube + Ycp_local_2;

Clp_Contrib2 = -Set2_Nb * Cla_Set2 * (S_aileron_Set2 * BrasLevier_Set2^2) / (S_ref * D_tube^2);
Cld_Contrib2 = 0; % Fixe


% =========================================================================
% === SET 3 : AILERONS HAUT (MÉTHODE LINEAIRE BARROWMAN) ===
% =========================================================================
S_aileron_Set3 = Set3_Span * (Set3_Cr + Set3_Ct) / 2;
AR3 = (2 * Set3_Span)^2 / (2 * S_aileron_Set3);
Cla_iso3 = (2 * pi * AR3) / (2 + sqrt(4 + (AR3 * beta_PG)^2)); 
K_fb3 = 1 + (R_tube / (Set3_Span + R_tube));
Cla_Set3 = Cla_iso3 * K_fb3;
Ycp_local_3 = (Set3_Span / 3) * ((Set3_Cr + 2*Set3_Ct) / (Set3_Cr + Set3_Ct));
BrasLevier_Set3 = R_tube + Ycp_local_3;

Clp_Contrib3 = -Set3_Nb * Cla_Set3 * (S_aileron_Set3 * BrasLevier_Set3^2) / (S_ref * D_tube^2);
Cld_Contrib3 = 0; % Fixe


%% --- 3. AGRÉGATION FINALE (DYNAMIQUE GLOBALE) ---

% Somme des amortissements (Tous les sets freinent la rotation)
Clp_Total_Adim = Clp_Contrib1 + Clp_Contrib2 + Clp_Contrib3;

% Pour le contrôle, Simulink utilise la "2-D Lookup Table" (Cl_2D_Map).
% Cld_Total_Adim ne sert plus qu'à titre indicatif.
Cld_Total_Adim = Cld_Contrib1 + Cld_Contrib2 + Cld_Contrib3;

% Mise à l'échelle pour les blocs Gain de Simulink (AMORTISSEMENT)
% Le bloc Amortissement attend : T = q * (Clp_group) * (p * D / 2V)
Clp_group = abs(Clp_Total_Adim * S_ref * D_tube);

% Paramètres d'Inertie
I_x = 0.0072; % Kg.m^2 (Inertie Roulis corrigée)


%% --- 4. GAINS DU CONTRÔLEUR PID ---
% Points de rupture pour la pression dynamique 'q' (en Pa)
q_breakpoints = [ 0, 1000, 3000, 6000, 12000 ];

% Gains PID (Gain Scheduling)
Kp_data = [12.0, 8.0, 3.0, 1.0, 0.4];
Ki_data = [0.0, 0.0, 0.0, 0.0, 0.0];
Kd_data = [8.0, 6.0, 3.0, 1.0, 0.4];


%% --- 5. AFFICHAGE POUR VÉRIFICATION (CORRIGÉ 2D) ---
fprintf('\n==============================================\n');
fprintf('   PARAMÈTRES AÉRODYNAMIQUES DYNAMIQUES\n');
fprintf('==============================================\n');

fprintf('--- Map 2D (Contrôle) ---\n');
fprintf('Plage Mach          : 0.00 à 0.50\n');
fprintf('Angle max linéaire  : ~10 deg\n');
fprintf('Début décrochage    : %.1f deg\n', Alpha_Stall_Deg);

% Vérification : Cl max à Mach 0.385
idx_check = find(abs(Mach_vector - 0.385) < 0.006, 1);
if ~isempty(idx_check)
    max_val = max(abs(Cl_2D_Map(:, idx_check)));
    fprintf('Cl max (à Mach 0.385): %.5f\n', max_val);
end

fprintf('\n--- Amortissement (Clp) ---\n');
fprintf('Clp Total (Adim)    : %.4f\n', Clp_Total_Adim);
fprintf('Clp Group (Simulink): %.8f m^3 (Coefficient physique)\n', Clp_group);

fprintf('\n--- Données Simulink prêtes ---\n');
fprintf('Bloc "2-D Lookup Table" doit utiliser :\n');
fprintf(' - Breakpoints 1 : delta_rad_vector\n');
fprintf(' - Breakpoints 2 : Mach_vector\n');
fprintf(' - Table Data    : Cl_2D_Map\n');
fprintf('==============================================\n');

%test coeffs openrocket --> linear so must use same approx as me (limited
%in the soft at 15deg so can assume this is the breakpoint for linearity) 

% Vecteur d'entrée (Breakpoints) en Radians [-14° à 14°]
deployment_angle = [-0.2443, -0.2269, -0.2094, -0.1920, -0.1745, -0.1571, -0.1396, -0.1222, -0.1047, -0.0873, -0.0698, -0.0524, -0.0349, -0.0175, 0.0000, 0.0175, 0.0349, 0.0524, 0.0698, 0.0873, 0.1047, 0.1222, 0.1396, 0.1571, 0.1745, 0.1920, 0.2094, 0.2269, 0.2443];

% roll forcing coefficients (symetric) pour 4 fin complete (a diviser par 4
% et multiplier par l'aspect ratio)
roll_forcing_coeffs = [2.77, 2.56, 2.37, 2.17, 1.97, 1.77, 1.56, 1.37, 1.17, 0.98, 0.78, 0.59, 0.39, 0.19, 0, 0.19, 0.39, 0.59, 0.78, 0.98, 1.17, 1.37, 1.56, 1.77, 1.97, 2.17, 2.37, 2.56, 2.77];








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




