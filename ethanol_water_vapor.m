%% environment variables
T_C = 25;
T_K = 298;
P_atm_kpa = 101.325;

%% solution variables
vol_conc = 0.75;            % liquid volume concentration (100%)
density_C2H6O = 0.7893;     % ethanol density (g/ml)
density_H2O = 1.0;          % water density (g/ml)
mol_mass_C2H6O = 46.8;      % (g/mol)
mol_mass_H2O = 18;          % (g/mol)
vol_mixture = 0.2;          % volume of liquid mixture (ml)

%% vapor pressure
vapor_pressure_H2O = 0.61121 * exp((18.678 - T_C/234.5) * (T_C / (T_C + 257.14)));  % water: Buck equation (kPa)
vapor_pressure_C2H6O = power(10, 8.20417 - 1642.89/(230.3 + T_C)) / 7.501;          % ethanol: Antoine equation (kPa)

%% mole fraction in the liquid mixture
% moles = (density * volume * vol_concentration) / molar_mass
moles_H2O = (density_H2O * vol_mixture * (1-vol_conc)) / mol_mass_H2O;
moles_C2H6O = (density_C2H6O * vol_mixture * vol_conc) / mol_mass_C2H6O;
% mole fraction
X_H2O = moles_H2O / (moles_H2O + moles_C2H6O);
X_C2H6O = moles_C2H6O / (moles_H2O + moles_C2H6O);

%% 
%%%%%%%%%% UNIFAC method %%%%%%%%%

%% lookup variables
% volume contributions: R
R_H2O = 0.92;
R_CH3 = 0.9011;
R_CH2 = 0.6744;
R_OH = 1;
% group surface area: Q
Q_H2O = 1.4;
Q_CH3 = 0.848;
Q_CH2 = 0.54;
Q_OH = 1.2;
% number of structure groups of type k in the molecule i:v_i_k
v_H2O_H2O = 1;
v_C2H6O_CH3 = 1;
v_C2H6O_CH2 = 1;
v_C2H6O_OH = 1;

%% combinatorial part of activity coefficient
% Van-der-Waals volume: r
r_H2O = v_H2O_H2O * R_H2O;
r_C2H6O = v_C2H6O_CH3 * R_CH3 + v_C2H6O_CH2 * R_CH2 + v_C2H6O_OH * R_OH;
% Van-der-Waals surface area: q
q_H2O = v_H2O_H2O * Q_H2O;
q_C2H6O = v_C2H6O_CH3 * Q_CH3 + v_C2H6O_CH2 * Q_CH2 + v_C2H6O_OH * Q_OH;
% molar weighted segment: theta 
theta_H2O = (q_H2O * X_H2O) / (q_H2O * X_H2O + q_C2H6O * X_C2H6O);
theta_C2H6O = (q_C2H6O * X_C2H6O) / (q_H2O * X_H2O + q_C2H6O * X_C2H6O);
% area fractional components: phi
phi_H2O = (r_H2O * X_H2O) / (r_H2O * X_H2O + r_C2H6O * X_C2H6O);
phi_C2H6O = (r_C2H6O * X_C2H6O) / (r_H2O * X_H2O + r_C2H6O * X_C2H6O);
% compound parameter of r, z, q: l
z = 10;
l_H2O = (z/2) * (r_H2O - q_H2O) - (r_H2O - 1);
l_C2H6O = (z/2) * (r_C2H6O - q_C2H6O) - (r_C2H6O - 1);
% combinatorial part of activity coefficient: gamma_i_C
ln_gamma_H2O_C = log(phi_H2O/X_H2O) + (z/2) * q_H2O * log(theta_H2O/phi_H2O) +...
    l_H2O - (phi_H2O/X_H2O) * (X_H2O * l_H2O + X_C2H6O * l_C2H6O);

ln_gamma_C2H6O_C = log(phi_C2H6O/X_C2H6O) + (z/2) * q_C2H6O * log(theta_C2H6O/phi_C2H6O) +...
    l_C2H6O - (phi_C2H6O/X_C2H6O) * (X_H2O * l_H2O + X_C2H6O * l_C2H6O);

%% residual part of activity coefficient
% the CH3, CH2 listed for the main group 'alkanes', which has the same set of energy interaction parameters
a_n_m = [0 986.5 1318.0; 156.4 0 353.5; 300 -229.1 0];
Chi_n_m = exp(-a_n_m./T_K);
X_CH3 = v_C2H6O_CH3 * X_C2H6O / (v_H2O_H2O * X_H2O + v_C2H6O_CH2 * X_C2H6O + v_C2H6O_CH3 * X_C2H6O + v_C2H6O_OH * X_C2H6O);
X_CH2 = v_C2H6O_CH2 * X_C2H6O / (v_H2O_H2O * X_H2O + v_C2H6O_CH2 * X_C2H6O + v_C2H6O_CH3 * X_C2H6O + v_C2H6O_OH * X_C2H6O);
X_OH = v_C2H6O_OH * X_C2H6O / (v_H2O_H2O * X_H2O + v_C2H6O_CH2 * X_C2H6O + v_C2H6O_CH3 * X_C2H6O + v_C2H6O_OH * X_C2H6O);
X_H2O1 = v_H2O_H2O * X_H2O / (v_H2O_H2O * X_H2O + v_C2H6O_CH2 * X_C2H6O + v_C2H6O_CH3 * X_C2H6O + v_C2H6O_OH * X_C2H6O);

theta_CH3 = Q_CH3 * X_CH3 / (Q_CH3 * X_CH3 + Q_CH2 * X_CH2 + Q_OH * X_OH + Q_H2O * X_H2O1);
theta_CH2 = Q_CH2 * X_CH2 / (Q_CH3 * X_CH3 + Q_CH2 * X_CH2 + Q_OH * X_OH + Q_H2O * X_H2O1);
theta_OH = Q_OH * X_OH / (Q_CH3 * X_CH3 + Q_CH2 * X_CH2 + Q_OH * X_OH + Q_H2O * X_H2O1);
theta_H2O1 = Q_H2O * X_H2O1 / (Q_CH3 * X_CH3 + Q_CH2 * X_CH2 + Q_OH * X_OH + Q_H2O * X_H2O1);

ln_Gama_CH3 = Q_CH3 * (1 - log(theta_OH * Chi_n_m(2,1) + theta_H2O1 * Chi_n_m(3,1)) -...
    (theta_OH * Chi_n_m(1,2)/(theta_CH3 *Chi_n_m(1,2) + theta_CH2 *Chi_n_m(1,2) + theta_H2O *Chi_n_m(3,2)) +...
    theta_H2O * Chi_n_m(1,3)/(theta_CH3 *Chi_n_m(1,3) + theta_CH2 *Chi_n_m(1,3) + theta_OH *Chi_n_m(2,3))));

ln_Gama_CH2 = Q_CH2 * (1 - log(theta_OH * Chi_n_m(2,1) + theta_H2O1 * Chi_n_m(3,1)) -...
    (theta_OH * Chi_n_m(1,2)/(theta_CH3 *Chi_n_m(1,2) + theta_CH2 *Chi_n_m(1,2) + theta_H2O *Chi_n_m(3,2)) +...
    theta_H2O * Chi_n_m(1,3)/(theta_CH3 *Chi_n_m(1,3) + theta_CH2 *Chi_n_m(1,3) + theta_OH *Chi_n_m(2,3))));

ln_Gama_OH = Q_OH * (1 - log(theta_CH3 * Chi_n_m(1,2) + theta_CH2 * Chi_n_m(1,2) + theta_H2O1 * Chi_n_m(3,2)) -...
    (theta_CH3 * Chi_n_m(2,1)/(theta_OH *Chi_n_m(2,1) + theta_H2O *Chi_n_m(3,1)) +...
    theta_CH2 * Chi_n_m(2,1)/(theta_OH *Chi_n_m(2,1) + theta_H2O *Chi_n_m(3,1)) +...
    theta_H2O * Chi_n_m(2,3)/(theta_CH3 *Chi_n_m(1,3) + theta_CH2 *Chi_n_m(1,3) + theta_OH *Chi_n_m(2,3))));

ln_Gama_H2O = Q_H2O * (1 - log(theta_CH3 * Chi_n_m(1,3) + theta_CH2 * Chi_n_m(1,3) + theta_OH * Chi_n_m(2,3)) -...
    (theta_CH3 * Chi_n_m(3,1)/(theta_OH *Chi_n_m(2,1) + theta_H2O *Chi_n_m(3,1)) +...
    theta_CH2 * Chi_n_m(3,1)/(theta_OH *Chi_n_m(2,1) + theta_H2O *Chi_n_m(3,1)) +...
    theta_OH * Chi_n_m(3,2)/(theta_CH3 *Chi_n_m(1,2) + theta_CH2 *Chi_n_m(1,2) + theta_H2O *Chi_n_m(3,2))));

% Gamma_i_k
ln_Gama_C2H6O_CH3 = Q_CH3 * (1 - log(theta_OH * Chi_n_m(2,1)) -...
    (theta_OH * Chi_n_m(1,2)/(theta_CH3 *Chi_n_m(1,2) + theta_CH2 *Chi_n_m(1,2) + theta_H2O *Chi_n_m(3,2))));

ln_Gama_C2H6O_CH2 = Q_CH2 * (1 - log(theta_OH * Chi_n_m(2,1)) -...
    (theta_OH * Chi_n_m(1,2)/(theta_CH3 *Chi_n_m(1,2) + theta_CH2 *Chi_n_m(1,2) + theta_H2O *Chi_n_m(3,2))));

ln_Gama_C2H6O_OH = Q_OH * (1 - log(theta_CH3 * Chi_n_m(1,2) + theta_CH2 * Chi_n_m(1,2)) -...
    (theta_CH3 * Chi_n_m(2,1)/(theta_OH *Chi_n_m(2,1) + theta_H2O *Chi_n_m(3,1)) +...
    theta_CH2 * Chi_n_m(2,1)/(theta_OH *Chi_n_m(2,1) + theta_H2O *Chi_n_m(3,1))));

ln_Gama_H2O_H2O = Q_H2O;

% residual part of activity coefficient: gamma_i_R
ln_gamma_C2H6O_R = v_C2H6O_CH3 * (ln_Gama_CH3 - ln_Gama_C2H6O_CH3) + v_C2H6O_CH2 * (ln_Gama_CH2 - ln_Gama_C2H6O_CH2) +...
    v_C2H6O_OH * (ln_Gama_OH - ln_Gama_C2H6O_OH);

ln_gamma_H2O_R = v_H2O_H2O * (ln_Gama_H2O - ln_Gama_H2O_H2O);


%% activity coefficient
ln_gamma_C2H6O = ln_gamma_C2H6O_R + ln_gamma_C2H6O_C;
ln_gamma_H2O = ln_gamma_H2O_R + ln_gamma_H2O_C;

gamma_C2H6O = log(ln_gamma_C2H6O);
gamma_H2O = log(ln_gamma_H2O);

%% mole fractions of vapor at the droplet surface 
X_s_H2O = gamma_H2O * X_H2O * (vapor_pressure_H2O / P_atm_kpa);
X_s_C2H6O = gamma_C2H6O * X_C2H6O * (vapor_pressure_C2H6O / P_atm_kpa);

%% gas law
% n/V = P/RT




