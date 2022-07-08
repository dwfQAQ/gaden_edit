clc;
clear;
%% environment variables
T_C = 25;
T_K = 298;
P_atm_kpa = 101.325;

%% liquid variables
vol_conc = 0.75;            % liquid volume concentration (100%)
density_C2H6O = 0.7893;     % ethanol density (g/ml)
density_H2O = 1.0;          % water density (g/ml)
mol_mass_C2H6O = 46.8;      % (g/mol)
mol_mass_H2O = 18;          % (g/mol)
vol_mixture = 0.2;          % volume of liquid mixture (ml)

%% mole fraction in the liquid mixture
% moles = (density * volume * vol_concentration) / molar_mass
moles_H2O = (density_H2O * vol_mixture * (1-vol_conc)) / mol_mass_H2O;
moles_C2H6O = (density_C2H6O * vol_mixture * vol_conc) / mol_mass_C2H6O;
% mole fraction
X_H2O = moles_H2O / (moles_H2O + moles_C2H6O);
X_C2H6O = moles_C2H6O / (moles_H2O + moles_C2H6O);
x_c = [1 0];
%% vapor pressure
vapor_pressure_H2O = 0.61121 * exp((18.678 - T_C/234.5) * (T_C / (T_C + 257.14)));  % water: Buck equation (kPa)
vapor_pressure_C2H6O = power(10, 8.20417 - 1642.89/(230.3 + T_C)) / 7.501;          % ethanol: Antoine equation (kPa)
vapor_pressure_ideal = vapor_pressure_H2O * X_H2O + vapor_pressure_C2H6O * X_C2H6O; % ideal vapor pressure of mixture (kPa)

%% 
%%%%%%%%%% UNIFAC method %%%%%%%%%
%% calculate parameters
N = 2;              % number of components
M = 4;         % number of structure groups in each component [C2H6O, H2O]
v = [1, 1, 1, 0;...    % number of each structure group type in each component
    0, 0, 0, 1];
R = [0.9011 0.6744 1 0.92];     % volume contributions
Q = [0.848 0.54 1.2 1.4];       % group surface area
a_mn = [0 0 986.5 1318.0; 0 0 986.5 1318.0; 156.4 156.4 0 353.5; 300 300 -229.1 0];

%% combinatorial
r = zeros(1,N);     % Van-der-Waals volume: r
q = zeros(1,N);     % Van-der-Waals surface area: q
phi_c = zeros(1,N);
theta_c = zeros(1,N);
ln_gamma_i_c = zeros(1,N);

z = 10;
l = zeros(1,N);
sum_xl = 0;
sum_phi_c = 0;
sum_theta_c = 0;

for i = 1:N
    for k = 1:M
       r(i) = r(i) + v(i, k) * R(k);
       q(i) = q(i) + v(i, k) * Q(k);
    end
    l(i) = (z/2) * (r(i) - q(i)) - (r(i) - 1);
    sum_xl = sum_xl + l(i) * x_c(i);
    sum_phi_c = sum_phi_c + r(i) * x_c(i);
    sum_theta_c = sum_theta_c + q(i) * x_c(i);
end

for i = 1:N
   phi_c(i) = r(i) * x_c(i)/sum_phi_c;
   theta_c(i) = q(i) * x_c(i)/sum_theta_c;
%    ln_gamma_i_c(i) = log(phi_c(i)/x_c(i)) + (z/2) * q(i) * log(theta_c(i)/phi_c(i)) + l(i) - (phi_c(i)/x_c(i)) * sum_xl;
   ln_gamma_i_c(i) = log(r(i)/sum_phi_c) + (z/2) * q(i) * log(q(i)*sum_phi_c/(r(i)*sum_theta_c)) + l(i) - (r(i)/sum_phi_c) * sum_xl;
end

%% residual part
phi_mn = exp(-(a_mn/T_K));

X_m = zeros(1, 4);
X_m_den = 0;
for j = 1:N
    for n = 1:M
        X_m_den = X_m_den + v(j, n) * x_c(j);
    end
end

for m = 1:M
    for j = 1:N
        X_m(m) = X_m(m) + v(j, m) * x_c(j);
    end
    X_m(m) = X_m(m)/X_m_den;
end

theta_m_den = 0;
for j = 1:M
    theta_m_den = theta_m_den + X_m(j) * Q(j);
end

theta_m = zeros(1, 4);
for m = 1:M
   theta_m(m) = Q(m) * X_m(m)/theta_m_den; 
end

sum_theta_m_phi_mk = zeros(1, 4);
sum_theta_m_phi_km = zeros(1, 4);
sum_theta_n_phi_nm = zeros(1, 4);
ln_Gamma_k = zeros(1, 4);

for m = 1:4
    for n = 1:4
        sum_theta_n_phi_nm(m) = sum_theta_n_phi_nm(m) + theta_m(n) * phi_mn(n, m);
    end
end

for k = 1:4
    for m = 1:4
       sum_theta_m_phi_mk(k) = sum_theta_m_phi_mk(k) + theta_m(m) * phi_mn(m, k);
       sum_theta_m_phi_km(k) = sum_theta_m_phi_km(k) + theta_m(m) * phi_mn(k, m)/sum_theta_n_phi_nm(m);
    end
    ln_Gamma_k(k) = Q(k) * (1 - log(sum_theta_m_phi_mk(k)) - sum_theta_m_phi_km(k));
end
%%
theta_mi = zeros(2, 4);
X_mi = zeros(2, 4);
sum_v_mi = zeros(1, 4);

% X_mi
for m = 1:M
    for i = 1:N
        sum_v_mi(m) = sum_v_mi(m) + v(i, m);
    end
end

for m = 1:M
    for i = 1:N
        X_mi(i, m) = v(i, m)/sum_v_mi(m);
    end
end

% theta_mi
sum_Q_j_Xji = zeros(1, 2);
for j = 1:M
   for i = 1:N
       sum_Q_j_Xji(i) = sum_Q_j_Xji(i) + Q(j) * X_mi(i, j);
   end
end

for m = 1:M
   for i = 1:N
      theta_mi(i, m) = Q(m) * X_mi(i, m) /  sum_Q_j_Xji(i);
   end
end

% seconde element
sum_theta_mi_phi_mk = zeros(2, 4);


% third element denominator
sum_theta_ni_phi_nm = zeros(2, 4);
for m = 1:M
   for i = 1:N
      for n = 1:M
          sum_theta_ni_phi_nm(i, m) = sum_theta_ni_phi_nm(i, m) + theta_mi(i, n) * phi_mn(n, m);
      end
   end
end

ln_Gamma_ki = zeros(2, 4);
sum_theta_mi_phi_km = zeros(2, 4);
pre_Qk = zeros(2, 4);
for k = 1:4
   for i = 1:2
      for m = 1:4
          sum_theta_mi_phi_mk(i, k) = sum_theta_mi_phi_mk(i, k) + theta_mi(i, m) * phi_mn(m, k);
          sum_theta_mi_phi_km(i, k) = sum_theta_mi_phi_km(i, k) + theta_mi(i, m) * phi_mn(k, m)/sum_theta_ni_phi_nm(i, k);
      end
      ln_Gamma_ki(i, k) = Q(k) * (1 - log(sum_theta_mi_phi_mk(i, k)) - sum_theta_mi_phi_km(i, k));
   end
end

ln_gamma_i_R = zeros(1, 2);
for i = 1:2
   for k = 1:4
       ln_gamma_i_R(i) = ln_gamma_i_R(i) + v(i, k) * (ln_Gamma_k(k) - ln_Gamma_ki(i, k));
   end
end

ln_gamma_i = zeros(1, 2);
for i = 1:N
   ln_gamma_i(i) = ln_gamma_i_c(i) + ln_gamma_i_R(i);
end

gamma_i = exp(ln_gamma_i)
gamma_C2H6O = gamma_i(1);
gamma_H2O = gamma_i(2);

%% mole fractions of vapor at the ethanol/water droplet surface 
% X_s_H2O = gamma_H2O * X_H2O * (vapor_pressure_H2O / P_atm_kpa);
% X_s_C2H6O = gamma_C2H6O * X_C2H6O * (vapor_pressure_C2H6O / P_atm_kpa);

X_s_H2O = gamma_H2O * X_H2O ;
X_s_C2H6O = gamma_C2H6O * X_C2H6O;
%% vapor pressure with activity coefficient
vapor_pressure_mixture_ac = vapor_pressure_C2H6O * X_s_C2H6O + vapor_pressure_H2O * X_s_H2O;

vapor_pressure_mixture_ac


