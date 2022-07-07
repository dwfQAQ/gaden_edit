syms T F;
clc;
clear;

T = 298;
P = 101.325;
% critical volume (m3/kg)101.325
v_cA = 169;

% saturated volume (L/kmol)
v_bA = 0.285 * (v_cA ^ 1.048);
v_bAIR = 33.33;

% critical boiling temperature (Kelvin)
T_bA = 351.52;
T_bAIR = 84.35;

% molecular weight
w_A = 46.0684;
w_AIR = 28.85;

% epsilon_gasType/K
epsilon_k_air = 1.15 * T_bA;
epsilon_k_ethanol = 1.15 * T_bAIR;
epsilon_k_mixture = sqrt(epsilon_k_air * epsilon_k_ethanol);

collision_integral = 1.06036/(T/epsilon_k_mixture)^0.1561 + 0.193/exp(0.47635 * T/epsilon_k_mixture) +...
    1.03587/exp(1.52996 * T/epsilon_k_mixture) + 1.76474/exp(3.89411 * T/epsilon_k_mixture);

F = 7.47e-5 * (v_bA/v_bAIR)^3 - 3.23e-4 * (v_bA/v_bAIR)^2 - 0.0266 * (v_bA/v_bAIR) + 1.03;
% Sutherland constant
S_mixture = 1.47 * F * sqrt(T_bA * T_bAIR);

% Diffusion coefficient
% D_mixture = 100 * (8.37e-3 * (T ^ 1.5) * sqrt(1/w_A + 1/w_AIR)) /...
%    (P * (v_bA^(1/3) + v_bAIR^(1/3))^2 * (1 + S_mixture/T));

D_mixture = 100 * (8.37e-3 * power(T, 1.5) * sqrt(1/w_A + 1/w_AIR)) /...
    (P * (power(v_bA, (1/3)) + power(v_bAIR, (1/3))) * (power(v_bA, (1/3)) + power(v_bAIR, (1/3))) * (1 + S_mixture/T));
