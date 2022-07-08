clc;
clear;

vol_conc = 0:0.05:1;
vapor_pressure_ac = zeros(1, 21);
vapor_pressure_id = zeros(1, 21);
for i = 1: 21
    [vapor_pressure_id(i), vapor_pressure_ac(i)] = ethanol_water_vapor(vol_conc(i));

end

figure(1);
plot(vol_conc, vapor_pressure_ac);
hold on;
plot(vol_conc, vapor_pressure_id);
legend("ac", "id");

