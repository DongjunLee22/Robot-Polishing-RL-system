clear;
close all;
clc;

%%
diameter = 94.9;
r = diameter/2;
A = pi * r^2 * 1e-3;    % [m^2]
fprintf('\t단면적 : %.3f [m^2]\n\n',A);


F_set = 140;     % 접촉력 보정값
P_corr = F_set/A * 1e-6;
fprintf('\t공압 보정값 : %f [MPa]\n\n',P_corr);


P_set = 0.2;
F_corr = P_set * 1e+6 * A * 1e-3;
fprintf('\t접촉력 보정값 : %f [N]\n\n',F_corr);