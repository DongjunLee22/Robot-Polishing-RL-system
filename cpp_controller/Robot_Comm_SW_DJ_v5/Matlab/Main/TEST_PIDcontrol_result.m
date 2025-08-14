clear;
close all;
clc;

Initialization;
Figure_setup;

%%
pre_data = readtable('Data_Speedl_F_30_P_80.00_I_130.00_D_0.00.txt');
data = table2array(pre_data);

%%
t = (data(:,1) - data(1,1))*0.001;
pos = data(:,2:4);
ang = data(:,5:7);
F = data(:,26:28);
v = data(:,32);

P_input = data(:,33);
V_input = data(:,34);
P_des = data(:,35);
pid_error = data(:,36);

%%
figure;
subplot(4,1,1);
plot(t,P_input);
xlim([0 t(end)]);
xlabel('Time (s)');
ylabel('Pressure (MPa)');

subplot(4,1,2);
plot(t,abs(F(:,3)));
xlim([0 t(end)]);
xlabel('Time (s)');
ylabel('Force.Z (N)');

subplot(4,1,3);
plot(t,pid_error);
xlim([0 t(end)]);
xlabel('Time (s)');
ylabel('PID error (N)');

subplot(4,1,4);
plot(t,V_input);
xlim([0 t(end)]);
xlabel('Time (s)');
ylabel('Volts (MPa)');

%%
% figure;
% plot(t,P_send);
% hold on;
% plot(t,P_des,'r--');
% xlim([0 t(end)]);
% xlabel('Time (s)');
% ylabel('Pressure (MPa)');
% legend('P Send','P Desired');
% 
% %%
% 
% figure;
% subplot(4,1,1);
% plot(t,abs(F(:,3)));
% xlim([0 t(end)]);
% xlabel('Time (s)');
% ylabel('Force.Z (N)');
% 
% subplot(4,1,2);
% plot(t,P_send);
% xlim([0 t(end)]);
% ylim([0 0.6]);
% xlabel('Time (s)');
% ylabel('Pressure (MPa)');
% 
% subplot(4,1,3);
% plot(t,V_send);
% xlim([0 t(end)]);
% ylim([0 10]);
% xlabel('Time (s)');
% ylabel('Volts (V)');
% 
% subplot(4,1,4);
% plot(t,pid_error);
% xlim([0 t(end)]);
% xlabel('Time (s)');
% ylabel('PID error (N)');
% 
