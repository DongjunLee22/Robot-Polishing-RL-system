clear;
close all;
clc;

Initialization;
Figure_setup;

%%
fname = 'Data_Speedl_F_30_P_10.00_I_0.00_D_0.00.csv';
data = readmatrix(fname);
Target_F = sscanf(fname, 'Data_Speedl_F_%d_P_%*f_I_%*f_D_%*f.txt', 1);

t = (data(:,1) - data(1,1))*0.001;
pos = data(:,2:4);
ang = data(:,5:7);
F = abs(data(:,26:28));
v = data(:,32);

P_input_ch = data(:,33);
V_input_ch = data(:,34);
P_input_sp = data(:,35);
V_input_sp = data(:,36);

pid_error_ch = data(:,37);

P_receive_ch = data(:,38);
V_receive_ch = data(:,39);

P_receive_sp = data(:,40);
V_receive_sp = data(:,41);

%%
figure;
subplot(3,1,1);
plot(t,F(:,3));
yline(30,'r--','LineWidth',3);
xlabel('Time [s]');
ylabel('Force.Z [N]');
legend('Actual Force','Desired Force');
xlim([0 t(end)]);
ylim([0 40]);

subplot(3,1,2);
plot(t,P_input_ch);
hold on;
plot(t,P_receive_ch);
xlabel('Time [s]');
ylabel('Pressure [MPa]');
xlim([0 t(end)]);
% ylim([0 0.4]);

subplot(3,1,3);
plot(t,V_input_ch);
hold on;
plot(t,V_receive_ch);
xlabel('Time [s]');
ylabel('Volts [V]');
xlim([0 t(end)]);
% ylim([0 0.6]);

%%
figure;
subplot(2,1,1)
plot(t,F(:,3));
xlabel('Time (s)');
ylabel('Force.Z (N)')
subplot(2,1,2);
plot(t,v);
xlabel('Time (s)');
ylabel('Vel.z (mm/s)');

