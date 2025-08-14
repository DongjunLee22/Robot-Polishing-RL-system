clear;
close all;
clc;

Initialization;
Figure_setup;

%%
fname = 'Data_Speedl_F_30_P_80.00_I_130.00_D_0.00.txt';
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

%%
% close all;
% flag_plot = 7;
% 
% figure;
% if flag_plot == 1
%     plot(t,F(:,3));
%     ylabel('Force.Z (N)');
% elseif flag_plot == 2
%     plot(t,v);
%     ylabel('Vel.Z (mm/s)');
% elseif flag_plot == 3
%     plot(P_input_ch);
%     ylabel('Pressure Input (MPa)');
% elseif flag_plot == 4
%     plot(t,V_input_ch);
%     ylabel('Volts Input (V)');
% elseif flag_plot == 5
%     plot(t,pid_error_ch);
%     ylabel('PID Error of chamber [N]');
% end
% xlim([0 t(end)]);
% xlabel('Time (s)');

%% PID 제어 적용 구간 추출
tol      = 1e-5;

% 1) PID 제어 시작점
check_start_V_input = mean(V_input_ch(1:100));
idxOn = find(abs(V_input_ch - check_start_V_input) > tol, 1, 'first');
% idxOn = find(V_input_ch ~= check_start_V_input, 1, 'first');
t0 = t(idxOn);

% 2) PID 제어 종료점
V_input_ch_flipped = flipud(V_input_ch);
check_end_V_input = mean(V_input_ch_flipped(1:100));
pre_idxOff = find(abs(V_input_ch_flipped - check_end_V_input) > tol, 1, 'first');
idxOff = length(V_input_ch) - pre_idxOff +  1;
tf = t(idxOff);

% 3) 구간 인덱스
idxCtrl = (t >= t0) & (t <= tf);

% 4) 시간축 재설정
t_ctrl = t(idxCtrl) - t0;
F_ctrl = F(idxCtrl,:);
P_ctrl = P_input_ch(idxCtrl);
E_ctrl = pid_error_ch(idxCtrl);
V_ctrl = V_input_ch(idxCtrl);

%%
figure;
plot(t_ctrl,F_ctrl(:,3));
xlim([0 t_ctrl(end)]);


%%
F0 = F_ctrl(1,3);
F_shift = F0 - F_ctrl(:,3);
Target_F_shift = F0 - Target_F;

%%
% 시간영역 지표
[Mp, tr, ts, ess] = fun_timeDomainMetrics(t_ctrl,F_shift,Target_F_shift);

% 통합오차 지표
iae = fun_computeIAE(t_ctrl,E_ctrl);
ise = fun_computeISE(t_ctrl,E_ctrl);
itae = fun_computeITAE(t_ctrl,E_ctrl);

% 스펙트럼 분석
[f, Pxx] = fun_computeFFTAnalysis(t_ctrl,F_shift);
%%
% 결과 출력
fprintf('Overshoot: %.2f%%\n', Mp);
fprintf('Rise time: %.3fs, Settling time: %.3fs, Steady-state error: %.3f\n', tr, ts, ess);
fprintf('IAE: %.3f, ISE: %.3f, ITAE: %.3f\n', iae, ise, itae);

Fz_RMSE = rmse(Target_F_shift,F_shift);
fprintf('\nFz 접촉력에 대한 RMSE: %.4f [N]\n',Fz_RMSE);

figure;
subplot(3,1,1);
plot(t_ctrl,F_shift);
hold on;
yline(Target_F_shift,'r--','LineWidth',3);
xlim([0 t_ctrl(end)]);
xlabel('Time (s)');
ylabel(sprintf('Force.Z (N)\n(Normalized)'));
hold off;

subplot(3,1,2);
plot(t_ctrl,V_ctrl);
xlim([0 t_ctrl(end)]);
xlabel('Time (s)');
ylabel('Volts (V)');

subplot(3,1,3);
plot(t_ctrl,E_ctrl);
xlim([0 t_ctrl(end)]);
xlabel('Time (s)');
ylabel('PID error (N)');
