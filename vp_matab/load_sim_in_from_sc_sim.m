clear all; 
close all; 

%% load SystemC-AMS DAT file 
% file location of systemc simulation output, can 
systemCAMS_sim = load('../vp_systemc/bin/tr_ecu_demo.dat');

ams_time = systemCAMS_sim(:,1);       % AMS simulation time
% sig_pwm =   systemCAMS_sim(:,3);    %
% sig_v   =   systemCAMS_sim(:,8);    %
% sig_i   =   systemCAMS_sim(:,5);    %
% sig_a   =   systemCAMS_sim(:,4);    %
% pedal_pos = systemCAMS_sim(:,6);    %  AMS simulation time

% if debug 
pedal_pos = systemCAMS_sim(:,2);      % 
sig_a   =   systemCAMS_sim(:,3);      %
sig_i   =   systemCAMS_sim(:,4);      %
% sig_v   =   systemCAMS_sim(:,5);      % only for debug
% sig_pwm   =   systemCAMS_sim(:,6);      % only for debug


%%  store sim-input into workspace 
sim_in (:, 1) = ams_time;
sim_in (:, 2) = pedal_pos;


%%  load dc motor parameter 
sim_demo_set_para; 


%% simulink execution 
modelName = 'etc_demo_simc_performance';
load_system(modelName)
set_param(modelName,'Profile','on')
open_system(modelName)

tic
sim(modelName); 
toc


%%  plot  compare 

figure
%
% Shaft position 
%
subplot(2,1,1)    
plot(simin.time,  simin.signals.values, 'k-'); 
hold on;
plot(ams_time, sig_a, 'r-'); 
hold on;
plot(position_simscape.time,  position_simscape.signals.values, 'b-'); 
% hold on;
% plot(position_simulink.time,  position_simulink.signals.values, 'g-');
hold off;
% ylim([-50e-6 200e-6])
%xlim ([2e-8 3e-8])
grid on; 
ylabel ('Theta(position rad)');
xlabel ('time (s)');
% legend ('SystemC-AMS', 'Simscape');

%
% current 
%
subplot(2,1,2)   
plot(ams_time, sig_i,'r-');
hold on; 
plot(current_simscape.time,  current_simscape.signals.values, 'b-'); 
% hold on; 
% plot(current_simulink.time,  current_simulink.signals.values, 'g-'); 
% ylim([0 6])
hold off; 
grid on; 
ylabel ('Current (amp)');
legend ('SystemC(TLM,AMS)', 'Simscape', 'Simulink');





