% (J)     moment of inertia of the rotor     3.2284E-6 kg.m^2
% (b)     motor viscous friction constant    3.5077E-6 Nms
% (Ke)    electromotive force constant       0.0274 V/rad/sec
% (Kt)    motor torque constant              0.0274 Nm/Amp
% (R)     electric resistance                4 ohm
% (L)     electric inductance                2.75E-6H

% J= 3.2284e-6; 
% B = 3.5077e-6; 
% Ke = 0.0274; 
% Kt = 0.0274; 
% K = Kt;
% R = 4; 
% L = 2.75e-6; 

% clear all; 

% following conf form paper: 
% Electronic Throttle Control System : Modeling, Identification and Model-
% Based Control Designs , Robert N.K.Loh, et.al. 
% Hardware: Bosch DV-E5 ETC
J = 0.0021; 
B = 0.0088; 
K = 0.383; % for both Kb and Km
Kb = K;     
Km = K; 
R  = 1.5; 
L  = 1.5e-3; 
Ks = 0.087; % load, return spring, sping constant 

Tpl = 0.396;  % pre-load torque 
% Tf = 0.284; % friction torque
% Tpl = 0;    % pre-load torque 
Tf = 0;       % friction torque

% load("sim_pedal_pos.mat");



num_p0 = Km;
den_p3 = J*L;
den_p2 = L*B + R*J;
den_p1 = L*Ks + R*B + Km*Kb;
den_p0 = R*Ks;
        
num_i2 = J;
num_i1 = B;
num_i0 = Ks;
        
        
s=tf('s');
p_motor = num_p0/(den_p3*s^3 + den_p2*s^2 + den_p1*s + den_p0);
I_motor = (num_i2*s^2+num_i1*s+num_i0)/(den_p3*s^3 + den_p2*s^2 + den_p1*s + den_p0);


% PID_control = 0.8+0.4/s; % 2018-06-08
% PID_dis=c2d(PID_control, 0.5e-3);






