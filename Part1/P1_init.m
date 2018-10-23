% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -3;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]

%%%% PART I %%%%
Vs_star=6.88;
Vd_star=0;
K_f=-((m_c*g*l_c) - (l_h*(2*m_p*g)))/(Vs_star*l_h); 
K_1=(l_p*K_f)/(2*m_p*l_h.^2);
K_2=(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*l_h.^2));
K_3=-(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*(l_h.^2 + l_p.^2)));

%plot physical behaviour of simple control
%plot(pitch, elev)
%plot(phys_behav(3,xx:yy),phys_behav(2,xx:yy))
%plot Vs-value for helicopter in equilibrium

%{
hold on
title('Vs_star equilibrium value');
plot(V_s_equil(1,12000:30878), V_s_equil(2,12000:30878));

xlabel('Time');
ylabel('V_s');
legend(['V_s']);
hold off
%}

figure;
hold on
title('Travel with feed forward control');
plot(travel(1,1:length(travel)), travel(2,1:length(travel)));
xlabel('Time [s]');
ylabel('Travel angle [rad]');
legend(['Helicopter travel, {\lambda}']);
hold off

figure;
hold on
title('Elevation with feed forward control');
plot(elevation(1,1:length(elevation)), elevation(2,1:length(elevation)));
xlabel('Time [s]');
ylabel('Elevation angle [rad]');
legend(['Helicopter elevation, $e$']);
hold off