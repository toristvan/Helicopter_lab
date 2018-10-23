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
Joystick_gain_y = -1;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]

%plot Vs-verdi for helikopter i likevekt
%plot(ScopeData_y.time, ScopeData_y.signals.values)
%%%% Part 1 %%%%
Vs_star=6.88;
Vd_star=0;
K_f=-((m_c*g*l_c) - (l_h*(2*m_p*g)))/(Vs_star*l_h); 
K_1=(l_p*K_f)/(2*m_p*l_h.^2);
K_2=(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*l_h.^2));
K_3=-(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*(l_h.^2 + l_p.^2)));
%%%% Part 2/Problem 1 %%%%
K_pp=4;
%K_pd=2*sqrt(K_pp/K_1); K_pd1
%K_pd=0.5*2*sqrt(K_pp/K_1); %K_pd2
%K_pd=0.1*2*sqrt(K_pp/K_1); %K_pd3
%K_pd=0.2*sqrt(K_pp/K_1); %K_pd4
K_pd=0.2*sqrt(K_pp/K_1);
K_rp=-1.2;
%K_rp3=-1.2;
%K_rp2=-1;
%K_rp1=-0.4;

%plotting
%{
length_vector=length(pitch_1_kd5);
pc=p_c_1(2,1:length_vector);
pm_kd_1=pitch_1_kd1(2,1:length_vector);
pm_kd_2=pitch_1_kd_new(2,1:length_vector);
pm_kd_3=pitch_1_kd3(2,1:length_vector);
pm_kd_5=pitch_1_kd5(2,1:length_vector);
t=p_c_1(1,1:length_vector);

%K_pd1
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(2*sqrt(K_pp/K_1))');
pc_kd_1=p_c_1(2,1:length_vector);
plot(t,pc_kd_1,'r',t,pm_kd_1,'b');
xlabel('Time[t]');
ylabel('pitch angle[rad]');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd2
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.5*2*sqrt(K_pp/K_1))');
pc_kd_1=p_c_1(2,1:length_vector);
plot(t,pc_kd_1,'r',t,pm_kd_2,'b');
xlabel('Time[t]');
ylabel('pitch angle[rad]');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd3
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.1*2*sqrt(K_1/K_pp))');
plot(t,pc,'r',t,pm_kd_3,'b');
xlabel('Time[t]');
ylabel('pitch angle[rad]');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd4
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.2*2*sqrt(K_1/K_pp))');
plot(t,pc,'r',t,pm_kd_5,'b');
xlabel('Time[t]');
ylabel('pitch angle[rad]');
legend({'Pitch reference','Pitch Measured'});
hold off

%All
figure;
hold on
title('Pitch reference vs measured pitch');
plot(t,pc,'k',t,pm_kd_1,'c',t,pm_kd_2,'g',t,pm_kd_3,'b',t,pm_kd_5,'m','LineWidth',1.5);
xlabel('Time[s]');
ylabel('Pitch angle[rad]');
legend({'Pitch reference','Pitch Measured K_{pd1}','Pitch Measured K_{pd2}','Pitch Measured K_{pd3}', 'Pitch Measured K_{pd4}'});
hold off


trav_rate_c_step=trav_rate_c(2,1:length(trav_rate_c));
trm_kd3=travel_rate_2(2,1:length(trav_rate_c));
%tm_kd3=travel_2(2,1:length(trav_rate_c));
t_travel=trav_rate_c(1,1:length(trav_rate_c));
%Travel rate step
figure;
hold on
title('Travel rate reference vs measured travel and travel rate');
plot(t_travel,trav_rate_c_step,'k',t_travel,trm_kd3,'r','LineWidth',1.5);
xlabel('Time[t]');
ylabel('Travel angle[rad]');
legend({'Travel rate reference','Travel rate measured','Travel measured'});
hold off
%}
