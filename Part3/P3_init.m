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
Joystick_gain_x = 2; %pitch
Joystick_gain_y = -1; %elev


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
%Vs_star=9.35; %heli 5
Vd_star=0;
K_f=-((m_c*g*l_c) - (l_h*(2*m_p*g)))/(Vs_star*l_h); 
K_1=(l_p*K_f)/(2*m_p*l_h.^2);
K_2=(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*l_h.^2));
K_3=-(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*(l_h.^2 + l_p.^2)));
%%%%% Part II %%%%%%
%%%% Problem 1 %%%%
K_pp=4;
K_pd=0.2*2*sqrt(K_pp/K_1));
K_rp=-0.4;


%%%%% Part III %%%%%
%%%%Problem 1 %%%%
A_P=[0 1 0; 0 0 0; 0 0 0];
B_P=[0 0; 0 K_1; K_2 0];

%%%% Problem 2 %%%%
Co= ctrb(A_P,B_P);
%rank(Co)

%Starting both R & Q as diagonal matrices

Q_P=[20 0 0;
    0 10 0;
    0 0 50];
R_P=[0.1 0;
    0 1];



K_P=lqr(A_P,B_P,Q_P,R_P);
%K_P(2,2)=10;

C_P=[1 0 0;
    0 0 1];

P_P= inv(C_P*inv(B_P*K_P - A_P)*B_P);


%%%% Problem 3 %%%%

A_PI=[0 1 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    1 0 0 0 0;
    0 0 1 0 0];

C_PI=[1 0 0 0 0;
    0 0 1 0 0];

B_PI= [0 0;
    0 K_1;
    K_2 0;
    0 0;
    0 0];

Q_PI=[10 0 0 0 0;
    0 10 0 0 0;
    0 0 100 0 0;
    0 0 0 10 0;
    0 0 0 0 100];

R_PI=[0.1 0;
    0 2];
%{
q_1 =  % Pitch
q_2 =  % Pitch rate
q_3 =  % Elevation rate
q_4 =  % Pitch integral
q_5 =  % Elevation integral
r_1 =  % V_s
r_2 =  % V_d
%}

K_PI=lqr(A_PI,B_PI,Q_PI,R_PI);
%K_PI(2,2)=10;
%K_PI(2,4)=4;
disp('K_PI')
disp(K_PI);
%disp('P_PI');
%disp(P_PI);
%disp('Ratio: (Pitch/rate)')
disp(K_P(2,1)/K_P(2,2));
%P becomes Inf - we use P from last assignment
P_PI=inv(C_P*inv(B_P*K_PI(1:2,1:3) -A_P)*B_P);
%P_PI=[0 1; 1 0];

%{
K_PI(1)=0; 
K_PI(3)=0;
K_PI(6)=0;
K_PI(7)=0;
K_PI(10)=0;
 %}
%Added J=[0 0;0 0;0 0;-1 0;0 -1];

%Plotting

%Pitch
pitch_ref_P=p_c_P(2,1:length(p_c_PI));
pitch_meas_P=p_m_P(2,1:length(p_c_PI));
pitch_ref_PI=pitch_ref_P;
pitch_meas_PI=p_m_PI(2,1:length(p_c_PI));
t=p_c_P(1,1:length(p_c_PI));
hold on
title('Pitch step response for P and PI regulator');
plot(t,pitch_ref_P,'k',t,pitch_meas_P,'b',t,pitch_meas_PI,'r','LineWidth',1.5);
xlabel('Time[s]');
ylabel('Pitch angle[rad]');
legend({'Pitch reference','Pitch measured with P regulator','Pitch measured with PI regulator'});
hold off


%Elevation
elev_rate_ref_P=e_dot_c_P(2,1:length(e_dot_c_P));
elev_rate_meas_P=e_dot_m_P(2,1:length(e_dot_c_P));
elev_rate_ref_PI=elev_rate_ref_P;
elev_rate_meas_PI=e_dot_m_PI(2,1:length(e_dot_c_P));
t_elev=e_dot_c_P(1,1:length(e_dot_c_P));
figure;
hold on
title('Elevation rate step reponse with P and PI regulator');
plot(t_elev,elev_rate_meas_P,'b',t_elev,elev_rate_meas_PI,'r','LineWidth',0.5);
plot(t_elev,elev_rate_ref_P,'k','LineWidth',3);
xlabel('Time[s]');
ylabel('Elevation rate angle[rad]');
legend({'Elevation rate measured with P regulator','Elevation rate measured with PI regulator','Elevation rate reference'});
hold off
