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
Joystick_gain_x = 2; %change this
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
%%%% PART I %%%%
Vs_star=6.88;
Vd_star=0;
K_f=-((m_c*g*l_c) - (l_h*(2*m_p*g)))/(Vs_star*l_h); 
K_1=(l_p*K_f)/(2*m_p*l_h.^2);
K_2=(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*l_h.^2));
K_3=-(l_h*K_f)/((m_c*l_c.^2)+(2*m_p*(l_h.^2 + l_p.^2)));
%%%% PART II %%%%
%%%% Problem 1 %%%%
K_pp=4;
K_pd=0.2*(2*sqrt(K_1*K_pp));
K_rp=-0.4;


%%%% Part III %%%%
%%%% Problem 1 %%%%
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

C_P=[1 0 0;
    0 0 1];

P_P= inv(C_P*inv(B_P*K_P -A_P)*B_P);

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
    0 3];

K_PI=lqr(A_PI,B_PI,Q_PI,R_PI);
P_PI=inv(C_P*inv(B_P*K_PI(1:2,1:3) -A_P)*B_P);
%P becomes Inf - we use P-style from last assignment

%%%%% PART IV %%%%%
%%% Problem 1 %%%%
A= [0 1 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 1;
    K_3 0 0 0 0 0];

B= [0 0;
    0 K_1;
    0 0;
    K_2 0;
    0 0;
    0 0];

C=[1 0 0 0 0 0;
   0 0 1 0 0 0;
   0 0 0 0 1 0];

%%%% Problem 2 %%%%
%%%% Finding system poles %%%%
%PI_system%
Q_L=[10 0 0 0 0;
    0 10 0 0 0;
    0 0 100 0 0;
    0 0 0 10 0;
    0 0 0 0 100]; 
R_L=[0.1 0;
    0 300000000];
K_L=lqr(A_PI,B_PI,Q_L,R_L);
P_L=inv(C_P*inv(B_P*K_L(1:2,1:3) -A_P)*B_P);

%Poles of PI system
PI_poles=eig(A_PI-B_PI*K_L); 

%P_system%
P_poles=eig(A_P-B_P*K_P);

%Poles in fan formation
%{
r_0=max(abs(P_poles)); %min_eig
k_r=20*r_0; %tune
theta_start=9*pi/10; %tune
theta_stop=11*pi/10; %tune
steps=5; %number of poles -1;
theta_steps=theta_start:(theta_stop-theta_start)/steps:theta_stop;
poles_fan=k_r*exp(1i*theta_steps);
poles_fan(4)=conj(poles_fan(3));
poles_fan(5)=conj(poles_fan(2));
poles_fan(6)=conj(poles_fan(1));
L=transpose(place(A',C',poles_fan)); %L_fan
%}

%%%alternative creation of L; no imaginary value
%%%This is the L-matrix we use
%same as PI_poles
[~, index] = max(abs(PI_poles)); %P_poles
r_0 = PI_poles(index); %P_poles
spacing_i = 0.15;
k_r = 20*r_0; 
poles_real = 1:spacing_i:(1 + spacing_i*(size(A) - 1));
poles_real = poles_real * k_r;
L_real=transpose(place(A',C',poles_real)); %L_real
%scatter(real(poles_real),imag(poles_real));
%for P_poles
%{
[~,index_p]=max(abs(P_poles));
r_0_p=P_poles(index_p);
spacing_i_p=0.15;
k_r_p=20*r_0_p;
poles_real_p = 1:spacing_i:(1 + spacing_i*(size(A) - 1));
poles_real_p = poles_real_p * k_r;
L_real=transpose(place(A',C',poles_real_p));
%}

%%%% Problem 3 %%%%
C_elev_trav=[0 0 1 0 0 0;
   0 0 0 0 1 0];
C_pitch_elev=[1 0 0 0 0 0;
    0 0 1 0 0 0];
OBS_all=obsv(A,C); %observable
OBS_elev_trav=obsv(A,C_elev_trav); %observable
OBS_pitch_elev=obsv(A,C_pitch_elev); %not observable

%%%Alter poles for pitch and pitch rate
%L_elev_trav_real=transpose(place(A', C_elev_trav', poles_real));
%disp('L_pre scale');
%disp(L_elev_trav_real);
poles_real_elev_trav=poles_real;
poles_real_elev_trav(1)=poles_real_elev_trav(1)*0.25;
poles_real_elev_trav(2)=poles_real_elev_trav(2)*0.0001;
L_elev_trav_real=transpose(place(A', C_elev_trav', poles_real_elev_trav));
%disp('L_post_scale');
%disp(L_elev_trav_real);



%%%%PLotting
%{
%Observer with p,e,\lambda
%Pitch and pitch rate
%PI
pitch_meas_est_PI=p_m_PI_est(2,1:length(p_m_PI_est));
pitch_meas_real_PI=p_m_PI_real(2,1:length(p_m_PI_est));
pitch_rate_meas_est_PI=p_m_rate_PI_est(2,1:(length(p_m_PI_est)-1)/4);
pitch_rate_meas_real_PI=p_m_rate_PI_real(2,1:(length(p_m_PI_est)-1)/4);
t=p_m_PI_est(1,1:length(p_m_PI_est));
t_r=p_m_PI_est(1,1:(length(p_m_PI_est)-1)/4);
figure;
hold on
title('Measured pitch and estimated pitch with PI regulator and observer');
plot(t,pitch_meas_real_PI,'b',t,pitch_meas_est_PI,'-.r','LineWidth',1.5);
xlabel('Time/s');
ylabel('Pitch angle[rad]');
legend({'Measured pitch','Estimated pitch'});
hold off
figure;
hold on
title('Measured pitch rate and estimated pitch rate with PI regulator and observer');
plot(t_r,pitch_rate_meas_real_PI,'b',t_r,pitch_rate_meas_est_PI,'-.r','LineWidth',1.5);
xlabel('Time[s]');
ylabel('Pitch rate angular velcoity[rad/s]');
legend({'Measured pitch rate','Estimated pitch rate'});
hold off

%P
pitch_meas_est_P=p_m_P_est(2,1:length(p_m_P_est));
pitch_meas_real_P=p_m_P_real(2,1:length(p_m_P_est));
rate_length_P=(length(p_m_P_est)-3)/4;
pitch_rate_meas_est_P=p_m_rate_P_est(2,1:rate_length_P);
pitch_rate_meas_real_P=p_m_rate_P_real(2,1:rate_length_P);
t_p=p_m_P_est(1,1:length(p_m_P_est));
t_r_p=p_m_rate_P_est(1,1:rate_length_P);
figure;
hold on
title('Measured pitch and estimated pitch with P regulator and observer');
plot(t_p,pitch_meas_real_P,'b',t_p,pitch_meas_est_P,'-.r','LineWidth',1.5);
xlabel('Time/s');
ylabel('Pitch angle[rad]');
legend({'Measured pitch','Estimated pitch'});
hold off
figure;
hold on
title('Measured pitch rate and estimated pitch rate with P regulator and observer');
plot(t_r_p,pitch_rate_meas_real_P,'b',t_r_p,pitch_rate_meas_est_P,'-.r','LineWidth',1.5);
xlabel('Time[s]');
ylabel('Pitch rate angular velcoity[rad/s]');
legend({'Measured pitch rate','Estimated pitch rate'});
hold off


%elevation rate
%PI
length_elev_rate=(length(e_rate_m_PI_est)/4);
e_r_est=e_rate_m_PI_est(2,1:length_elev_rate);
e_r_real=e_rate_m_PI_real(2,1:length_elev_rate);
t_e=e_rate_m_PI_real(1,1:length_elev_rate);
figure;
hold on
title('Measured elevation rate and estimated elevation rate with PI regulator and observer');
plot(t_e,e_r_real,'b',t_e,e_r_est,'-.r', 'LineWidth', 1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{e}/rad/s$', 'Interpreter', 'latex');
legend({'Measured elevation rate', 'Estimated elevation rate'});
hold off

%P
length_elev_rate=(length(e_rate_m_P_est)-2)/4;
e_r_est=e_rate_m_P_est(2,1:length_elev_rate);
e_r_real=e_rate_m_P_real(2,1:length_elev_rate);
t_e=e_rate_m_P_real(1,1:length_elev_rate);
figure;
hold on
title('Measured elevation rate and estimated elevation rate with P regulator and observer');
plot(t_e,e_r_real,'b',t_e,e_r_est,'-.r', 'LineWidth', 1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{e}/rad/s$', 'Interpreter', 'latex');
legend({'Measured elevation rate', 'Estimated elevation rate'});
hold off

%%% Problem 3 %%%
%Observer with e and \lambda%
%PI
%pitch and pith rate
length_alt_rate=(length(alt_p_m_PI_est)-3)/4;
pitch_meas_est_PI=alt_p_m_PI_est(2,1:length(alt_p_m_PI_est));
pitch_meas_real_PI=alt_p_m_PI_real(2,1:length(alt_p_m_PI_est));
pitch_rate_meas_est_PI=alt_p_m_rate_PI_est(2,1:length_alt_rate);
pitch_rate_meas_real_PI=alt_p_m_rate_PI_real(2,1:length_alt_rate);
t=alt_p_m_PI_est(1,1:length(alt_p_m_PI_est));
t_r=alt_p_m_PI_est(1,1:length_alt_rate);
figure;
hold on
title({'Measured pitch and estimated pitch with PI regulator and observer.';' With observer not using measured pitch'});
plot(t,pitch_meas_real_PI,'b',t,pitch_meas_est_PI,'-.r','LineWidth',1.5);
xlabel('Time/s');
ylabel('Pitch angle[rad]');
legend({'Measured pitch','Estimated pitch'});
hold off
figure;
hold on
title({'Measured pitch rate and estimated pitch rate with PI regulator and observer.';' With observer not using measured pitch'});
plot(t_r,pitch_rate_meas_real_PI,'b',t_r,pitch_rate_meas_est_PI,'-.r','LineWidth',1.5);
xlabel('Time[s]');
ylabel('Pitch rate angular velcoity[rad/s]');
legend({'Measured pitch rate','Estimated pitch rate'});
hold off

%elevation rate
length_elev_rate=(length(alt_e_rate_m_PI_est)-3)/4;
e_r_est=alt_e_rate_m_PI_est(2,1:length_elev_rate);
e_r_real=alt_e_rate_m_PI_real(2,1:length_elev_rate);
t_e=alt_e_rate_m_PI_real(1,1:length_elev_rate);
figure;
hold on
title({'Measured elevation rate and estimated elevation rate with PI regulator and observer';'With observer not using measured pitch'});
plot(t_e,e_r_real,'b',t_e,e_r_est,'-.r', 'LineWidth', 1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{e}/rad/s$', 'Interpreter', 'latex');
legend({'Measured elevation rate', 'Estimated elevation rate'});
hold off
%}
