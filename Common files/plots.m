%%%%PLots from assignments%%%%

%%%% PART I %%%%
%{
hold on
title('Vs_star equilibrium value');
plot(V_s_equil(1,12000:30878), V_s_equil(2,12000:30878));

xlabel('Time/s');
ylabel('Voltage/V');
legend(['V_s']);
hold off
%}

figure;
hold on
title('Travel with feed forward control');
plot(travel(1,1:length(travel)), travel(2,1:length(travel)));
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('${\lambda}/rad$', 'Interpreter', 'latex');
legend('Travel');
hold off

figure;
hold on
title('Elevation with feed forward control');
plot(elevation(1,1:length(elevation)), elevation(2,1:length(elevation)));
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$e/rad$', 'Interpreter', 'latex');
legend('Elevation');
hold off

%%%% PART II %%%%
%%% Problem 1 %%%

length_vector=length(pitch_1_kd5);
pc=p_c_1(2,1:length_vector);
pm_kd_1=pitch_1_kd1(2,1:length_vector);
pm_kd_2=pitch_1_kd_new(2,1:length_vector);
pm_kd_3=pitch_1_kd3(2,1:length_vector);
pm_kd_5=pitch_1_kd5(2,1:length_vector);
t=p_c_1(1,1:length_vector);

%All individually
%{
%K_pd1
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(2*sqrt(K_pp/K_1))');
pc_kd_1=p_c_1(2,1:length_vector);
plot(t,pc_kd_1,'r',t,pm_kd_1,'b');
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd2
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.5*2*sqrt(K_pp/K_1))');
pc_kd_1=p_c_1(2,1:length_vector);
plot(t,pc_kd_1,'r',t,pm_kd_2,'b');
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd3
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.1*2*sqrt(K_1/K_pp))');
plot(t,pc,'r',t,pm_kd_3,'b');
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd4
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.25*2*sqrt(K_1/K_pp))');
plot(t,pc,'r',t,pm_kd_4,'b');
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference','Pitch Measured'});
hold off

%K_pd5
figure;
hold on
title('Pitch reference vs measured pitch with K_pd=(0.2*2*sqrt(K_1/K_pp))');
plot(t,pc,'r',t,pm_kd_5,'b');
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference','Pitch Measured'});
hold off
%}
%All
figure;
hold on
title('Pitch reference vs measured pitch');
plot(t,pc,'k',t,pm_kd_1,'c',t,pm_kd_2,'g',t,pm_kd_3,'b',t,pm_kd_5,'m','LineWidth',1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference';'Pitch Measured K_pd1';'Pitch Measured K_pd2';'Pitch Measured K_pd3'; 'Pitch Measured K_pd4'});
hold off

%%% Problem 2 %%%
trav_rate_c_step=trav_rate_c(2,1:length(trav_rate_c));
trm_kd3=travel_rate_2(2,1:length(trav_rate_c));
%tm_kd3=travel_2(2,1:length(trav_rate_c));
t_travel=trav_rate_c(1,1:length(trav_rate_c));
%Travel rate step
figure;
hold on
title('Travel rate reference vs measured travel and travel rate');
plot(t_travel,trav_rate_c_step,'k',t_travel,trm_kd3,'r','LineWidth',1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{\lambda}/rad/s$', 'Interpreter', 'latex');
legend({'Travel rate reference','Measured travel rate'});
hold off


%%%% PART III %%%%
%%% Problem 3 %%%
%Pitch
pitch_ref_P=p_c_P(2,1:length(p_c_PI));
pitch_meas_P=p_m_P(2,1:length(p_c_PI));
pitch_ref_PI=pitch_ref_P;
pitch_meas_PI=p_m_PI(2,1:length(p_c_PI));
t=p_c_P(1,1:length(p_c_PI));
figure;
hold on
title('Pitch step response for P and PI regulator');
plot(t,pitch_ref_P,'k',t,pitch_meas_P,'b',t,pitch_meas_PI,'r','LineWidth',1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Pitch reference','Measured pitch with P regulator','Measured pitch with PI regulator'});
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
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{e}/rad/s$', 'Interpreter', 'latex');
legend({'Measured elevation rate with P regulator','Measured elevation rate with PI regulator','Elevation rate reference'});
hold off

%%%% PART IV %%%%
%%% Problem 1 %%%


%%% Problem 2 %%%
%{
%Poles
%PI
figure;
hold on
title('Poles in system with PI regulator');
scatter(real(PI_poles),imag(PI_poles));
grid on
xlabel('Real value');
ylabel('Imaginary value');
hold off
%P
figure;
hold on
title('Poles in system with P regulator');
grid on
scatter(real(P_poles),imag(P_poles));
xlabel('Real value');
ylabel('Imaginary value');
hold off
%real placed poles for L-matrix p, e and \lambda, using poles from PI
%system
figure;
hold on
title('Placed poles to make L matrix for system with PI regulator and observer');
grid on
scatter(real(poles_real),imag(poles_real));
xlabel('Real value');
ylabel('Imaginary value');
hold off
%real placed poles for L-matrix p, e and \lambda, using poles from P
figure;
hold on
title('Placed poles to make L matrix for system with P regulator and observer');
grid on
scatter(real(poles_real_p),imag(poles_real_p));
xlabel('Real value');
ylabel('Imaginary value');
hold off
%}
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
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Measured pitch','Estimated pitch'});
hold off
figure;
hold on
title('Measured pitch rate and estimated pitch rate with PI regulator and observer');
plot(t_r,pitch_rate_meas_real_PI,'b',t_r,pitch_rate_meas_est_PI,'-.r','LineWidth',1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{p}/rad/s$', 'Interpreter', 'latex');
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
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Measured pitch','Estimated pitch'});
hold off
figure;
hold on
title('Measured pitch rate and estimated pitch rate with P regulator and observer');
plot(t_r_p,pitch_rate_meas_real_P,'b',t_r_p,pitch_rate_meas_est_P,'-.r','LineWidth',1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{p}/rad/s$', 'Interpreter', 'latex');
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
%Poles
%real placed poles for L-matrix e and \lambda with poles from PI
%{
figure;
hold on
title({'Placed poles to make L matrix for system with PI regulator and observer';'With observer not using measured pitch'});
grid on
scatter(real(poles_real_elev_trav),imag(poles_real_elev_trav));
xlabel('Real value');
ylabel('Imaginary value');
hold off
%}
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
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$p/rad$', 'Interpreter', 'latex');
legend({'Measured pitch','Estimated pitch'});
hold off
figure;
hold on
title({'Measured pitch rate and estimated pitch rate with PI regulator and observer.';' With observer not using measured pitch'});
plot(t_r,pitch_rate_meas_real_PI,'b',t_r,pitch_rate_meas_est_PI,'-.r','LineWidth',1.5);
xlabel('$Time/s$', 'Interpreter', 'latex');
ylabel('$\dot{p}/rad/s$', 'Interpreter', 'latex');
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
