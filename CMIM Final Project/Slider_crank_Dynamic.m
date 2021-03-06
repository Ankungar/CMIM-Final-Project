%% CMIM Final Project

% Daphne van Dijken
% Mattia Cipriani
% Vojtech Pospisil

clear all
close all
clc

Slider_crank_Kinematic

%% DYNAMIC ANALYSIS
%% Define bodies
% ground
l1 = 1; 
body(1).m = 1; 
body(1).Ic = 1; 
body(1).q = q1;

% link 1
l2 = 0.2;
body(2).m = l2*2;  
body(2).Ic = body(2).m * l2^2 / 12; 
body(2).q = q2;

% link 2 
l3 = 0.5; 
body(3).m = l3*3; 
body(3).Ic = body(3).m * l3^2 / 12; 
body(3).q = q3;

% slider/ground
l4 = 1; 
body(4).m = 1; 
body(4).Ic = 1; 
body(4).q = q4;

% system of coordinates
q_0_dyn = system_coordinates(body);

%% Mass matrix
M = mass_matrix(body);

%% Force system
% gravity force
grav = [0; -9.81];

% external forces
sforce.f = [0; 0];
sforce.i = 2;
sforce.u_i = [0; 0];

% force system
F_dyn = @(q) force_vector(grav, sforce, body, q);

%% Jacobian 
Cq_fun_dyn = @(t, q) constraint_dq_dyn(revolute, simple, t, q);
 
%% Constraints
C_fun_dyn = @(t, q) constraint_dyn(revolute, simple, t, q);

% g
g = @(t ,q ,dq) constraint_dtt_dyn(revolute, simple, t, dq, q); 
alpha0 = 1;
beta0 = sqrt(2*alpha0);
g_cap = @(t, q, dq) g(t,q,dq) - beta0^2 * C_fun_dyn(t, q);

%% integration of the equations of motion with g
% acc_f = @(t, q, dq) inv(M) * F_dyn(q) + inv(M) * Cq_fun_dyn(t, q)' * inv((Cq_fun_dyn(t, q) * inv(M) * Cq_fun_dyn(t, q)')) * (g(t, q, dq) - Cq_fun_dyn(t, q) * inv(M) * F_dyn(q));
% [t, u, v] = EulerCromer(acc_f, 0.3, q_0_dyn, zeros(size(q_0_dyn)), 0.0001);
% 
% %plot the POSITIONS
% figure
% plot(u(:,4), u(:,5), ...
%     u(:,7), u(:,8), ...
%     u(:,10), u(:,11), 'LineWidth', 2);
% title('positions in dynamic with g');
% legend('crank','link','slider');
% xlabel('x [m]');
% ylabel('y [m]');
% axis equal

%% integration of the equations of motion with g_hat

% acc_f_2nd = @(t, q, dq) inv(M)*Cq_fun_dyn(t, q)'*inv((Cq_fun_dyn(t, q)*inv(M)*Cq_fun_dyn(t, q)'))*(g_cap(t, q, dq)-Cq_fun_dyn(t, q)*inv(M)*F_dyn(q)) + inv(M)*F_dyn(q);
% [t, u, v] = EulerCromer(acc_f_2nd, 0.3, q_0_dyn, zeros(size(q_0_dyn)), 0.0001);
% 
% %plot the POSITIONS
% figure
% plot(u(:,4), u(:,5), ...
%     u(:,7), u(:,8), ...
%     u(:,10), u(:,11), 'LineWidth', 2 );
% title('positions in dynamic with g-cap');
% legend('crank','link', 'slider');
% xlabel('x [m]');
% ylabel('y [m]');
% axis equal
% 
%% integration of the equations of motion with g_hat and ODE 45
% with g
[t, Q_dyn] = ode45(@(t, q)acc_ode2(M, F_dyn, Cq_fun_dyn, g, t, q) , [0:0.0001:0.3],  [q_0_dyn ;zeros(size(q_0_dyn))]);

figure
plot(Q_dyn(:,4), Q_dyn(:,5), ...
    Q_dyn(:,7), Q_dyn(:,8), ...
    Q_dyn(:,10), Q_dyn(:,11),'LineWidth', 2);
title('positions in dynamic with g with ode45');
legend('crank','link','slider');
xlabel('x [m]');
ylabel('y [m]');
axis equal

% with g_cap
[t, Q_dyn2] = ode45(@(t, q)acc_ode2(M, F_dyn, Cq_fun_dyn, g_cap, t, q) , [0:0.0001:0.3],  [q_0_dyn ;zeros(size(q_0_dyn))]);

figure
plot(Q_dyn2(:,4), Q_dyn2(:,5), ...
    Q_dyn2(:,7), Q_dyn2(:,8), ...
    Q_dyn2(:,10), Q_dyn2(:,11),'LineWidth', 2);
title('positions in dynamic with g-cap with ode45');
legend('crank','link','slider');
xlabel('x [m]');
ylabel('y [m]');
axis equal


