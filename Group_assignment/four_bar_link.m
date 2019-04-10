
%% Initial Value
BD = sqrt(4^2 + 10^2 - 2*4*10*cosd(120));

gamma = acosd((BD^2 - 10^2 - 12^2) / (-2*12*10));

alpha = asind(10/BD*sind(gamma));

beta = asind(4/BD*sind(120));

zeta = alpha + beta;

teta = 360 - 120 - gamma - alpha - beta;

AG2 = sqrt(4^2 + 5^2 - 2*4*5*cosd(teta));

eta = asind(5/AG2*sind(teta));

omega = 120 - eta;

xG2 = AG2*cosd(omega);

yG2 = AG2*sind(omega);

delta = teta - (360 - 120 -90 - 90);

%% Coordinates
% ground left
qA = [0; 0; 0];
% crank
qG1 = [-2*sind(30) 
    2*cosd(30)
    deg2rad(30)];
% link 1
qG2 = [xG2
    yG2
    deg2rad(delta)];
% link 2
qG3 = [10 - 6*cosd(zeta)
    6*sind(zeta)
    deg2rad(90-zeta)];
% ground right
qD = [10; 0; 0];

q_0 = [qA; qG1; qG2; qG3; qD];

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0; -2];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0; 2];
revolute(2).s_j = [-5; 0];

% 3 connects link and link
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [5; 0];
revolute(3).s_j = [0; 6];

% 4 connects link and ground
revolute(4).i = 4;
revolute(4).j = 5;
revolute(4).s_i = [0; -6];
revolute(4).s_j = [0; 0];

% Check revolute joint constraints
r = revolute(4)
C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% Three simple joints to fix the ground origin
simple(4).i = 5;
simple(4).k = 1;
simple(4).c_k = 10;

simple(5).i = 5;
simple(5).k = 2;
simple(5).c_k = 0;

simple(6).i = 5;
simple(6).k = 3;
simple(6).c_k = 0;

% check simple constraints
for s = simple
    C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
end

%% driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) -2/3*pi + 0.5*t;

% Verify
d = driving(1);
C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

%% Verify constraint function
clc
C = constraint(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using fsolve
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
[T, Q] = position_fsolve(C_fun, 1, q_0, 0.1);

plot(Q(:,1), Q(:,2));

%% Some verification plots
plot(Q(:, 1), Q(:, 2), ... 
    Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    Q(:, 13), Q(:, 14), '*' , 'LineWidth', 2);
axis equal

