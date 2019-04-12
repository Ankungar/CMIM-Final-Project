%% Define one body

L1 = 0.04; % Length of link 1
body(1).m = L1*10; 
body(1).Ic = body(1).m * L1^2 / 12; 
body(1).q = [1;2;3];

L2 = 0.1; % Length of link 2
body(2).m = L2*10; 
body(2).Ic = body(2).m * L2^2 / 12;
body(2).q = [4;5;6];

L3 = 0.12; % Length of link 3
body(3).m = L3*10; 
body(3).Ic = body(3).m * L3^2 / 12; 
body(3).q = [7;8;9];

grav = [0; -9.81]; % gravitational acceleration

%% Revolute joints
% 2 connects link 1 and link 2
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0.02; 0];
revolute(1).s_j = [-0.05; 0];

% 3 connects link 2 and link 3
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0.05; 0];
revolute(2).s_j = [-0.06; 0];

%% Simple constrains
simple = [];

%% Driving constraints
driving.i = 1;
driving.k = 3;
driving.d_k = @(t) (2/3)*pi + 45 * t;
driving.d_k_t = @(t) 45;
driving.d_k_tt = @(t) 0;

%% Get mass matrix

M = mass_matrix(body);
q0 = system_coordinates(body);
F = force_vector(grav, [], body);

%% Jacobian of our constraints
Cq = constraint_dq(revolute, simple, driving, 0, q0);

%% Time to integrate it
% Note that M is constant, but F, in general, no
% We can use one of the following:
%   ode45 from Matlab
%   Euler-Cromer as introduced some time ago
%   Lets try Euler-Cromer
%acc_f = @(~, ~, ~) M\F;

MCq = [M, Cq'; Cq, zeros(size(Cq,1))];

G = zeros(5,1);

FG = [F;G];

acc_f = @(~,~,~) MCq\FG;

lambda = [zeros(5,1)];
q0 = [q0;lambda];


[t, u, v] = EulerCromer(acc_f, 2, q0, zeros(size(q0)), 0.01);

%% Now some verification plots
figure
hold on
plot(t, u(:, 2))
plot(t, u(:, 5))
plot(t, u(:, 8))

%% Add single force to the system
%sforce.f = [1; 0];
%sforce.i = 1;
%sforce.u_i = [0; 1];

%F = force_vector(grav, sforce, body, q0);

%% Second integration of the equations of motion

%acc_f = @(t, q, qp) system_accelerations(t, q, qp, M, sforce, grav, body);
%[t, u, v] = EulerCromer(acc_f, 2, q0, zeros(size(q0)), 0.001);

%% Verification plots part 2
%figure
%plot(t, u(:, 2), t, u(1, 2)-9.81 / 2 .* t .^ 2)
%figure
%plot(t, u(:, 1), t, u(1, 1) + 0.25 * t .^ 2)
%figure
%plot(t, u(:, 3))