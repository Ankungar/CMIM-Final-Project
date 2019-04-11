%% Define one body
L1 = 0.04; % Length of link 1
body(1).m = L1*10; 
body(1).Ic = body(1).m * L1^2 / 12; 
body(1).q = [1;2;3];

L2 = 01; % Length of link 2
body(2).m = L2*10; 
body(2).Ic = body(2).m * L2^2 / 12;
body(2).q = [4;5;6];

L3 = 0.12; % Length of link 3
body(3).m = L3*10; 
body(3).Ic = body(3).m * L3^2 / 12; 
body(3).q = [7;8;9];

grav = [0; -9.81]; % gravitational acceleration

%% Get mass matrix

M = mass_matrix(body);
q0 = system_coordinates(body);
F = force_vector(grav, [], body);

%% Time to integrate it
% Note that M is constant, but F, in general, no
% We can use one of the following:
%   ode45 from Matlab
%   Euler-Cromer as introduced some time ago
%   Lets try Euler-Cromer
acc_f = @(~, ~, ~) M\F;
[t, u, v] = EulerCromer(acc_f, 2, q0, zeros(size(q0)), 0.01);

%% Now some verification plots
plot(t, u(:, 2), t, u(1, 2)-9.81 / 2 .* t .^ 2)

%% Add single force to the system
sforce.f = [1; 0];
sforce.i = 1;
sforce.u_i = [0; 1];

F = force_vector(grav, sforce, body, q0);

%% Second integration of the equations of motion

acc_f = @(t, q, qp) system_accelerations(t, q, qp, M, sforce, grav, body);
[t, u, v] = EulerCromer(acc_f, 2, q0, zeros(size(q0)), 0.001);

%% Verification plots part 2
plot(t, u(:, 2), t, u(1, 2)-9.81 / 2 .* t .^ 2)
plot(t, u(:, 1), t, u(1, 1) + 0.25 * t .^ 2)
plot(t, u(:, 3))