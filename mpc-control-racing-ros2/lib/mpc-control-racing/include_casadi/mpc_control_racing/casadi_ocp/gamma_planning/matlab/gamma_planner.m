% Car race along a track
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.

% this is the working one available on ROS2
% the parameters and setup is little bit different from Roland's
% to compile the race_car_nlpsol_autogen.c, do "gcc -fPIC -shared -O3 race_car_nlpsol_autogen.c -o race_car_nlpsol_autogen.so"
% then add the .so file to mpc-control-racing-ros2/lib/mpc-control-racing/include_casadi/mpc_control_racing/spline_following_yang/mx_gen_nlpsol/race_car_nlpsol_autogen.so
% modify the absolute path in mpc-control-racing-ros2/src/SplineFollowingY_MGMI_Node.cpp 

close all;
[fx, fy, kappa, n_point, data_points] = la_track_spline_fit;
track_length = 6+3*pi/2;

start_gamma = 0.2;

import casadi.*

T = 1; % Time horizon
N = 20; % number of control intervals
nx = 2;
nu = 1;

% Declare model variables
gamma = MX.sym('gamma');
gamma_dot = MX.sym('gamma_dot');
state = [gamma; gamma_dot];

gamma_input = MX.sym('gamma_input');
input = gamma_input;

% Model equations
state_dot = [gamma_dot; 
             gamma_input];

% costs 
L = gamma_input*gamma_input - gamma_dot;

% Continuous time dynamics
f = Function('f', {state, input}, {state_dot, L});

% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
DT = T/N;
X0 = MX.sym('X0', nx);
U = MX.sym('U', nu);
X = X0;
Q = 0;
[k1, k1_q] = f(X, U);
[k2, k2_q] = f(X + DT/2 * k1, U);
[k3, k3_q] = f(X + DT/2 * k2, U);
[k4, k4_q] = f(X + DT * k3, U);
X = X + DT/6*(k1 +2*k2 +2*k3 +k4);
Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

state_initial = [start_gamma];
% "Lift" initial conditions

Xk = MX.sym('X0', nx);
w = {w{:}, Xk};
lbw = [lbw; state_initial; 0];
ubw = [ubw; state_initial; 0.5];
w0 = [w0; state_initial; 0];

% stagewise constraints 
a_lat = 5;
a_long = 1;
v_max = 1;
state_lb = [0; 0];
state_ub = [inf; v_max];
input_lb = [-a_long];
input_ub = [a_long];

% Formulate the NLP
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], nu);
    w = {w{:}, Uk};
    lbw = [lbw; input_lb];
    ubw = [ubw; input_ub];
    w0 = [w0; zeros(nu, 1)];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J = J + Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx);
    w = [w, {Xk}];
    lbw = [lbw; state_lb];
    ubw = [ubw; state_ub];
    w0 = [w0; zeros(nx, 1)];

    % Add equality constraint
    g = [g, {Xk_end-Xk}];
    lbg = [lbg; zeros(nx, 1)];
    ubg = [ubg; zeros(nx, 1)];
    
    % Add inequality constraint
    V = track_length * Xk(2);
    g = [g, { a_lat-V*V*kappa(Xk(1)) }];
    lbg = [lbg; 0];
    ubg = [ubg; inf];
end
J = J - Xk(1);

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
jit_options = struct;
jit_options.flags = '-O3';
jit_options.verbose = true;
options = struct;
options.jit = false;
options.jit_options = jit_options;
options.compiler = 'shell';


% options.ipopt.max_iter = 45;
solver = nlpsol('solver', 'ipopt', prob, options);

 % normal generate will not work
 % will complain about type IpoptInterface and cannot compile
% solver.generate_dependencies('race_car_nlpsol_autogen.c');

% Solve the NLP
tic 
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
toc 

w_opt = full(sol.x);

%% Plot the solution
gamma_opt = w_opt(1:nx+nu:end);
gamma_dot_opt = w_opt(2:nx+nu:end);

gamma_input_opt = w_opt(3:nx+nu:end);
tgrid = linspace(0, T, N+1);
figure()
hold on
plot(tgrid, gamma_opt)
plot(tgrid, gamma_dot_opt)
% plot(tgrid, sqrt(a_lat./full(kappa(gamma_opt))))
plot(tgrid, (gamma_dot_opt).^2.*full(kappa(gamma_opt)))

xlabel('t')
legend('gamma', 'gamma dot', 'lat acc/track\_length^2')

figure()
hold on
stairs(tgrid, [gamma_input_opt; nan])
xlabel('t')
legend('gamma input')

figure
hold on
plot(data_points(1,2:n_point),data_points(2,2:n_point),'or')
plot(full(fx(gamma_opt)), full(fy(gamma_opt)), 'x-b')
axis equal

gamma_opt_cpp = [
     0.2 0.207314 0.214447 0.221253 0.227596 0.233486 0.239183 0.245074  0.25145 0.258434 0.266011 0.274066 0.282477 0.291122 0.299879 0.308627 0.317614 0.327592 0.338974 0.351699 0.365706
     ];
pos_opt_cpp = [
 2.09573  2.14739   2.1891  2.21972  2.23919  2.24863  2.24916  2.24074  2.22216  2.19156  2.14761  2.09031   2.0208  1.94125  1.85459  1.76431  1.66992  1.56403  1.44197  1.30417  1.15127
0.196653 0.251821 0.308958 0.365849 0.420386 0.471893  0.52208 0.573935 0.629439 0.688773 0.750473 0.811862 0.869998 0.921901 0.964769 0.996215  1.01543  1.02327  1.01958  1.00623 0.989291
    ];
plot(pos_opt_cpp(1, :), pos_opt_cpp(2, :), '^-k')
% plot(full(fx(gamma_opt_cpp)), full(fy(gamma_opt_cpp)), 'x-b')

disp('initial state')
disp([state_initial;gamma_dot_opt(1)])
disp('objective')
disp(full(sol.f))