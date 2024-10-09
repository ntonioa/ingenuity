F = [1; 3; 999999999999];

K_lift = init.K_lift;
K_drag = init.K_drag;

omega_up = sign(F(3))*sqrt(norm(F)/(K_lift - K_drag));
omega_lo = -sign(F(3))*sqrt(norm(F)/(K_lift - K_drag));
alpha = atan(F(1)/ F(3));
beta = atan(F(2)/ F(3));

gamma = atan(sqrt(tan(alpha)^2 + tan(beta)^2));
T = [tan(alpha)*cos(gamma); tan(beta)*cos(gamma); cos(gamma)];

G = 1/sqrt(1 - sin(alpha)^2*sin(beta)^2) * [sin(alpha)*cos(beta); sin(beta)*cos(alpha); cos(alpha)*cos(beta)];

norm_F_lo_b = (K_lift-K_drag)*omega_lo^2;
norm_F_up_b = (K_lift-K_drag)*omega_up^2;

F_up_b = T*norm_F_up_b*sign(omega_up);
F_lo_b = -T*norm_F_lo_b*sign(omega_lo);

%% plot the vector

% Define the origin point
origin = [0, 0, 0];

% Plotting the vector
vector = F;
quiver3(origin(1), origin(2), origin(3), vector(1), vector(2), vector(3), 'LineWidth', 2);
hold on
vector = F1; 
quiver3(origin(1), origin(2), origin(3), vector(1), vector(2), vector(3), 'LineWidth', 2);
vector = F2;
quiver3(origin(1), origin(2), origin(3), vector(1), vector(2), vector(3), 'LineWidth', 2);

% Set plot labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Vector');

% Set plot limits
grid on;

% Show plot
hold off;

%% rot mat
W_dot = [0; 0; 0];
W = [pi/2; 0; 0];

    c1 = cos(W(1));
    s1 = sin(W(1));
    c2 = cos(W(2));
    s2 = sin(W(2));
    c3 = cos(W(3));
    s3 = sin(W(3));

    R = [c3*c2, c3*s2*s1 - s3*c1, c3*s2*c1 + s3*s1;
         s3*c2, s3*s2*s1 + c3*c1, s3*s2*c1 - c3*s1;
         -s2, c2*s1, c2*c1];

    R_dot=[- c2*s3*W_dot(3) - c3*s2*W_dot(2), s1*s3*W_dot(1) - c1*c3*W_dot(3) + c1*c3*s2*W_dot(1) + c3*c2*s1*W_dot(2) - s1*s3*s2*W_dot(3), c1*s3*W_dot(1) + c3*s1*W_dot(3) - c3*s1*s2*W_dot(1) - c1*s3*s2*W_dot(3) + c1*c3*c2*W_dot(2);
           c3*c2*W_dot(3) - s3*s2*W_dot(2), c1*s3*s2*W_dot(1) - c1*s3*W_dot(3) - c3*s1*W_dot(1) + c3*s1*s2*W_dot(3) + c2*s1*s3*W_dot(2), s1*s3*W_dot(3) - c1*c3*W_dot(1) + c1*c3*s2*W_dot(3) + c1*c2*s3*W_dot(2) - s1*s3*s2*W_dot(1);
           -c2*W_dot(2), c1*c2*W_dot(1) - s1*s2*W_dot(2), - c2*s1*W_dot(1) - c1*s2*W_dot(2)];


syms s1 c1 s2 c2 s3 c3;
rotx = [1 0 0; 0 c1 -s1; 0 s1 c1];
roty = [c2 0 s2; 0 1 0; -s2 0 c2];
rotz = [c3 -s3 0; s3 c3 0; 0 0 1];

rotz*roty*rotx;

%%
syms m d I1 I2 I3 g max_ang real positive;
u = sym('u', [3 1], 'real');
v = sym('v', [3 1], 'real');
lambda = sym('lambda', [3 1], 'real');
W = sym('W', [3 1], 'real');
W_dot = sym('W_dot', [3 1], 'real');
D = [0; 0; d];
I = [I1 0 0; 0 I2 0; 0 0 I3];
F_grav_i = [0; 0; -m*g];
F = sym('F', [3, 1]);

c1 = cos(W(1));
s1 = sin(W(1));
c2 = cos(W(2));
s2 = sin(W(2));
c3 = cos(W(3));
s3 = sin(W(3));

R = [c3*c2, c3*s2*s1 - s3*c1, c3*s2*c1 + s3*s1;
     s3*c2, s3*s2*s1 + c3*c1, s3*s2*c1 - c3*s1;
     -s2, c2*s1, c2*c1];

syms F3 real;

A = 2/m*R*[0; 0; F3];
sol1 = solve(A(3) == g, F3);

F3u = sol1 + u(3);
u(1) = W(1);
u(2) = W(2);
P_ddot = 1/m*F_grav_i + 2/m*R*[0; 0; F3];
P_ddot = subs(P_ddot, F3, F3u);

assume(u(3) == 0);
% assume(abs(u(1)) <= pi);
% assume(abs(u(2)) <= pi);
sol2 = solve(P_ddot == v, u);

%%
u = optimvar('u', [3, 1]);
W1 = u(1);
W2 = u(2);
u3 = u(3);
m = 1.8;
g = 3.69;
x0.u = [0; 0; 0];
W3 = 0;
v = [1; 1; 1];

obj = 1/2*(u(1).')*u(1);
prob = optimproblem('Objective', obj);
prob.Constraints.con1 = (2*(u3 + (g*m)/(2*cos(W1)*cos(W2)))*(sin(W1)*sin(W3) + cos(W1)*cos(W3)*sin(W2)))/m == v(1);
prob.Constraints.con2 = -(2*(u3 + (g*m)/(2*cos(W1)*cos(W2)))*(cos(W3)*sin(W1) - cos(W1)*sin(W2)*sin(W3)))/m == v(2);
prob.Constraints.con3 = (2*cos(W1)*cos(W2)*(u3 + (g*m)/(2*cos(W1)*cos(W2))))/m - g == v(3);

sol = solve(prob, x0);

%%
syms alpha beta real;
syms omega_up omega_lo u3 real;
syms d_com_lo d_com_up kl_minus_kd I1 I2 I3 real positive;
I = [I1 0 0; 0 I2 0; 0 0 I3];
W_dot = sym('W_dot', [3 1], 'real');
v = sym('v', [3, 1]);


% gamma = atan(sqrt(tan(alpha)^2 + tan(beta)^2));
% T = [tan(alpha)*cos(gamma); tan(beta)*cos(gamma); cos(gamma)];
T = [alpha; beta; 1];

F_up_b = T*kl_minus_kd*omega_up^2;
F_lo_b = T*kl_minus_kd*omega_lo^2;

tau_up_b = cross([0; 0; d_com_up], F_up_b);
tau_lo_b = cross([0; 0; d_com_lo], F_lo_b);
tau_r = [0; 0; 0.02*(norm(F_lo_b) - norm(F_up_b))];

tau = tau_up_b + tau_lo_b + tau_r;
eq = [u3; v] == [F_up_b(3) + F_lo_b(3); I\tau - inv(I)*(cross(W_dot, I*W_dot))];

solve(eq, {alpha, beta, omega_up, omega_lo});

%%
syms K real positive; % K = K_lift - K_drag
syms UP LO real positive; % UP = omega_up^2 LO = omega_lo^2
syms alpha beta real;
syms du dl real positive;

v = sym('v', [3, 1]);
syms u3 real positive;

eq = [v; u3] == [-beta*K*(du*UP + dl*LO); alpha*K*(du*UP + dl*LO); 0.02*K*(UP-LO); K*(UP+LO)];
sol = solve(eq(3:4), {UP, LO}, 'ReturnConditions', true, 'real', true);

eq1 = subs(eq, UP, sol.UP);
eq2 = subs(eq1, LO, sol.LO);

sol2 = solve(eq2(1:2), {alpha, beta}, 'ReturnConditions', true, 'real', true);

%%
Omega_b = sym('Omega_b', [3, 1]);
F_up_b = sym('F_up_b', [3, 1]);
F_lo_b = sym('F_lo_b', [3, 1]);
u = sym('u', [3, 1]);
syms m g d_com_lo d_com_up I1 I2 I3 real positive;
F_grav_i = [0; 0; -m*g];

I = [I1 0 0; 0 I2 0; 0 0 I3];

assume(F_up_b + F_lo_b == u);

Omega_b_dot = I\cross(Omega_b, I*Omega_b) + I\(cross([0; 0; d_com_lo], F_lo_b) + cross([0; 0; d_com_up], F_up_b));
