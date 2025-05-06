clc
clear all

% Symbolic variables
syms x1 x2 x3 x4 k l T phi theta psi

% System of equations
eq1 = x1^2*k + x2^2*k + x3^2*k + x4^2*k == T;
eq2 = (-x1^2*k + x3^2*k)*l == phi;
eq3 = (-x2^2*k + x4^2*k)*l == theta;
eq4 = (x2^2*k + x4^2*k - x1^2*k - x3^2*k) == psi;

% Solve the system
sol = solve([eq1, eq2, eq3, eq4], [x1, x2, x3, x4]);

% Display solutions
disp('Solutions for x1:');
disp(sol.x1);
disp('Solutions for x2:');
disp(sol.x2);
disp('Solutions for x3:');
disp(sol.x3);
disp('Solutions for x4:');
disp(sol.x4);

% Example with numerical values (optional)
% Substitute some example values
T_val = 10;
phi_val = 1;
theta_val = 2;
psi_val = 3;
k_val = 0.5;
l_val = 2;

% Substitute numerical values
x1_num = subs(sol.x1, [T, phi, theta, psi, k, l], [T_val, phi_val, theta_val, psi_val, k_val, l_val]);
x2_num = subs(sol.x2, [T, phi, theta, psi, k, l], [T_val, phi_val, theta_val, psi_val, k_val, l_val]);
x3_num = subs(sol.x3, [T, phi, theta, psi, k, l], [T_val, phi_val, theta_val, psi_val, k_val, l_val]);
x4_num = subs(sol.x4, [T, phi, theta, psi, k, l], [T_val, phi_val, theta_val, psi_val, k_val, l_val]);

% Display numerical solutions
disp('Numerical solutions with example values:');
disp('x1:');
disp(double(x1_num));
disp('x2:');
disp(double(x2_num));
disp('x3:');
disp(double(x3_num));
disp('x4:');
disp(double(x4_num));
