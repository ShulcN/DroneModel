function x_next = dynamics_discrete_2(x, u, dt)
% Динамика квадрокоптера, описанная системой уравнений,
% дискретизированная методом Эйлера.

% Распаковка состояния
rx = x(1); ry = x(2); rz = x(3);
vx = x(4); vy = x(5); vz = x(6);
phi = x(7); theta = x(8); psi = x(9);
w_phi = x(10); w_theta = x(11); w_psi = x(12);

% Распаковка управления
T = u(1); 
tau_phi = u(2); 
tau_theta = u(3); 
tau_psi = u(4);

% Параметры квадрокоптера
m = 0.65; 
g = 9.8;
Ix = 0.002113472222222;
Iy = Ix;
Iz = 0.004315416666667;


% Производные угловых скоростей
dw_phi = (tau_phi + (Iy - Iz) * w_theta * w_psi) / Ix;
dw_theta = (tau_theta + (Iz - Ix) * w_phi * w_psi) / Iy;
dw_psi = (tau_psi + (Ix - Iy) * w_phi * w_theta) / Iz;

w_phi = w_phi + dt*dw_phi;
w_theta = w_theta + dt*dw_theta;
w_psi = w_psi + dt*dw_psi;
% Вычисление тригонометрических функций
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta);


% Производные углов
dphi = w_phi + tan(theta) * (w_theta * sphi + w_psi * cphi);
dtheta = w_theta * cphi - w_psi * sphi;
dpsi = (sphi / cth) * w_theta + (cphi / cth) * w_psi;

phi =phi + dt * dphi;
theta = theta + dt*dtheta;
psi = psi + dt*dpsi;

% Вычисление тригонометрических функций
cphi = cos(phi); sphi = sin(phi);
epsilon = 1e-4;
cth = max(cos(theta), epsilon); sth = sin(theta);
cps = cos(psi);   sps = sin(psi);


% Ускорения
ax = (T / m) * (cps * sth * cphi + sps * sphi);
ay = (T / m) * (sps * sth * cphi - cps * sphi);
az = (T / m) * (cth * cphi) - g;

vx = vx + ax*dt;
vy = vy + ay*dt;
vz = vz + az*dt;

rx = rx + dt*vx;
ry = ry + dt*vy;
rz = rz + dt*vz;
% Интегрирование методом Эйлера
x_next = [rx; ry; rz; vx; vy; vz; phi; theta; psi; w_phi; w_theta; w_psi];
end