function dxdt = dynamics_continuous(x, u, p)
% Входы:
%   x - текущее состояние (12x1)
%   u - управление (4x1)
%   p - параметры системы
% Выход:
%   dxdt - производные состояния (12x1)

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

% Параметры системы
m = p.m; 
g = p.g;
Ix = p.Ix; Iy = p.Iy; Iz = p.Iz;

% 1. Производные угловых скоростей
dw_phi = (tau_phi + (Iy - Iz)*w_theta*w_psi)/Ix;
dw_theta = (tau_theta + (Iz - Ix)*w_phi*w_psi)/Iy;
dw_psi = (tau_psi + (Ix - Iy)*w_phi*w_theta)/Iz;

% 2. Производные углов Эйлера
dphi = w_phi + tan(theta)*(w_theta*sin(phi) + w_psi*cos(phi));
dtheta = w_theta*cos(phi) - w_psi*sin(phi);
dpsi = (sin(phi)/cos(theta))*w_theta + (cos(phi)/cos(theta))*w_psi;

% 3. Ускорения в глобальной системе координат
cth = cos(theta); sth = sin(theta);
cph = cos(phi); sph = sin(phi);
cps = cos(psi); sps = sin(psi);

ax = (T/m)*(cps*sth*cph + sps*sph);
ay = (T/m)*(sps*sth*cph - cps*sph);
az = (T/m)*(cth*cph) - g;

% Сборка вектора производных
dxdt = zeros(12,1);
dxdt(1:3) = [vx; vy; vz];     % Производные позиций
dxdt(4:6) = [ax; ay; az];     % Производные скоростей
dxdt(7:9) = [dphi; dtheta; dpsi]; % Производные углов
dxdt(10:12) = [dw_phi; dw_theta; dw_psi]; % Производные угловых скоростей
end