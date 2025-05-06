%% Дискретная динамика с использованием RK4
function x_next = dynamics_discrete(x, u, dt, p)
% Входы:
%   x - текущее состояние (12x1)
%   u - управление (4x1)
%   dt - шаг интегрирования
%   p - параметры системы
% Выход:
%   x_next - состояние после шага интегрирования

% Вычисление коэффициентов Рунге-Кутта
k1 = dynamics_continuous(x, u, p);
k2 = dynamics_continuous(x + 0.5*dt*k1, u, p);
k3 = dynamics_continuous(x + 0.5*dt*k2, u, p);
k4 = dynamics_continuous(x + dt*k3, u, p);

% Комбинирование коэффициентов
x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

% Применение ограничений (опционально)
% x_next(7:9) = wrapToPi(x_next(7:9)); % Для углов, если нужно
end