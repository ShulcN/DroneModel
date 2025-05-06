clc
clear
close all


%%%%%%%% Квадракоптер %%%%
graphic_names = ["восьмерка", "прямая", "фигура"];
m=0.65;
a=0.07;
b=0.07;
l=0.15;
c=0.02;
g=9.8;
% g=g/2;
maxDroneSpeed=40;
maxMotorSpeed=8450/60*2*pi;
R=165/1000;
Ke=11/maxMotorSpeed;
Km=Ke;
Imot=16;
J = 2.5*10^(-6);
%J = 0.005;
B = 1*10^(-5);
La = 100*10^(-6);
%La = 0.015;
ra = 0.058;
%ra = 1;
ke = 0.00734;
kt = ke;
demp = 0;
R=ra;
Ke=ke;
Km=kt;
L = La;

% B = 0;
% La = 0.00047;
% ra = R;
% ke = Km;
% kt = Ke;



rho = 1.225;
C_drag = 0;
c_a = 0;
c_b = 0;
c_d = 0; 
A_flap = [c_a, c_b, 0;
          -c_b, c_a, 0;
          0, 0, 0];
A_ind = [c_d, 0, 0;
           0, c_d, 0;
           0, 0, 0];


Sblade=0.00525; %площадь лопасти
airDensity=1.225; %плотность воздуха
ks=1; %коэффициент подъемной силы
r=0.07; %радиус лопастей

maxU=16;
%%%%%%%%%%%%%%%%%%%%%%%%%% Рассчет некоторых постоянных %%%%%%%%%%%%%%%%%%%

S=l*c*4+a*b;

kf=ks*r^2*airDensity*Sblade/2; %коэффициент силы тяги
maxAirResistance=airDensity*maxDroneSpeed^2*S*0.5/2; %максимальное сопративление воздуха(не учитывается в модели)

%maxAirResistance
Ip=(1.8*r)^2*0.015/12; %момент инерции лопасти
Im=0.0135^2*0.045/2; %момент инерции мотора
Ix=m/3*a^2/12+2*0.045*l^2; %момент инерции относительно x
Iy=m/3*a^2/12+2*0.045*l^2; %момент инерции относительно y
Iz=m/3*a^2/4+4*0.045*l^2;  %момент инерции относительно z
% Ix = 8.1e-3;        % момент инерции по x
% Iy = 8.1e-3;        % момент инерции по y
% Iz = 14.2e-3;       % момент инерции по z


c_a_arr = [0;0;0;0.1;0.1;0.1];
C_drag_arr = [0;0;0;1.1;1.1;1.1];
c_b_arr = [0;0;0;0;0;0];
c_d_arr = [0;0;0;0.1;0.1;0.1];

prefix = "modeling_v_0_1";
path = pwd+"\images\"


T = 0;

% Матрица Adrag
Adrag = -T/m * (A_ind + A_flap);

% Построение матрицы A
A = zeros(12);
A(1:3, 4:6) = eye(3);          % позиции -> скорости
A(4:6, 4:6) = Adrag;           % блок аэродинамического сопротивления
A(7:9, 10:12) = eye(3);          % блок интегральных состояний (пример)
A(4, 8) = g;
A(5, 7) = -g;
disp(A)
% Построение матрицы B
B = zeros(12,4);
B(6, 1) = 1/m;                 % управление по оси Z
B(10, 2) = 1/Ix;               % управление по крену
B(11, 3) = 1/Iy;               % управление по тангажу
B(12, 4) = 1/Iz;               % управление по рысканью

Q = diag([1 1 1 0 0 0 0 0 1 0 0 0]); % веса состояний
%Q = diag([1 1 1 1 1 1 30 30 1 0 0 0]);
Qc = diag([15 15 15 0 0 0 30 30 1 0 0 0])*100;
Qc = diag([500, 500, 500, 1, 1, 1, 1, 1, 10, 1, 1, 1]);
% Q = eye(12)*10;
%Rc = eye(4);                           % веса управлени
Rc = diag([1, 1, 1, 1]);
Rc = diag([0.1, 0.1, 0.1, 0.1]);
disp("Det Q:")
disp(det(Qc))
disp("Det R:")
disp(det(Rc))
time_model_arr=[15;5;15;5;5;15];
kv = 1300;

Mode = 1;
x0=0;
y0=0;
z0=0;
phi0=0;
theta0=0;
psi0=0;
Xd=0;
Yd=0;
Zd=0;
Psid=0;

forward_1 = 20;
forward_2 = 20;
up_down = 5;
total_time = 60;

params = struct();
params.N = 10;                     % горизонт предсказания
params.dt = 0.05;                  % шаг дискретизации, например 50 мс
params.alpha = 0.1;                % шаг градиентного спуска
params.max_iters = 10;            % количество итераций оптимизации

% Физические параметры квадрокоптера
params.m = m;                    % масса (кг)
params.g = g;                   % ускорение свободного падения
params.Ix = Ix;                  % моменты инерции
params.Iy = Iy;
params.Iz = Iz;

% Ограничения на управление
params.umin = [0; -0.1; -0.1; -0.1];  % минимальные значения [T; tau_phi; tau_theta; tau_psi]
params.umax = [15; 1; 1; 11];   % максимальные значения

u_nominal = [m*g; 0; 0; 0];  % держит квадрокоптер в воздухе
U_ref  = repmat(u_nominal', params.N, 1);  % [N x 4]\

N = params.N;
disp(N)
for i=1:1
    C_drag = C_drag_arr(i);
    c_a = c_a_arr(i);
    c_b = c_b_arr(i);
    c_d = c_d_arr(i);
    time_model=time_model_arr(i);
    return
    Simulation = sim('MPC_1_QuadrotorModel.slx','FixedStep','0.05');
    %Simulation = sim('model_for_testing_errors_2.slx');
    f = figure;
    plot3(squeeze(Simulation.x.Data), squeeze(Simulation.y.Data), squeeze(Simulation.z.Data), 'r');
    hold on
    % plot3(Simulation.trajectory.Data(1,:),Simulation.trajectory.Data(2,:),Simulation.trajectory.Data(3,:),'b--')
    %plot3(Simulation.x1.Data, Simulation.y1.Data, Simulation.z1.Data, 'g--');
    xlim([-15, 15])
    ylim([-15, 15])
    zlim([0, 10])
    grid on
    grid minor
    scatter3(Simulation.x.Data(end), Simulation.y.Data(end), Simulation.z.Data(end),15,'filled', 'b')
    xlabel("x, м")
    ylabel("y, м")
    zlabel("z, м")
    legend({"заданная траектория", "модель"})
    X = [Simulation.x.Data, Simulation.y.Data, Simulation.z.Data, Simulation.psi.Data];
    Xd = [Simulation.trajectory.Data(1,:)',Simulation.trajectory.Data(2,:)',Simulation.trajectory.Data(3,:)',Simulation.trajectory.Data(9,:)'];
    U = Simulation.U.Data;
    t = Simulation.U.Time;
    Qm = eye(4);
    Rm = eye(4);
    calculate_metrics(X, Xd, U, t, Qm, Rm)
    %title("Движение по траектории "+ graphic_names[Mode])
    grid on
    ax = gca;
    hold off
    plot(Simulation.psi.Time, Simulation.psi.Data);
    hold on
    plot(Simulation.psi.Time, Simulation.trajectory.Data(9,:)')
    ax.GridColor = [0, 0, 0];  % [R, G, B]
    %saveas(f, path+prefix+"_"+num2str(i)+".png")
    %hold off
    %close all
end