clc
clear
close all


%%%%%%%% Квадракоптер %%%%

m=0.65;
a=0.07;
b=0.07;
l=0.15;
c=0.02;
g=9.8;
maxDroneSpeed=40;
maxMotorSpeed=8450/60*2*pi;
R=165/1000;
Ke=11/maxMotorSpeed;
Km=Ke;
Imot=12;
J = 2.5*10^(-6);
B = 1*10^(-5);
La = 50*10^(-6);
ra = 0.048;
ke = 0.0073;
kt = 0.0073;

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

maxU=11;
%%%%%%%%%%%%%%%%%%%%%%%%%% Рассчет некоторых постоянных %%%%%%%%%%%%%%%%%%%

S=l*c*4+a*b;

kf=ks*r^2*airDensity*Sblade/2; %коэффициент силы тяги
maxAirResistance=airDensity*maxDroneSpeed^2*S*0.5/2; %максимальное сопративление воздуха(не учитывается в модели)

%maxAirResistance
Ip=(1.8*r)^2*0.015/12; %момент инерции лопасти
Im=0.0135^2*0.045/2; %момент инерции мотора
Ix=m/3*a^2/12+2*0.045*l^2; %момент инерции относительно x
Iy=m/3*a^2/12+4*0.045*l^2; %момент инерции относительно y
Iz=m/3*a^2/4+2*0.045*l^2;  %момент инерции относительно z

c_a_arr = [0;0;0;0.1;0.1;0.1];
C_drag_arr = [0;0;0;1.1;1.1;1.1];
c_b_arr = [0;0;0;0;0;0];
c_d_arr = [0;0;0;0.1;0.1;0.1];
time_model_arr=[60;5;15;5;5;15];
prefix = "modeling_v_0_1";
path = pwd+"\images\"

for i=1:1
    C_drag = C_drag_arr(i);
    c_a = c_a_arr(i);
    c_b = c_b_arr(i);
    c_d = c_d_arr(i); 
    time_model=time_model_arr(i);
    
    Simulation = sim('MPC_1_QuadrotorModel.slx');

    f = figure;
    plot3(Simulation.x.Data, Simulation.y.Data, Simulation.z.Data, 'r');
    hold on
    plot3(Simulation.trajectory.Data(1,:),Simulation.trajectory.Data(2,:),Simulation.trajectory.Data(3,:),'b')
    grid on
    grid minor
    scatter3(Simulation.x.Data(end), Simulation.y.Data(end), Simulation.z.Data(end),15,'filled', 'b')
    xlabel("x, м")
    ylabel("y, м")
    zlabel("z, м")
    grid on
    ax = gca;

    ax.GridColor = [0, 0, 0];  % [R, G, B]
    %saveas(f, path+prefix+"_"+num2str(i)+".png")
    %hold off
    %close all
end