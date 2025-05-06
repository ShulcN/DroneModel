for i=1:1
    C_drag = C_drag_arr(i);
    c_a = c_a_arr(i);
    c_b = c_b_arr(i);
    c_d = c_d_arr(i);
    time_model=time_model_arr(i);
    Simulation = out;
    %Simulation = sim('MPC_1_QuadrotorModel.slx','FixedStep','0.05');
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