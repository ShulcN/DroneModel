function plotQuadrotorData(X, Xd, U, t, path, prefix)
    % Создаем директорию, если не существует
    if ~exist(path, 'dir')
        mkdir(path);
    end

    % --- 1. Ошибка по координатам ---
    figure;
    err = X(:,1:3) - Xd(:,1:3);
    plot(t, err(:,1), 'r', 'DisplayName', '$$e_x$$', 'LineWidth', 1.5); hold on;
    plot(t, err(:,2), 'g', 'DisplayName', '$$e_y$$', 'LineWidth', 1.5);
    plot(t, err(:,3), 'b', 'DisplayName', '$$e_z$$', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position Error [m]');
    legend('Interpreter','latex');
    title('Position Error');
    grid on;
    saveas(gcf, fullfile(path, ['error_position_' prefix '.png']));

    % --- 2. Управление ---
    figure;
    plot(t, U(:,1), 'k', 'DisplayName', '$$T$$', 'LineWidth', 1.5); hold on;
    plot(t, U(:,2), 'r', 'DisplayName', '$$\tau_{\phi}$$', 'LineWidth', 1.5);
    plot(t, U(:,3), 'g', 'DisplayName', '$$\tau_{\theta}$$', 'LineWidth', 1.5);
    plot(t, U(:,4), 'b', 'DisplayName', '$$\tau_{\psi}$$', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Control Inputs');
    legend('Interpreter','latex');
    title('Control Inputs');
    grid on;
    saveas(gcf, fullfile(path, ['control_inputs_' prefix '.png']));

    % --- 3. Углы ориентации ---
    figure;
    plot(t, X(:,7), 'r', 'DisplayName', '$$\phi$$', 'LineWidth', 1.5); hold on;
    plot(t, X(:,8), 'g', 'DisplayName', '$$\theta$$', 'LineWidth', 1.5);
    plot(t, X(:,9), 'b', 'DisplayName', '$$\psi$$', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Orientation [rad]');
    legend('Interpreter','latex');
    title('Orientation Angles');
    grid on;
    saveas(gcf, fullfile(path, ['orientation_' prefix '.png']));
end
